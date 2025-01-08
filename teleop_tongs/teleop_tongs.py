
import os
import math
import numpy as np
import threading
from threading import Lock
from ikpy.chain import Chain
import urchin as urdf_loader
from scipy.spatial.transform import Rotation
import time 

import teleop_tongs.simple_ik as si
import teleop_tongs.goal_from_teleop as gt
import teleop_tongs.dex_teleop_parameters as dt
import teleop_tongs.webcam_teleop_interface as wt


def timer(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time_ms = (end_time - start_time) * 1000  # Convert to milliseconds
        print(f"Execution time of {func.__name__}: {execution_time_ms:.2f} milliseconds")
        return result
    return wrapper

def load_urdf(file_name):
    if not os.path.isfile(file_name):
        print(f"\nERROR: {file_name} was not found. Please ensure the URDF file is accessible.\n")
        raise FileNotFoundError(f"{file_name} not found.")
    urdf = urdf_loader.URDF.load(file_name, lazy_load_meshes=True)
    return urdf


def get_joint_limits(urdf):
    joint_limits = {}
    for joint in urdf.actuated_joints:
        lower = float(joint.limit.lower)
        upper = float(joint.limit.upper)
        joint_limits[joint.name] = (lower, upper)
    return joint_limits


def map_within_limits(position, lowest_position, highest_position, lower_limit, upper_limit, margin=0.1):
    """
    position values are always within [lowest_position, highest_position]
    and are mapped to [lower_limit, upper_limit].
    """
    mapped_position = (
        (position - lowest_position)
        / (highest_position - lowest_position)
        * (upper_limit - lower_limit)
        + lower_limit
    )
    return np.clip(mapped_position, lower_limit + margin, upper_limit - margin)


class DexTeleop:
    def __init__(self, urdf_path='giraffe.urdf', cam_calib_path=None, degree=True, visualize_detections=False):
        # Initialize configurations and objects
        self.max_goal_wrist_position_z = dt.goal_max_position_z
        self.min_goal_wrist_position_z = dt.goal_min_position_z

        self.center_configuration = dt.get_center_configuration()
        self.starting_configuration = dt.get_starting_configuration()
        self.simple_ik = si.SimpleIK(urdf_path)

        self.webcam_aruco_detector = wt.WebcamArucoDetector(
            tongs_prefix='right',
            visualize_detections=visualize_detections,
            cam_calib_path=cam_calib_path
        )
        self.in_degree = degree
        # Center wrist position (used for teleoperation origin)
        self.center_wrist_position = self.simple_ik.fk(self.center_configuration)
        self.goal_from_markers = gt.GoalFromMarkers(dt.teleop_origin, self.center_wrist_position)

        # IKPy chain setup
        active_links_mask = [False, True, True, True, True, True, False]
        self.urdf = load_urdf(urdf_path)
        self.giraffe_chain = Chain.from_urdf_file(
            active_links_mask=active_links_mask,
            urdf_file=urdf_path
        )
        self.joint_limits = get_joint_limits(self.urdf)
        print('SimpleIK: Joint limits:', self.joint_limits)

        # State variables
        self.markers = None
        self.marker_lock = Lock()

        # Smoothing filter
        self.smoothed_positions = None
        self.smoothing_alpha = 0.2  # Alpha for exponential moving average

    def apply_smoothing(self, new_positions):
        if self.smoothed_positions is None:
            self.smoothed_positions = np.array(new_positions)
        else:
            self.smoothed_positions = (
                self.smoothing_alpha * np.array(new_positions)
                + (1 - self.smoothing_alpha) * self.smoothed_positions
            )
        return self.smoothed_positions.tolist()

    def get_goal_pose(self):
        """
        Compute IK based on detected markers and send commands to motors.
        """
        # Acquire the current markers result
        self.markers = self.webcam_aruco_detector.process_next_frame()
        goal_dict = self.goal_from_markers.get_goal_dict(self.markers)

        if goal_dict is None:
            return None
        # Extract orientation axes
        x_axis = goal_dict.get('gripper_x_axis', [1.0, 0.0, 0.0])
        y_axis = goal_dict.get('gripper_y_axis', [0.0, 1.0, 0.0])
        z_axis = goal_dict.get('gripper_z_axis', [0.0, 0.0, 1.0])

        wrist_position = goal_dict.get('wrist_position', [0.0, 0.0, 0.0])
        x, y, z = wrist_position


        # Enforce Z-bound for teleop
        if not (self.min_goal_wrist_position_z <= z <= self.max_goal_wrist_position_z):
            return None

        # Gripper width -> joint limit mapping
        gripper_width = goal_dict.get('grip_width', None)
        lower_limit, upper_limit = self.joint_limits['wrist_2_gripper_joint']
        gripper_position = map_within_limits(gripper_width, 0.0, 1.0, lower_limit, upper_limit, 0)

        # Compute orientation (roll/pitch/yaw)
        rotation_matrix = np.array([x_axis, y_axis, z_axis]).T
        original_rpy = Rotation.from_matrix(rotation_matrix).as_euler('xyz')
        new_yaw = -np.arctan2(abs(y / 8), -x)
        new_marker_rpy = [-original_rpy[1], -original_rpy[0], new_yaw]

        # Slight offset along wrist_1
        wrist_1_to_gripper = 0.08
        wrist_position[0] = wrist_position[0] * 1
        wrist_position[1] = wrist_position[1] * 1
        wrist_position[2] += wrist_1_to_gripper * math.sin(new_marker_rpy[1])

        # Re-map to physical coordinates for IK
        physical_wrist_position = [
            wrist_position[1],
            -wrist_position[0],
            wrist_position[2]
        ]

        # IK from IKPy
        joint_positions = self.giraffe_chain.inverse_kinematics(physical_wrist_position)
        if len(joint_positions) <= 0:
            return None

        # Adjust final angles
        new_marker_rpy[1] += -3.1415 if new_marker_rpy[1] > 0 else 3.1415
        ordered_positions = [float(x) for x in joint_positions[1:-1]]
        ordered_positions[-1] = new_marker_rpy[0] + 1.5708
        ordered_positions[-2] -= new_marker_rpy[1]

        # Smooth the joint positions
        ordered_positions.append(gripper_position)
        ordered_positions = self.apply_smoothing(ordered_positions)
        if self.in_degree:
            deg_positions = [x * 180 / 3.1415 for x in ordered_positions[:-1]]
            deg_positions.append(ordered_positions[-1])
            return deg_positions
        return ordered_positions

if __name__ == '__main__':
    dt = DexTeleop(urdf_path='giraffe.urdf', cam_calib_path='camera_calibration_results.yaml', degree=True, visualize_detections=True)

    while True:
        try:
            goal_pose = dt.get_goal_pose()
            if goal_pose is not None:
                print('goal_pose =', goal_pose)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(e)

##############################################################
## NOTES
##############################################################

#######################################
#
# Overview
#
# Dexterous teleoperation uses a marker dictionary representing either
# a real or virtual ArUco marker specified with respect to the
# camera's frame of reference. The marker's position controls the
# robot's wrist position via inverse kinematics (IK). The marker's
# orientation directly controls the joints of the robot's dexterous
# wrist.
#
#######################################

#######################################
#
# The following coordinate systems are important to this teleoperation
# code
#
#######################################

#######################################
# ArUco Coordinate System
#
# Origin in the middle of the ArUco marker.
#
# x-axis
# right side when looking at marker is pos
# left side when looking at marker is neg

# y-axis
# top of marker is pos
# bottom of marker is neg

# z-axis
# normal to marker surface is pos
# pointing into the marker surface is neg
#
#######################################

#######################################
# Camera Coordinate System
#
# Camera on the floor looking with the top of the camer facing away
# from the person.
#
# This configuration matches the world frame's coordinate system with
# a different origin that is mostly just translated along the x and y
# axes.
#
# Origin likely at the optical cemter of a pinhole
# model of the camera.
#
# The descriptions below describe when the robot's mobile base is at
# theta = 0 deg.
#
# x-axis
# human left is pos / robot forward is pos
# human right is neg / robot backward is neg

# y-axis
# human arm extended is neg / robot arm extended is neg
# human arm retracted is pos / robot arm retracted is pos

# z-axis
# up is positive for person and the robot
# down is negative for person and the robot
#
#######################################

#######################################
# IK World Frame Coordinate System
#
# Origin at the axis of rotation of the mobile
# base on the floor.
#
# x-axis
# human/robot left is pos
# human/robot right is neg

# y-axis
# human/robot forward is neg
# human/robot backward is pos

# z-axis
# human/robot up is pos
# human/robot down is neg
#
#######################################

#######################################
# Robot Wrist Control

# wrist yaw
#     - : deployed direction
#     0 : straight out parallel to the telescoping arm
#     + : stowed direction

# wrist pitch
#     - : up
#     0 : horizontal
#     + : down

# wrist roll
#     - : 
#     0 : horizontal
#     + :
#
#######################################

##############################################################
