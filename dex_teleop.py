import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import webcam_teleop_interface as wt
import simple_ik as si
import goal_from_teleop as gt
import dex_teleop_parameters as dt
import pprint as pp
import loop_timer as lt
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation
from ikpy.chain import Chain
import urchin as urdf_loader
import os
import math
import time
from threading import Lock
from sensor_msgs.msg import JointState


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

def map_within_limits(position, lowest_position, highest_position,lower_limit, upper_limit, margin=0.1):
    # position values are always within lowest_position and highest_position and are mapped to lower_limit and upper_limit
    mapped_position = (position - lowest_position) / (highest_position - lowest_position) * (upper_limit - lower_limit) + lower_limit
    return np.clip(mapped_position, lower_limit + margin, upper_limit - margin)

class DexTeleopNode(Node):
    def __init__(self):
        super().__init__('dex_teleop_node')

        # Initialize configurations and objects
        self.max_goal_wrist_position_z = dt.goal_max_position_z
        self.min_goal_wrist_position_z = dt.goal_min_position_z

        self.center_configuration = dt.get_center_configuration()
        self.starting_configuration = dt.get_starting_configuration()
        self.simple_ik = si.SimpleIK()

        self.webcam_aruco_detector = wt.WebcamArucoDetector(tongs_prefix='right', visualize_detections=True)

        # Center wrist position (used for teleoperation origin)
        self.center_wrist_position = self.simple_ik.fk(self.center_configuration)
        self.goal_from_markers = gt.GoalFromMarkers(dt.teleop_origin, self.center_wrist_position)

        # ROS 2 Publishers
        self.trajectory_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_publisher = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.joint_command_pub = self.create_publisher(JointState, "/command", 10)

        active_links_mask = [False, True, True, True, True, True, False]
        urdf_file_name = 'giraffe.urdf'
        self.urdf = load_urdf(urdf_file_name)
        self.giraffe_chain = Chain.from_urdf_file(active_links_mask=active_links_mask, urdf_file=urdf_file_name)
        self.joint_limits = get_joint_limits(self.urdf)
        print('SimpleIK: Joint limits:', self.joint_limits)

        # Timer
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.teleop_callback)
        self.marker_timer = self.create_timer(self.timer_period, self.marker_callback)

        # Debugging
        self.print_timing = False
        self.print_goal = False
        self.publish_goal_pose_marker = True
        self.loop_timer = lt.LoopTimer()

        # Joint names
        self.joint_names = [
            'base_link_shoulder_pan_joint',
            'shoulder_pan_shoulder_lift_joint',
            'shoulder_lift_elbow_joint',
            'elbow_wrist_1_joint',
            'wrist_1_wrist_2_joint'
        ]
        self.pose_msg = PoseStamped()
        self.pose_msg.header.frame_id = "world"
        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.joint_names = self.joint_names
        self.point = JointTrajectoryPoint()
        self.point.time_from_start.sec = 1
        self.gripper_msg = JointTrajectory()
        self.gripper_msg.joint_names = ['wrist_2_gripper_joint']
        self.gripper_point = JointTrajectoryPoint()
        self.gripper_point.time_from_start.sec = 1

        # State variables
        self.previous_positions = []  # Track last sent positions to avoid duplicate commands
        self.markers = None
        self.marker_lock = Lock()

        # Smoothing filter
        self.smoothed_positions = None
        self.smoothed_marker_positions = None
        self.smoothing_alpha = 0.5  # Alpha for exponential moving average

    def marker_callback(self):
        with self.marker_lock:
            self.markers = self.webcam_aruco_detector.process_next_frame()

    def apply_smoothing(self, new_positions):
        if self.smoothed_positions is None:
            self.smoothed_positions = np.array(new_positions)
        else:
            self.smoothed_positions = (
                self.smoothing_alpha * np.array(new_positions) +
                (1 - self.smoothing_alpha) * self.smoothed_positions
            )
        return self.smoothed_positions.tolist()

    def apply_smoothing_on_marker(self, new_positions):
        if self.smoothed_marker_positions is None:
            self.smoothed_marker_positions = np.array(new_positions)
        else:
            self.smoothed_marker_positions = (
                self.smoothing_alpha * np.array(new_positions) +
                (1 - self.smoothing_alpha) * self.smoothed_marker_positions
            )
        return self.smoothed_marker_positions.tolist()

    def teleop_callback(self):
        self.loop_timer.start_of_iteration()

        # Process the next webcam frame
        with self.marker_lock:
            goal_dict = self.goal_from_markers.get_goal_dict(self.markers)

        if goal_dict is not None:
            wrist_position = goal_dict.get('wrist_position', [0.0, 0.0, 0.0])
            x, y, z = wrist_position  # Extract x and y from the position
            if z < self.min_goal_wrist_position_z or z > self.max_goal_wrist_position_z:
                return

            # Extract orientation axes
            x_axis = goal_dict.get('gripper_x_axis', [1.0, 0.0, 0.0])
            y_axis = goal_dict.get('gripper_y_axis', [0.0, 1.0, 0.0])
            z_axis = goal_dict.get('gripper_z_axis', [0.0, 0.0, 1.0])

            # Convert orientation axes to quaternion
            rotation_matrix = np.array([x_axis, y_axis, z_axis]).T

            if self.print_goal:
                self.get_logger().info(f'Goal dict:\n{pp.pformat(goal_dict)}')

            gripper_width = goal_dict.get('grip_width', None)
            lower_limit, upper_limit = self.joint_limits['wrist_2_gripper_joint']
            gripper_position = map_within_limits(gripper_width, 0.0, 1.0, lower_limit, upper_limit, 0)

            original_rpy = Rotation.from_matrix(rotation_matrix).as_euler('xyz')
            new_yaw = -np.arctan2(abs(y/8), -x)
            new_marker_rpy = [-original_rpy[1], -original_rpy[0], new_yaw]

            wrist_1_to_gripper = 0.08
            wrist_position[0] = wrist_position[0] * 1
            wrist_position[1] = wrist_position[1] * 1
            wrist_position[2] = wrist_position[2] + wrist_1_to_gripper * math.sin(new_marker_rpy[1])

            # if len(self.previous_positions) > 0 and self.positions_similar(wrist_position, self.previous_positions, 0.01):
            #     self.get_logger().debug('No significant change in positions; skipping trajectory publish.')
            #     return
            if self.publish_goal_pose_marker:
                new_marker_rotation_matrix = Rotation.from_euler('xyz', new_marker_rpy).as_matrix()
                quaternion = Rotation.from_matrix(new_marker_rotation_matrix).as_quat()

                self.pose_msg.header.stamp = self.get_clock().now().to_msg()
                smoothened_wrist_position = self.apply_smoothing_on_marker(wrist_position)
                self.pose_msg.pose.position.x = smoothened_wrist_position[0]
                self.pose_msg.pose.position.y = smoothened_wrist_position[1]
                self.pose_msg.pose.position.z = smoothened_wrist_position[2]
                self.pose_msg.pose.orientation.x = quaternion[0]
                self.pose_msg.pose.orientation.y = quaternion[1]
                self.pose_msg.pose.orientation.z = quaternion[2]
                self.pose_msg.pose.orientation.w = quaternion[3]

                self.goal_pose_publisher.publish(self.pose_msg)
                

            physical_wrist_position = [
                wrist_position[1],
                -wrist_position[0],
                wrist_position[2]
            ]

            joint_positions = self.giraffe_chain.inverse_kinematics(physical_wrist_position)

            if len(joint_positions) > 0:
                new_marker_rpy[1] += -3.1415 if new_marker_rpy[1] > 0 else 3.1415
                ordered_positions = [float(x) for x in joint_positions[1:-1]]
                ordered_positions[-1] = new_marker_rpy[0] + 1.5708
                ordered_positions[-2] -= new_marker_rpy[1]

                ordered_positions = self.apply_smoothing(ordered_positions)

                

                self.point.positions = ordered_positions
                self.trajectory_msg.points = [self.point]

                self.gripper_point.positions = [gripper_position]
                self.gripper_msg.points = [self.gripper_point]
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = self.joint_names
                joint_state.name.append("wrist_2_gripper_joint")
                joint_state.position = ordered_positions
                joint_state.position.append(gripper_position)

                self.joint_command_pub.publish(joint_state)

                # self.trajectory_publisher.publish(self.trajectory_msg)
                # self.gripper_publisher.publish(self.gripper_msg)

                self.previous_positions = wrist_position

        self.loop_timer.end_of_iteration()

        if self.print_timing:
            self.loop_timer.pretty_print()

    def positions_similar(self, positions1, positions2, threshold=0.01):
        """Check if two position lists are similar within a threshold."""
        return all(abs(p1 - p2) < threshold for p1, p2 in zip(positions1, positions2))

def main(args=None):
    rclpy.init(args=args)

    node = DexTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DexTeleopNode...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



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
