import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

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

        # Declare and parse parameters
        self.declare_parameter('use_fastest_mode', False)
        self.declare_parameter('left_handed', False)

        use_fastest_mode = self.get_parameter('use_fastest_mode').value
        left_handed = self.get_parameter('left_handed').value

        # Robot speed
        self.robot_speed = 'fast' if use_fastest_mode else 'slow'
        self.get_logger().info(f'Running with robot_speed = {self.robot_speed}')

        # Initialize configurations and objects
        self.center_configuration = dt.get_center_configuration()
        self.starting_configuration = dt.get_starting_configuration()
        self.simple_ik = si.SimpleIK()

        if left_handed:
            self.webcam_aruco_detector = wt.WebcamArucoDetector(tongs_prefix='left', visualize_detections=False)
        else:
            self.webcam_aruco_detector = wt.WebcamArucoDetector(tongs_prefix='right', visualize_detections=True)

        # Center wrist position (used for teleoperation origin)
        self.center_wrist_position = self.simple_ik.fk(self.center_configuration)
        self.goal_from_markers = gt.GoalFromMarkers(dt.teleop_origin, self.center_wrist_position)

        # ROS 2 Publishers
        self.trajectory_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_publisher = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        # Marker Pose Publishers (predefined for a maximum of 8 markers)
        # self.marker_pose_publishers = {
        #     f'marker_{i}': self.create_publisher(PoseStamped, f'/marker_pose/marker_{i}', 10)
        #     for i in range(8)
        # }
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10) 
        self.origin_publisher = self.create_publisher(PoseStamped, '/origin_pose', 10) 
        active_links_mask = [False, True, True, True, True, True, False]
        urdf_file_name = '/media/rightbot/data/Projects/teleop_tong/giraffe.urdf'
        self.urdf = load_urdf(urdf_file_name)
        self.giraffe_chain = Chain.from_urdf_file(active_links_mask=active_links_mask, urdf_file=urdf_file_name)
        self.joint_limits = get_joint_limits(self.urdf)
        print('SimpleIK: Joint limits:', self.joint_limits)

        # Timer
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.teleop_callback)

        # Debugging
        self.print_timing = False
        self.print_goal = True
        self.loop_timer = lt.LoopTimer()

        # Joint names
        self.joint_names = [
            'base_link_shoulder_pan_joint',
            'shoulder_pan_shoulder_lift_joint',
            'shoulder_lift_elbow_joint',
            'elbow_wrist_1_joint',
            'wrist_1_wrist_2_joint'
        ]

        # State variables
        self.previous_positions = None  # Track last sent positions to avoid duplicate commands

    def teleop_callback(self):
        self.loop_timer.start_of_iteration()

        # Process the next webcam frame
        markers = self.webcam_aruco_detector.process_next_frame()
        goal_dict = self.goal_from_markers.get_goal_dict(markers)
        if goal_dict is not None:
            wrist_position = goal_dict.get('wrist_position', [0.0, 0.0, 0.0])

            # Extract orientation axes
            x_axis = goal_dict.get('gripper_x_axis', [1.0, 0.0, 0.0])
            y_axis = goal_dict.get('gripper_y_axis', [0.0, 1.0, 0.0])
            z_axis = goal_dict.get('gripper_z_axis', [0.0, 0.0, 1.0])

            # Convert orientation axes to quaternion
            rotation_matrix = np.array([x_axis, y_axis, z_axis]).T # Columns as x, y, z axes

            if self.print_goal:
                self.get_logger().info(f'Goal dict:\n{pp.pformat(goal_dict)}')

            # Extract wrist position
            # wrist_position = goal_dict['wrist_position']
            gripper_width = goal_dict.get('grip_width', None)
            lower_limit, upper_limit = self.joint_limits['wrist_2_gripper_joint']
            gripper_position = map_within_limits(gripper_width, 0.0, 1.0, lower_limit, upper_limit, 0)
                

            # Extract orientation axes
            x_axis = goal_dict.get('gripper_x_axis', [1.0, 0.0, 0.0])
            y_axis = goal_dict.get('gripper_y_axis', [0.0, 1.0, 0.0])
            z_axis = goal_dict.get('gripper_z_axis', [0.0, 0.0, 1.0])

            # Decompose into roll, pitch, and yaw
            original_rpy = Rotation.from_matrix(rotation_matrix).as_euler('xyz')  # [roll, pitch, yaw]

            # Compute new yaw from wrist position
            x, y, _ = wrist_position  # Extract x and y from the position
            new_yaw = -np.arctan2(abs(y),-x )  # Calculate yaw in radians
            new_marker_yaw = np.arctan2(abs(y),x )  # Calculate yaw in radians

            # Combine original roll, pitch with new yaw
            new_rpy = [original_rpy[0], original_rpy[1], new_yaw]  # [roll, pitch, new_yaw]
            new_marker_rpy = [original_rpy[0], original_rpy[1], new_marker_yaw]
            # Create a new rotation matrix with updated yaw
            new_rotation_matrix = Rotation.from_euler('xyz', new_rpy).as_matrix()
            new_marker_rotation_matrix = Rotation.from_euler('xyz', new_marker_rpy).as_matrix()
            rotation_correction = np.array([
                [0, -1, 0],
                [1,  0, 0],
                [0,  0, 1]
            ])

            # Apply the correction
            corrected_rotation_matrix = new_rotation_matrix @ rotation_correction # Apply the correction
            correct_rotation_rpy = Rotation.from_matrix(corrected_rotation_matrix).as_euler('xyz')
            quaternion = Rotation.from_matrix(new_marker_rotation_matrix).as_quat()  # [x, y, z, w]
            wrist_position[0] = wrist_position[0] * 3
            wrist_position[1] = wrist_position[1] * 3
            wrist_position[2] = wrist_position[2] * 1

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"  # Update as per your RViz frame

            pose_msg.pose.position.x = wrist_position[0]
            pose_msg.pose.position.y = wrist_position[1]
            pose_msg.pose.position.z = wrist_position[2]
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            self.goal_pose_publisher.publish(pose_msg)
            # Solve IK to get joint positions
            physical_wrist_position = []
            physical_wrist_position.append(wrist_position[1])
            physical_wrist_position.append(-wrist_position[0])
            physical_wrist_position.append(wrist_position[2])
            physical_wrist_orientation = []
            physical_wrist_orientation.append(correct_rotation_rpy[1])
            physical_wrist_orientation.append(correct_rotation_rpy[0])
            physical_wrist_orientation.append(correct_rotation_rpy[2])

            # joint_positions = self.giraffe_chain.inverse_kinematics(physical_wrist_position, physical_wrist_orientation, orientation_mode="all")
            joint_positions = self.giraffe_chain.inverse_kinematics(physical_wrist_position)
            current_configuration = {
                'base_link_shoulder_pan_joint':     joint_positions[0],
                'shoulder_pan_shoulder_lift_joint': joint_positions[1],
                'shoulder_lift_elbow_joint':        joint_positions[2],
                'elbow_wrist_1_joint':              joint_positions[3],
                'wrist_1_wrist_2_joint':            joint_positions[4]
            }

            achieved_position = self.simple_ik.fk(current_configuration, use_urdf=True)
            self.get_logger().info(f'wrist_position: {physical_wrist_position}')
            self.get_logger().info(f'achieved_position: {achieved_position}')
            if len(joint_positions) > 0:
                # Map joint_positions to the correct order
                ordered_positions = [float(x) for x in joint_positions[1:-1]]
                self.get_logger().info(f'ordered_joint_positions: {ordered_positions}')

                # Check if the positions have changed significantly
                if self.previous_positions and self.positions_similar(ordered_positions, self.previous_positions):
                    self.get_logger().debug('No significant change in positions; skipping trajectory publish.')
                    return

                # Create a JointTrajectory message
                trajectory_msg = JointTrajectory()
                trajectory_msg.header = Header()
                trajectory_msg.header.stamp.sec = 0  # Match rqt_joint_trajectory_controller
                trajectory_msg.header.stamp.nanosec = 0
                trajectory_msg.joint_names = self.joint_names

                # Create a single trajectory point
                point = JointTrajectoryPoint()
                point.positions = ordered_positions
                point.time_from_start.sec = 1  # Match rqt_joint_trajectory_controller
                point.time_from_start.nanosec = 0
                trajectory_msg.points.append(point)

                gripper_msg = JointTrajectory()
                gripper_msg.header = Header()
                gripper_msg.header.stamp.sec = 0  # Match rqt_joint_trajectory_controller
                gripper_msg.header.stamp.nanosec = 0
                gripper_msg.joint_names = ['wrist_2_gripper_joint']

                gripper_point = JointTrajectoryPoint()
                gripper_point.positions = [gripper_position]  # Close the gripper
                gripper_point.time_from_start.sec = 1  # Match rqt_joint_trajectory_controller
                gripper_point.time_from_start.nanosec = 0
                gripper_msg.points.append(gripper_point)
                # Publish the trajectory
                self.trajectory_publisher.publish(trajectory_msg)
                self.gripper_publisher.publish(gripper_msg)

                # Update the previous positions
                self.previous_positions = ordered_positions

        self.loop_timer.end_of_iteration()

        if self.print_timing:
            self.loop_timer.pretty_print()

    def positions_similar(self, positions1, positions2, threshold=0.01):
        """Check if two position lists are similar within a threshold."""
        return all(abs(p1 - p2) < threshold for p1, p2 in zip(positions1, positions2))


def main(args=None):
    rclpy.init(args=args)

    # Start the DexTeleopNode
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
