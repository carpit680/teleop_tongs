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
            self.webcam_aruco_detector = wt.WebcamArucoDetector(tongs_prefix='left', visualize_detections=True)
        else:
            self.webcam_aruco_detector = wt.WebcamArucoDetector(tongs_prefix='right', visualize_detections=True)

        # Center wrist position (used for teleoperation origin)
        self.center_wrist_position = self.simple_ik.fk(self.center_configuration)
        self.goal_from_markers = gt.GoalFromMarkers(dt.teleop_origin, self.center_wrist_position)

        # ROS 2 Publishers
        self.trajectory_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

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

        if goal_dict:
            if self.print_goal:
                self.get_logger().info(f'Goal dict:\n{pp.pformat(goal_dict)}')

            # Extract wrist position
            wrist_position = goal_dict['wrist_position']

            # Solve IK to get joint positions
            joint_positions = self.simple_ik.ik(wrist_position)

            if joint_positions:
                # Map joint_positions to the correct order
                ordered_positions = [joint_positions[joint] for joint in self.joint_names]

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

                # Publish the trajectory
                self.get_logger().info(f'Publishing trajectory: {trajectory_msg}')
                self.trajectory_publisher.publish(trajectory_msg)

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
