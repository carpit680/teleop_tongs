
import os
import math
import time
import numpy as np
import pprint as pp
import threading
from threading import Lock
from ikpy.chain import Chain
import urchin as urdf_loader
from scipy.spatial.transform import Rotation
import cv2

import simple_ik as si
import loop_timer as lt
import goal_from_teleop as gt
import dex_teleop_parameters as dt
import webcam_teleop_interface as wt
from feetech import FeetechMotorsBus


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


class MultiCameraManager:
    def __init__(self, camera_indices=None):
        """
        Initialize the MultiCameraManager with an optional list of camera indices.
        
        :param camera_indices: A list of integers (camera indices). 
                               Defaults to [0] if not provided.
        """
        if camera_indices is None:
            camera_indices = [0]
        
        self.camera_indices = camera_indices
        self.captures = {}  # Dictionary to store {camera_index: VideoCapture}
        self.is_running = False

    def open_cameras(
        self, 
        resolutions=None,   # Dictionary: {camera_index: (width, height)}
        frame_rates=None    # Dictionary: {camera_index: fps}
    ):
        """
        Open all cameras specified in self.camera_indices and optionally set
        resolution (width, height) and frame rate (fps) for each camera.
        
        :param resolutions: A dict mapping camera index -> (width, height). 
                            Example: {0: (1280, 720), 1: (640, 480)}
        :param frame_rates: A dict mapping camera index -> fps (int or float).
                            Example: {0: 30, 1: 15}
        """
        for idx in self.camera_indices:
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                print(f"Error: Could not open camera with index {idx}")
            else:
                # Set resolution if specified
                if resolutions and idx in resolutions:
                    width, height = resolutions[idx]
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                
                # Set frame rate if specified
                if frame_rates and idx in frame_rates:
                    cap.set(cv2.CAP_PROP_FPS, frame_rates[idx])
                
                # Store the opened capture object
                self.captures[idx] = cap

        if not self.captures:
            print("No valid cameras could be opened.")

    def close_cameras(self):
        """Release all camera resources and destroy any OpenCV windows."""
        for idx, cap in self.captures.items():
            cap.release()
        self.captures.clear()
        cv2.destroyAllWindows()

    def read_frames(self):
        """
        Read a frame from each camera.
        Returns a dictionary of {camera_index: frame} for all opened cameras.
        """
        frames = {}
        for idx, cap in self.captures.items():
            ret, frame = cap.read()
            if ret:
                frames[idx] = frame
            else:
                print(f"Failed to grab frame from camera index {idx}")
        return frames

    def show_frames(self, frames):
        """
        Display each frame in its own window.
        
        :param frames: A dictionary of {camera_index: frame}.
        """
        for idx, frame in frames.items():
            cv2.imshow(f"Camera {idx}", frame)


class DexTeleop:
    def __init__(self):
        # Initialize configurations and objects
        self.max_goal_wrist_position_z = dt.goal_max_position_z
        self.min_goal_wrist_position_z = dt.goal_min_position_z

        self.center_configuration = dt.get_center_configuration()
        self.starting_configuration = dt.get_starting_configuration()
        self.simple_ik = si.SimpleIK()

        self.webcam_aruco_detector = wt.WebcamArucoDetector(
            tongs_prefix='right',
            visualize_detections=True
        )

        # Center wrist position (used for teleoperation origin)
        self.center_wrist_position = self.simple_ik.fk(self.center_configuration)
        self.goal_from_markers = gt.GoalFromMarkers(dt.teleop_origin, self.center_wrist_position)

        # IKPy chain setup
        active_links_mask = [False, True, True, True, True, True, False]
        urdf_file_name = 'giraffe.urdf'
        self.urdf = load_urdf(urdf_file_name)
        self.giraffe_chain = Chain.from_urdf_file(
            active_links_mask=active_links_mask,
            urdf_file=urdf_file_name
        )
        self.joint_limits = get_joint_limits(self.urdf)
        print('SimpleIK: Joint limits:', self.joint_limits)

        # Timer periods
        #   - marker_timer_period: how often we process camera frames
        #   - teleop_timer_period: how often we compute IK & send commands
        self.marker_timer_period = 0.05  # 10 Hz
        self.teleop_timer_period = 0.05  # 10 Hz
        self.camera_timer_period = 0.05  # 10 Hz

        # Loop timer (for optional printouts)
        self.loop_timer = lt.LoopTimer()

        # Joint names
        self.joint_names = [
            'base_link_shoulder_pan_joint',
            'shoulder_pan_shoulder_lift_joint',
            'shoulder_lift_elbow_joint',
            'elbow_wrist_1_joint',
            'wrist_1_wrist_2_joint'
        ]

        # Feetech motors
        self.motors_bus = FeetechMotorsBus(
            port="/dev/ttyACM0",
            motors={
                "base_link_shoulder_pan_joint": (1, "sts3215"),
                "shoulder_pan_shoulder_lift_joint": (2, "sts3215"),
                "shoulder_lift_elbow_joint": (3, "sts3215"),
                "elbow_wrist_1_joint": (4, "sts3215"),
                "wrist_1_wrist_2_joint": (5, "sts3215"),
                "wrist_2_gripper_joint": (6, "sts3215"),
            },
        )
        self.motor_order = [
            "base_link_shoulder_pan_joint",
            "shoulder_pan_shoulder_lift_joint",
            "shoulder_lift_elbow_joint",
            "elbow_wrist_1_joint",
            "wrist_1_wrist_2_joint",
            "wrist_2_gripper_joint",
        ]
        self.motors_bus.connect()

        # Setup camera(s)
        self.cams = MultiCameraManager(camera_indices=[0])
        resolutions = {
            0: (480, 640)
        }
        
        frame_rates = {
            0: 30,
        }
        
        # Open cameras with the specified resolution and FPS settings
        self.cams.open_cameras(resolutions=resolutions, frame_rates=frame_rates)

        # State variables
        self.previous_positions = []  # Track last sent positions to avoid duplicate commands
        self.markers = None
        self.marker_lock = Lock()

        # Smoothing filter
        self.smoothed_positions = None
        self.smoothing_alpha = 0.5  # Alpha for exponential moving average

        # Homing offsets
        self.offsets = [3.223, 3.043, 2.979, 3.152, 3.1415, 4.9532]

        # Motor parameters
        self.set_motor_acceleration(5, 50)

        # Print toggles
        self.print_timing = False
        self.print_goal = False
        self.publish_goal_pose_marker = True

        # Threading stop signal
        self.stop_event = threading.Event()

    def set_motor_acceleration(self, acceleration: int, gripper_acceleration: int):
        """Set acceleration for all motors."""
        try:
            motor_names = self.motors_bus.motor_names
            non_gripper_motors = motor_names[:-1]
            accelerations = [acceleration] * len(non_gripper_motors)
            self.motors_bus.write("Acceleration", accelerations, non_gripper_motors)
            gripper = motor_names[-1]
            self.motors_bus.write("Acceleration", gripper_acceleration, gripper)
            print(f"Set acceleration to {acceleration} for {motor_names[:-1]}.")
            print(f"Set acceleration to {gripper_acceleration} for {gripper}.")
        except Exception as e:
            print(f"Failed to set acceleration for motors: {e}")

    def radians_to_steps(self, radians: float) -> int:
        """
        Converts radians to motor steps based on the model resolution.
        Assumes a full rotation (-pi to +pi radians) maps to the full step range.
        """
        resolution = 4096
        degrees = np.degrees(radians)  # Convert radians to degrees
        steps = int(degrees / 360.0 * resolution)  # Map degrees to steps
        return steps

    def process_markers(self):
        """
        Pull the next webcam frame and update self.markers.
        """
        with self.marker_lock:
            self.markers = self.webcam_aruco_detector.process_next_frame()

    def apply_smoothing(self, new_positions):
        if self.smoothed_positions is None:
            self.smoothed_positions = np.array(new_positions)
        else:
            self.smoothed_positions = (
                self.smoothing_alpha * np.array(new_positions)
                + (1 - self.smoothing_alpha) * self.smoothed_positions
            )
        return self.smoothed_positions.tolist()

    def teleop_loop(self):
        """
        Compute IK based on detected markers and send commands to motors.
        """
        self.loop_timer.start_of_iteration()

        # Acquire the current markers result
        with self.marker_lock:
            goal_dict = self.goal_from_markers.get_goal_dict(self.markers)

        if goal_dict is not None:
            wrist_position = goal_dict.get('wrist_position', [0.0, 0.0, 0.0])
            x, y, z = wrist_position

            # Enforce Z-bound for teleop
            if not (self.min_goal_wrist_position_z <= z <= self.max_goal_wrist_position_z):
                self.loop_timer.end_of_iteration()
                if self.print_timing:
                    self.loop_timer.pretty_print()
                return

            # Extract orientation axes
            x_axis = goal_dict.get('gripper_x_axis', [1.0, 0.0, 0.0])
            y_axis = goal_dict.get('gripper_y_axis', [0.0, 1.0, 0.0])
            z_axis = goal_dict.get('gripper_z_axis', [0.0, 0.0, 1.0])
            rotation_matrix = np.array([x_axis, y_axis, z_axis]).T

            if self.print_goal:
                print(f'Goal dict:\n{pp.pformat(goal_dict)}')

            # Gripper width -> joint limit mapping
            gripper_width = goal_dict.get('grip_width', None)
            lower_limit, upper_limit = self.joint_limits['wrist_2_gripper_joint']
            gripper_position = map_within_limits(gripper_width, 0.0, 1.0, lower_limit, upper_limit, 0)

            # Compute orientation (roll/pitch/yaw)
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
            if len(joint_positions) > 0:
                # Adjust final angles
                new_marker_rpy[1] += -3.1415 if new_marker_rpy[1] > 0 else 3.1415
                ordered_positions = [float(x) for x in joint_positions[1:-1]]
                ordered_positions[-1] = new_marker_rpy[0] + 1.5708
                ordered_positions[-2] -= new_marker_rpy[1]

                # Smooth the joint positions
                ordered_positions = self.apply_smoothing(ordered_positions)
                ordered_positions.append(gripper_position)

                # Convert to motor steps
                positions = []
                homing_offsets = [-2082, -1992, -1949, -2023, -2046, -3225]
                for pos, offset in zip(ordered_positions, homing_offsets):
                    radians = pos
                    step_value = self.radians_to_steps(-radians) - offset
                    positions.append(step_value)

                # Send to motors
                self.motors_bus.write("Goal_Position", np.array(positions), self.motor_order)

                # Optionally read back the positions
                present_positions = self.motors_bus.read("Present_Position", self.motor_order)
                position_radians = self.motors_bus.steps_to_radians(
                    present_positions,
                    self.motors_bus.motors[self.motor_order[0]][1]
                )
                feedback = []
                for p_radians, offset in zip(position_radians, self.offsets):
                    feedback.append(-p_radians + offset)
                # Do something with `feedback` if desired

        self.loop_timer.end_of_iteration()
        if self.print_timing:
            self.loop_timer.pretty_print()

    def run_marker_loop(self):
        """
        This method runs in a separate thread to continuously process markers.
        """
        print("Marker processing thread started.")
        while not self.stop_event.is_set():
            self.process_markers()
            time.sleep(self.marker_timer_period)
        print("Marker processing thread stopped.")

    def run_teleop_loop(self):
        """
        This method runs in a separate thread to continuously update teleop logic.
        """
        print("Teleop loop thread started.")
        while not self.stop_event.is_set():
            self.teleop_loop()
            time.sleep(self.teleop_timer_period)
        print("Teleop loop thread stopped.")

    def run_camera_loop(self):
        """
        This method runs in a separate thread to continuously read camera data.
        """
        print("Camera loop thread started.")
        while not self.stop_event.is_set():
            frames = self.cams.read_frames()
            # self.cams.show_frames(frames)
            
            # # Check for ESC key press
            # if cv2.waitKey(1) & 0xFF == 27:  # ESC key
            #     print("ESC key pressed. Exiting...")
            time.sleep(self.camera_timer_period)
        self.cams.close_cameras()
        print("Camera loop thread stopped.")

    def run(self):
        """
        Main method to start and manage the two threads for marker processing
        and teleop logic. If a KeyboardInterrupt is received, signal both
        threads to stop, then wait for them to join.
        """
        # Create threads
        marker_thread = threading.Thread(target=self.run_marker_loop, daemon=True)
        teleop_thread = threading.Thread(target=self.run_teleop_loop, daemon=True)
        camera_thread = threading.Thread(target=self.run_camera_loop, daemon=True)

        # Start threads
        marker_thread.start()
        teleop_thread.start()
        camera_thread.start()

        print("Threads started. Press Ctrl-C to stop.")
        try:
            # Keep the main thread alive until user interrupts
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt received. Shutting down...")
            self.stop_event.set()
            # Wait for threads to finish
            marker_thread.join()
            teleop_thread.join()
            camera_thread.join()
            print("All threads stopped. Exiting.")


def main():
    # Instantiate and run in a pure Python context
    teleop = DexTeleop()
    teleop.run()


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
