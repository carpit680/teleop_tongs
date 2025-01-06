import math
import numpy as np
import os
import urchin as urdf_loader

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

def nan_in_configuration(configuration):
    for k, v in configuration.items():
        if math.isnan(v) or np.isnan(v):
            return True
    return False

class SimpleIK:
    def __init__(self, urdf_path):
        # URDF file and joint names
        self.urdf_file_name = urdf_path
        self.urdf = load_urdf(self.urdf_file_name)

        self.joint_names = [
            'base_link_shoulder_pan_joint',
            'shoulder_pan_shoulder_lift_joint',
            'shoulder_lift_elbow_joint',
            'elbow_wrist_1_joint',
            'wrist_1_wrist_2_joint'
        ]

        self.end_effector_name = 'gripper'  # Adjust based on URDF end-effector link
        self.joint_limits = get_joint_limits(self.urdf)

        print('SimpleIK: Joint limits:', self.joint_limits)

        self.calibrate_vectors_and_offsets()

    def calibrate_vectors_and_offsets(self):
        zero_cfg = {joint: 0.0 for joint in self.joint_names}
        zero_fk = self.urdf.link_fk(cfg=zero_cfg, links=[self.end_effector_name])
        zero_wrist_position = zero_fk[self.urdf.link_map[self.end_effector_name]].dot(
            np.array([0.0, 0.0, 0.0, 1.0]))[:3]

        self.unit_vectors = {}
        for joint in self.joint_names:
            cfg = zero_cfg.copy()
            cfg[joint] = 1.0  # Move this joint by 1 radian or unit
            fk = self.urdf.link_fk(cfg=cfg, links=[self.end_effector_name])
            wrist_position = fk[self.urdf.link_map[self.end_effector_name]].dot(
                np.array([0.0, 0.0, 0.0, 1.0]))[:3]
            unit_vector = wrist_position - zero_wrist_position
            unit_vector = unit_vector / np.linalg.norm(unit_vector)
            self.unit_vectors[joint] = unit_vector

        self.offsets = {joint: 0.0 for joint in self.joint_names}

    def within_joint_limits(self, robot_configuration):
        for joint_name, joint_value in robot_configuration.items():
            lower_limit, upper_limit = self.joint_limits[joint_name]
            if joint_value < lower_limit or joint_value > upper_limit:
                return False
        return True

    def clip_with_joint_limits(self, robot_configuration):
        for joint_name, joint_value in robot_configuration.items():
            lower_limit, upper_limit = self.joint_limits[joint_name]
            robot_configuration[joint_name] = np.clip(joint_value, lower_limit, upper_limit)

    def fk(self, robot_configuration, use_urdf=False):
        cfg = robot_configuration

        if use_urdf:
            urdf_fk = self.urdf.link_fk(cfg=cfg, links=[self.end_effector_name])
            wrist_position = urdf_fk[self.urdf.link_map[self.end_effector_name]].dot(
                np.array([0.0, 0.0, 0.0, 1.0]))[:3]
        else:
            wrist_position = np.zeros(3)
            for joint, value in cfg.items():
                wrist_position += value * self.unit_vectors[joint]
        return wrist_position

    def ik(self, wrist_position):
        goal = np.array(wrist_position)
        cfg = {}

        current_position = np.zeros(3)
        for joint in self.joint_names:
            unit_vector = self.unit_vectors[joint]
            dot_product = np.dot(goal - current_position, unit_vector)
            cfg[joint] = dot_product  # Basic projection onto the unit vector
            current_position += dot_product * unit_vector

        if nan_in_configuration(cfg) or (not self.within_joint_limits(cfg)):
            return None

        return cfg


if __name__ == '__main__':
    simple_ik = SimpleIK()

    wrist_positions = [
        [0.1, -0.5, 0.3],
        [0.2, 0.5, 0.3],
        [0.0, 0.0, 0.5],
        [-0.3, 0.2, 0.3],
        [0.2, -0.3, 0.4]
    ]

    for wrist_position_goal in wrist_positions:
        print(f'\nWrist position goal: {wrist_position_goal}')

        cfg = simple_ik.ik(wrist_position_goal)
        print('IK configuration:', cfg)

        if cfg is not None:
            wrist_position_fk = simple_ik.fk(cfg, use_urdf=False)
            print('FK wrist position:', wrist_position_fk)

            simple_ik.clip_with_joint_limits(cfg)
            print('Clipped IK configuration:', cfg)
