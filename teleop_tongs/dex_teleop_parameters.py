import numpy as np
import argparse


def get_arg_parser():
    parser = argparse.ArgumentParser(
        prog='Stretch Dexterous Teleop',
        description='This application enables a human operator to control Stretch using ArUco markers and a Logitech C930e webcam on the floor looking up with a light ring. Currently, the code supports tongs with ArUco markers attached.',
    )
    parser.add_argument('-l', '--left', action='store_true', help = 'Use Stretch as a left-handed robot. This requires left-hand tongs, which have distinct ArUco markers from right-hand tongs.')
    parser.add_argument('-f', '--fast', action='store_true', help = 'Move Stretch at the fastest available speed. The default is to move the robot at the slowest available speed')
    parser.add_argument('-g', '--ground', action='store_true', help = 'Manipulate at ground level up to around tabletop height. The default is to manipulate from tabletop height up to countertop height.')
    parser.add_argument('-s', '--stretch_2', action='store_true', help = 'Use a Stretch 2, which may require special settings.')
    parser.add_argument('-m', '--multiprocessing', action='store_true', help = 'Write goals to shared memory using Python multiprocessing.')
    parser.add_argument('-i', '--slide_lift_range', action='store_true', help = 'Holding the tongs high will gradually slide the lift range of motion upward. Holding them low will gradually slide the lift range of motion downward. The default is to use a fixed range of motion for the lift.')
    
    return parser


# Measurements for the tongs
tongs_to_use = '56mm' #'50mm' #'44mm'

if tongs_to_use == '56mm':
    # 50mm ArUco marker tongs
    tongs_cube_side = 0.075
    tongs_pin_joint_to_marker_center = 0.155
    tongs_pin_joint_to_tong_tip = 0.1296
    #tongs_marker_center_to_tong_tip = (tongs_cube_side / 2.0) + 0.0125
    tongs_marker_center_to_tong_tip = (tongs_cube_side / 2.0) + 0.0115
    tongs_open_grip_width = 0.0653
    tongs_closed_grip_width = 0.005
    

# The maximum and minimum goal_wrist_position z values do not
# need to be perfect due to joint limit checking performed by
# the SimpleIK based on the specialized URDF joint
# limits. They are specified with respect to the robot's
# coordinate system.
goal_max_position_z = 0.5
goal_min_position_z = 0.0

# Lower limit for wrist pitch. Allowing angles close to -Pi/2 will
# allow the gripper to point almost straight down, which is near a
# singularity. When the wrist is straight down with pitch = -Pi/2, the
# wrist roll and wrist yaw joints have rotational axes that are
# parallel resulting in gimbal lock. This can result in undesirable
# motions and unintuitive configurations. Changing this variable can
# be used to increase the risk of issues in order to point the gripper
# closer to straight down. A value of None will result in the pitch
# being allowed to move over the entire range of motion allowed by the
# robot's exported URDF. The default lowest pitch was -1.57 rad on the
# default URDF used during development (-89.954373836 deg)

#AFTER CHANGING THIS VARIABLE YOU NEED TO RUN
# prepare_specialized_urdfs.py FOR IT TO TAKE EFFECT.

wrist_pitch_lower_limit =  -0.8 * (np.pi/2.0)



# DROP GRIPPER ORIENTATION GOALS WITH LARGE JOINT ANGLE CHANGES
#
# Dropping goals that result in extreme changes in joint
# angles over a single time step avoids the nearly 360
# degree rotation in an opposite direction of motion that
# can occur when a goal jumps across a joint limit for a
# joint with a large range of motion like the roll joint.
#
# This also reduces the potential for unexpected wrist
# motions near gimbal lock when the yaw and roll axes are
# aligned (i.e., the gripper is pointed down to the
# ground). Goals representing slow motions that traverse
# near this gimbal lock region can still result in the
# gripper approximately going upside down in a manner
# similar to a pendulum, but this results in large yaw
# joint motions and is prevented at high speeds due to
# joint angles that differ significantly between time
# steps. Inverting this motion must also be performed at
# low speeds or the gripper will become stuck and need to
# traverse a trajectory around the gimbal lock region.
#
#max_allowed_wrist_yaw_change = np.pi/2.0
#max_allowed_wrist_roll_change = np.pi/2.0
max_allowed_wrist_yaw_change = 1.8 * (np.pi/2.0)
max_allowed_wrist_roll_change = 1.8 * (np.pi/2.0)

        
# This is the weight between 0.0 and 1.0 multiplied by the current
# command when performing exponential smoothing. A higher weight leads
# to less lag, but higher noise. A lower weight leads to more lag, but
# smoother motions with less noise.

# current_filtered_value = ((1.0 - exponential_smoothing) * previous_filtered_value) + (exponential_smoothing * current_value)
exponential_smoothing_for_orientation = 0.2 # 0.25 #0.1
exponential_smoothing_for_position = 0.2 #0.5 #0.1


# When False, the robot should only move to its initial position
# and not move in response to ArUco markers. This is helpful when
# first trying new code and interface objects.
robot_allowed_to_move = True

# Camera stand at maximal height
# Minimum distance from the tongs to the camera in meters
#min_dist_from_camera_to_tongs = 0.3
# Maximum distance from the tongs to the camera in meters
#max_dist_from_camera_to_tongs = 0.8

# Camera stand at minimal height

# This range represents the manipulation range. Commands for sliding
# the lift range are outside of this range.

# Minimum distance from the tongs to the camera in meters
min_dist_from_camera_to_tongs = 0.6 #0.5
# Maximum distance from the tongs to the camera in meters
max_dist_from_camera_to_tongs = 1.2 #1.0

# Maximum height range of tongs
max_tongs_height_range = max_dist_from_camera_to_tongs - min_dist_from_camera_to_tongs


# The origin for teleoperation with respect to the camera's frame
# of reference. Holding the teleoperation interface at this
# position with respect to the camera results in the robot being
# commanded to achieve the center_wrist_position. The camera's
# frame of reference has its origin in the optical center of the
# camera and its z-axis points directly out of the camera on the
# camera's optical axis.
teleop_origin_x = 0.0
teleop_origin_y = -0.40
teleop_origin_z = min_dist_from_camera_to_tongs + max_tongs_height_range/3.0

teleop_origin = np.array([teleop_origin_x, teleop_origin_y, teleop_origin_z])


# Robot configuration used to define the center wrist position

def get_center_configuration(): 
    # manipulate lower objects
    center_configuration = {
        'base_link_shoulder_pan_joint': 0.0,
        'shoulder_pan_shoulder_lift_joint': 0.3927,
        'shoulder_lift_elbow_joint': 0.1222,
        'elbow_wrist_1_joint': 0.0,
        'wrist_1_wrist_2_joint': 0.7854
    }
    return center_configuration

def get_starting_configuration(): 

    # The robot will attempt to achieve this configuration before
    # teleoperation begins. Teleoperation commands for the robot's
    # wrist position are made relative to the wrist position
    # associated with this starting configuration.
    starting_configuration = {
        'base_link_shoulder_pan_joint': 0.0,
        'shoulder_pan_shoulder_lift_joint': 0.3927,
        'shoulder_lift_elbow_joint': 0.1222,
        'elbow_wrist_1_joint': 0.0,
        'wrist_1_wrist_2_joint': 0.7854
    }
    return starting_configuration


# String used by processes to communicate via shared memory
shared_memory_name = 'gripper_goal_20231222'


def goal_dict_to_array(goal_dict):
    if goal_dict is None:
        return None

    goal_array = np.zeros((5, 3), dtype=np.float64)
    grip_width = goal_dict['grip_width']
    if grip_width is not None: 
        goal_array[0,0] = grip_width
    else:
        goal_array[0,0] = -10000.0
        
    goal_array[1,:] = goal_dict['wrist_position']
    goal_array[2,:] = goal_dict['gripper_x_axis']
    goal_array[3,:] = goal_dict['gripper_y_axis']
    goal_array[4,:] = goal_dict['gripper_z_axis']
    
    return goal_array


def goal_array_to_dict(goal_array):
    goal_dict = {}
    grip_width = goal_array[0,0]
    if grip_width < -1000.0:
        grip_width = None
    goal_dict['grip_width'] = grip_width
    goal_dict['wrist_position'] = goal_array[1,:]
    goal_dict['gripper_x_axis'] = goal_array[2,:]
    goal_dict['gripper_y_axis'] = goal_array[3,:]
    goal_dict['gripper_z_axis'] = goal_array[4,:]
    return goal_dict


def get_example_goal_array():
    
    goal_grip_width = 1.0
    goal_wrist_position = np.array([-0.03, -0.4, 0.9])
    goal_x_axis = np.array([ 1.0,  0.0, 0.0])
    goal_y_axis = np.array([ 0.0, -1.0, 0.0])
    goal_z_axis = np.array([ 0.0, 0.0, -1.0])

    example_goal = {'grip_width': goal_grip_width,
                    'wrist_position': goal_wrist_position,
                    'gripper_x_axis': goal_x_axis,
                    'gripper_y_axis': goal_y_axis,
                    'gripper_z_axis': goal_z_axis}

    goal_array = goal_dict_to_array(example_goal)
    return goal_array


def get_do_nothing_goal_array():
    
    goal_grip_width = -10000.0
    goal_wrist_position = np.array([-10000.0, -10000.0, -10000.0])
    goal_x_axis = np.array([ 1.0,  0.0, 0.0])
    goal_y_axis = np.array([ 0.0, -1.0, 0.0])
    goal_z_axis = np.array([ 0.0, 0.0, -1.0])

    example_goal = {'grip_width': goal_grip_width,
                    'wrist_position': goal_wrist_position,
                    'gripper_x_axis': goal_x_axis,
                    'gripper_y_axis': goal_y_axis,
                    'gripper_z_axis': goal_z_axis}

    goal_array = goal_dict_to_array(example_goal)
    return goal_array


def is_a_do_nothing_goal_array(goal_array):
    test = goal_array[1:,:] < -1000.0
    return np.any(test)
