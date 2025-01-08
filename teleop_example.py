from teleop_tongs.teleop_tongs import DexTeleop

dt = DexTeleop(urdf_path='giraffe.urdf', cam_calib_path='camera_calibration_results.yaml', degree=True, visualize_detections=True)

while True:
    goal_pose = dt.get_goal_pose()
    if goal_pose is not None:
        print('goal_pose =', goal_pose)