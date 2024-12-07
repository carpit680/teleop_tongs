# __Teleop Tongs for robotic manipulator__

| Robot Teleoperation                            |
| ---------------------------------------------- |
| ![](./gifs/single_arm_dishes_short_318x360.gif) |

This repository provides code for dexterous teleoperation of [Giraffe](https://github.com/carpit680/giraffe), a low-cost 5DoF robotic manipulator.

Teleop Tongs support performing efficient multijoint movement with Giraffe or other similar robots.

The human operator uses modified kitchen tongs or 3D printed tongs with attached ArUco markers to control the pose of the end effector. A webcam looking up from a stand placed on the ground observes the tongs to estimate the tongs' position, orientation, and grip width. A ring light around the webcam ensures that the ArUco markers can be detected during fast motions by reducing motion blur.

The system could be adapted to use other interfaces that can provide a five to seven degrees of freedom (5-7 DOF) target pose and grip width/angle at a high rate (e.g., >= 15 Hz). The position of this target pose controls the end of the robotic manipulator via inverse kinematics (IK). The grip width commands the robot's gripper.

## Motivation

| Single Robot Teleoperation                     |
| ---------------------------------------------- |
|          ![](./gifs/play_with_dog.gif)          |

Hello Robot provided code to teleoperate the first version of Stretch (the Stretch RE1) using a gamepad and a web interface. Both types of teleoperation have improved over the years with work from the Stretch community and Hello Robot. Gamepad teleoperation is simple and portable. Web-based teleoperation can be used over great distances and has been made accessible for people with disabilities. A notable disadvantage of these approaches is that they tend to favor slow single joint motions with a single arm.

HuggingFace has done some amazing work with the help of the community in developing lerobot with its plug-and-play Imitation Learning pipeline. At the time of development one can setup a Koch V1.1, Standard Open ARM 100 or Aloha bimanual setup with leader and follower arms to teleoperate and collect episodes to train the model with.

I initially wanted to repplicate their setup and build on top of that but later figured, why not make this pipeline even more accessible and so I developed my own version of low-cost robotic manipulator and this teleop_tongs to create a data collection setup at half the cost of other available setups.

## Setting Up Teleop dex

You should start by cloning this repository. All of the commands below should be run from the command line in the root directory of the repository on your robot's computer. 

### Build Your Interface

You can 3D print the components required for the Teleop Tongs setup along with the feducial markers by following the __[Teleop Tongs Assembly Guide](https://docs.google.com/document/d/1RCJbuiAU41ctlkckxPVuluC3laB2XDHw1zR1Jke-rWY/edit?usp=sharing)__.

You will need a camera, a ring light, and a stand as shown in the following photo.

<div style="text-align: center;">
    <img src="./images/camera_ring_light_and_stand.jpg" width="30%" alt="camera, ring light, and stand setup">
</div>

For a single robot, you will need right-hand tongs like those shown in the following two photos.

<div style="text-align: center;">
    <img src="./images/right_tongs_held_and_open.jpg" width="40%"> <img src="./images/right_tongs_held_and_closed.jpg" width="40%">
</div>

### Run the Installation Script

Clone this github repository.

```bash
git clone -b ros2 https://github.com/carpit680/teleop_tongs.git
```

Then run the following installation script found in the repository's root directory.

```bash
./install_dex_teleop.sh
```

The installation script sets up a udev rule for a Mi USB Webcam HD, so that the camera can be reset each time you run dexterous teleoperation. This is a workaround to avoid low frame rates and errors in the camera settings.

Next, the installation script installs v4l2 utilities, if necessary.

You also need to install some python dependencies:

```bash
pip install -r requirements.txt
```

### Generate Specialized URDFs

To run Teleop Tongs, you need to generate specialized URDF files. Teleop Tongs uses forward kinematic (FK) and inverse kinematic (IK) models of the robot. These models use specialized URDFs generated from the calibrated URDF on your robot.

```bash
python3 prepare_specialized_urdfs.py
```

### Set Up the Camera, Ring Light and Stand

As shown in the photo above, the camera stand should be placed on the ground, and the camera should be pointed straight up. The stand should be at its minimum height.

The camera should be plugged into your computer using a USB extension cable. The ring light should not be plugged carelessly as it requires too much power - it can either be plugged into a powered USB port, or externally.

When using the camera, the top of the camera should be pointed away from you. With respect to the robot, the top of the camera points in the direction of arm extended forward from its base, and the lens of the camera looks up.

### Calibrate the Webcam

After setting up your camera, you need to calibrate it.

First, generate a calibration board using the following command:

```bash
python3 webcam_calibration_create_board.py
```

This should result in the following PNG file.

```bash
webcam_aruco_calibration_board.png
```

Print this image out without scaling it. The resulting printout should match the dimensions specified in the PNG file.

Mount the resulting printout on a flat surface that you can move around the camera to capture calibration images __with the ring light turned on__.

Use the following command and your calibration pattern to collect calibration images for your Logitech C930e webcam. The entire calibration board should be visible and not too far away, or else the calibration images can lead to errors.

```bash
python3 webcam_calibration_collect_images.py
```

The images will be stored in the following directory.

```bash
./webcam_calibration_images/<camera name>/<camera resolution>
```

Once you've collected the calibration images, run the following command to process the images.

```bash
python3 webcam_calibration_process_images.py
```

Processing the images will generate a YAML calibration file similar to the following file.

```bash
./webcam_calibration_images/<camera name>/<camera resolution>/camera_calibration_results_20231211211703.yaml
```

### Test the Camera

To make sure that your camera detects the ArUco markers on your tongs, __turn on the ring light__ and run the following code.

```bash
python3 webcam_teleop_interface.py
```

You should see images from the camera with green boxes drawn around detected ArUco markers.

## Running Teleop Tongs

After you've gotten everything setup, you can try out Teleop Tongs. Make sure to start with slow motions, to test your system, gain experience, and warm up.

### Start with Slow Motions

After setting everything up, run the following command without any command line arguments. __This will result in the robot moving at the slowest available speed while you ensure that everything is working properly and get used to using the teleoperation system.__

```bash
python3 dex_teleop.py
```

### When You're Ready, Try Fast Motions

Once you are confident that you have the system correctly configured and have learned to use it at the slowest speed, you can run the following command to try it at the fastest available speed. __The robot will move fast, so be very careful!__

```bash
python3 dex_teleop.py --fast
```

### Advanced: Multiprocessing with Shared Memory

To achieve better performance, you can run Teleop Tongs using two processes that communicate via shared memory.

First, run the interface process in a terminal. This process observes ArUco markers with the webcam to create goals for the robot's gripper.

```bash
python3 goal_from_teleop.py --multiprocessing
```

Second, run the robot process in a different terminal. This process receives gripper goals and attempts to achieve them by controlling the robot.

```bash
python3 gripper_to_goal.py --multiprocessing --fast
```

## Acknowledgment

Blaine Matulevich has been extremely helpful throughout the development of Teleop Tongs, including testing, providing feedback, discussing the system, and contributing ideas. The entire Hello Robot team provided essential support throughout, including helping with early versions of Stretch 3, which the entire company worked on intensely.
