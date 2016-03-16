# Turtlebot SLAM (with RTAB-Map, Hand-Gestures, Face Recognition & AR Code Tracking)

## About:

A ROS project developed as part of Master fo Science in Robotics (MSR) program at Northwestern University (NU)

## Goal:

Navigation of turtlebot using hand-gestures to find target locations marked with AR codes and/or to find a specific person using face-recognition

## Overview:

Refer to my portfolio entry at http://patilnabhi.github.io/portfolio/tbotnav

## Tutorial:

The following sections describe contents of this package and how to use them:

### 1. Prequisites:

The following **hardware** is required for complete execution of project:

1. Turtlebot 2 (with Kobuki base)
2. Kinect (mounted on turtlebot)
3. A computer with webcam and installed with ROS Indigo and Ubuntu 14.04 (mounted/connected on turtlebot)
4. A second computer installed with ROS Indigo and Ubuntu 14.04 for visualization (Rviz) and hand gesture API
5. A second depth camera is *preferred* (Asus Xtion Pro or Kinect) for hand gesture recognition; this will be connected to the second computer
6. Printed AR codes from 2 to 5 that could be placed aywhere around turtlebot

The following **packages** need to be installed:

1. [turtlebot packages] - SLAM packages
2. [rtabmap_ros] - RTAB-Map package 
3. [openni2_launch] - required if using Asus Xtion Pro for hand gesture recognition
4. [freenect_launch] - required for `3dsensor.launch` with turtlebot navigation
5. [ar_track_alvar] - to recognize AR code tags and move turtlebot towards them

The following needs to be **setup** in order to run all nodes:

1. [Turtlebot setup]
2. [Turtlebot networking setup]
3. Second depth camera setup (Asus Xtion Pro camera) - Please edit the `camera_id` parameter value in `asus_cam.launch` with appropriate value for your camera. To find your `camera_id` value, launch the OpenNI2 driver and look for `device_id`:
```
roslaunch openni2_launch openni2.launch
```

### 2. 'tbotnav' Package Contents

This package consists the following **nodes**:

**Navigation:**

1. `move_to_pose.py` - this node is used to move turtlebot to a specific pose and uses the `MoveBaseAction` and `MoveBaseGoal` messages to do so
2. `run_tbot_routine.py` - this the main node that performs the entire routine, combining various nodes, as outlined in the Overview section. This node subscribes to the following topics:  
	a. `ar_pose_marker` - to determine the id and pose estimate of AR code
	b. `num_fingers` - the detected number of fingers using hand gestures
	c. `face_names` - get the names of people detected during face recognition mode
	d.  `odom` - this is required to know the current odometry of the robot and perform odom correction [implemnetation in progress]
	 
**Hand Gesture Recognition:**

3. `fingers_recog.py` - this node takes a input image and outputs an image with detected number of fingers
4. `get_hand_gestures.py` - this ndoe subscribes to a depth image `/asus/depth/image_raw`, processes the image using `finger_recog.py` and publishes the detected number of fingers at the topic `num_fingers`. This ndoe also outputs an image window showing the depth feed with hand and detected number of fingers.

**Face Recognition:**

5. `train_faces.py` - this node subscribes to a rgb image stream from webcam, detects faces, captures faces for training (using Fisherfaces algorithm) and saves the trained data in a xml file, to be used in face recognition.
6. `face_recog.py` - this node subscribes to rgb image stream from webcam, loads the trained data file from above and performs face recognition.
7. `gui_face.py` - this node launches a simple GUI making it easier for users to input their name, captures their faces, train the data and finally run the recognition API

This package consists the following *launch* files:

1. `move_base_rtabmap.launch` - (to be launched on turtlebot computer) this file performs the following:  
	a. Launches `minimal.launch` from `turtlebot_bringup` package
	b. Runs the `move_base` node for navigation
	c. Runs the `rtabmap` node 
	d. Launches `alvar.launch` for AR code detection
	e. Runs the `usb_cam` node for face recognition

2. `tbot_routine.launch`:
	a. Launches `tbot_routine_rviz.launch` and runs the `rviz` node, opening up a Rviz visualization window
	b. Launches `asus_cam.launch`, launching `openni2.launch` with custom `camera_id`
	c. Runs the `get_hand_gestures` node

<!-- This package consists the following *config* files:

1. `costmap_params.yaml`
2. `global_costmap_params.yaml`
3. `local_costmap_params.yaml`
4. `base_local_planner.yaml`
5. `global_planner_params.yaml`
6. `move_base_params.yaml`
7. `dwa_planner_params.yaml`
8. `tbot_rtabmap.rviz` -->

### 3. Step-by-step guide:

1. Turn on turtlebot and ensure that networking is setup correctly
2. Connect ASUS Xtion Pro to your 'second' computer for hand gesture recognition
3. Source the turtlebot workspace. For e.g, if your workspace is called 'tbot_ws', enter in command line:
```
source ~/tbot_ws/devel/setup.bash
```  
4. On turtlebot computer, run:
```
roslaunch tbotnav move_base_rtabmap.launch
```  
5. On your 'second' computer, run:
```
roslaunch tbotnav tbot_routine.launch
```  
6. On your 'second' computer, in another terminal window, run:
```
rosrun tbotnav run_tbot_routine.py
```  
7. Follow the instructions on the window launched in (6)


## Demo:

* Video