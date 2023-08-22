# Pick and Place Using Computer Vision Technics
We utilized Unity Game Engine as our main simulation tool, and Kinova Jaco robotic arm. The robotic arm uses machine learning models to detect objects with computer vision, then picks up detected objects and places them in a predetermined position.\
\
We implemented this project during our internship at Athena RC.

## Jaco Robotic Arm
Jaco Kinova arm is a 6DOF robot arm that can perform a wide range of tasks. In this project we use the model j2n6s200 Jaco arm.

## Configuration
In order to replicate the project in your computer, follow these steps:
1. Clone the repo
2. Navigate to arm_ws, execute:
   
   ```
   catkin_make
   source devel/setup.bash
   ```
3. Open Jaco_arm project in Unity. We used Unity version 2020.3.11f1.
4. Follow the instructions in Unity Robitics Hub tutorial [Quick Installation Instructions](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/quick_setup.md)
5. Change ROS IP Address in Unity:
    * Go to ROS Settings and change the ROS IP Address.
    * Find ROSConnectionPrefab and change the IP there too.
6. Install the python packages required for the detection models. They can be found in object_detection/requirements.txt.

## Gazebo Simulator
This repository contains files for simulating the robot arm in gazebo simulator using ROS.
In order to launch the robot arm in Gazebo and control it using RViz, execute:

```
roslaunch moveit_config my_robot_planning_execution.launch
```
## Unity Game Engine
We created 3 differrent Unity Scenes (found in Assets/Scenes directory)
  ### 1. Simple Pick and Place
  This scene is mostly based on the [Pick And Place](https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials/pick_and_place) tutorial from Unity Robotics Hub. In a terminal execute:
  ```
  roslaunch jaco_unity simple_pick_place.launch
  ```
  Open SampleScene and start the Unity simulation.
  ![Simple_Pick_Place](https://github.com/panagiotamoraiti/Jaco_Robotic_Arm/assets/72858165/be48df2c-46b3-482d-b84c-56752ed6ce5f)

  ### 2. Grape Leaves Disease Detection and Grasping
  In this scene we have a few leaves, some with disease spots and some healthy. The robot carries a camera near its end effector, takes a photo of the table and if a spot is detected, the arm picks up the leaf and places it in the bin. To run this, execute in a terminal:
  ```
  roslaunch jaco_unity camera_pick_place.launch
  ```
  Then in another terminal:
  ```
  python3 leaves_detection_model.py
  ```
  Open GrapeLeavesScene and start the Unity simulation.
  
  ### 3. Garbage Detection and Sorting
  In this scene we have some plastic bottles and tin cans. The robot carries a camera near its end effector, takes a photo of the table and places the object classified with the highest confidence in the suitable bin. The classification is based on the material (metal, plastic, paper and glass). To run this, execute in a terminal:
  ```
  roslaunch jaco_unity camera_pick_place.launch
  ```
  Then in another terminal:
  ```
  python3 garbage_detection_model.py
  ```
  Open GarbageSortingScene and start the Unity simulation.

## Object detection
We implemented two python scripts for garbage detection and leaf disease detection, using two already trained models we found. The scripts interact with the Unity environment using the text files found in Jaco_arm/temp_txt directory.
