# Robot Pose Guidance (So101 Robotic Arm)

This project focuses on controlling a 6-DoF So101 robotic arm, referred to as the Follower arm, using several methods. Its goal is to provide a complete pipeline for trajectory execution and manipulation tasks.

The So101 robot is part of the LeRobot ecosystem. Its hardware interface follows the original LeRobot design but has been fully reimplemented in C++ instead of Python and integrated with ros2_control.

The robot can be controlled using standard ros2_control controllers, teleoperated through a Leader arm, or driven by motion plans generated with MoveIt 2.

## How to Run

1. Calibrate the So101 Follower arm :
```bash
ros2 run so101_robot_hardware my_so101_ros2_calib_executable
```

2. Launch ros2_control and controllers :
```bash
ros2 launch so101_robot_bringup my_so101_robot.launch.py
```

3. Launch MoveIt and RViz interface for motion planning :
```bash
ros2 launch so101_robot_hardware so101_robot_moveit
```

## Results

<br>

<p align="center">
  <b>So101 Follower Arm</b><br>
  <img src="./so101.gif" width="80%" />
</p>

<br>

<p align="center">
  <b>Sample MoveIt Planning Groups Configuration</b><br>
  <img src="./so101_moveit.gif" width="80%" />
</p>
