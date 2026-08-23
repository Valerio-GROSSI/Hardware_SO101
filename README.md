# Robot Pose Guidance (So101 Robotic Arm)

This project focuses on controlling a 6-DoF So101 robotic arm, referred to as the Follower arm, using several methods. Its goal is to provide a complete pipeline for trajectory execution and manipulation tasks.

The So101 robot is part of the LeRobot ecosystem. Its hardware interface follows the original LeRobot design but has been fully reimplemented in C++ instead of Python and integrated with ros2_control.

The robot can be controlled using standard ros2_control controllers, teleoperated through a Leader arm, or driven by motion plans generated with MoveIt 2.

One of the project’s next objectives is to train the real robot to perform manipulation tasks, such as pick-and-place, using Isaac Lab for imitation learning and reinforcement learning, as well as the LeRobot framework.

<br>

## How to Run

### Calibration

Calibrate the So101 Leader arm :
```bash
ros2 run so101_robot_hardware my_so101_ros2_calib_executable --ros-args -p robot_name:=so101_leader port:=/dev/ttyACM0
```

Calibrate the So101 Follower arm :
```bash
ros2 run so101_robot_hardware my_so101_ros2_calib_executable --ros-args -p robot_name:=so101_follower port:=/dev/ttyACM1
```


### Script-Based Leader–Follower Teleoperation - (without ros2_control)

1. In a first terminal, run the So101 Publisher arm executable :
```bash
ros2 run so101_robot_hardware my_so101_ros2_pub_executable --ros-args -p robot_name:=so101_leader port:=/dev/ttyACM0 
```
2. In a second terminal, run the So101 Subscriber arm executable:
```bash
ros2 run so101_robot_hardware my_so101_ros2_sub_executable --ros-args -p robot_name:=so101_follower port:=/dev/ttyACM1
```

``` --ros-args -p recalibrate:=true ``` to recalibrate at the start of execution as well


### Direct ROS 2 Leader–Follower Teleoperation - (ros2_control-based)

1. In a first terminal, run the So101 Publisher arm launch file :
```bash
ros2 launch so101_robot_bringup publisher_arm.launch.py use_rviz:=true namespace:=leader
```

2. In a second terminal, run the So101 Subscriber arm launch file :
```bash
ros2 launch so101_robot_bringup subscriber_arm.launch.py use_rviz:=true namespace:=follower
```

3. In a third terminal, run the Publisher-Subscriber bridge executable :
```bash
ros2 run so101_robot_bringup bridge_forward
```

It uses the Publisher arm’s joint states as position commands for the Subscriber arm (via a `forward_command_controller`).  
The `namespace` must match the arm model. Either model can be configured to act as the Publisher or Subscriber arm.  
`recalibrate` must be set to `false` (default value). Indeed, recalibration implies `std::cin.get()` not supported by ROS 2 launch setup.



### Control of the Robotic Arm using standard ros2_control controllers

1. In a first terminal, run the So101 Subscriber arm launch file :
```bash
ros2 launch so101_robot_bringup subscriber_arm.launch.py use_rviz:=true namespace:=follower use_sim:=false
```

This launch file can be run without real hardware, in simulation, with either Gazebo Sim (`gz_sim`) or Gazebo Classic (`gazebo_classic`).  
For testing the ROS 2 control pipeline without real hardware nor launching a simulator, set `using mock_hardware` to true.  

2. In a second terminal, send a position command:
```bash
ros2 topic pub --once follower/forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}"
```

To use the ROS 2 Control `joint_trajectory_controller` instead:
```bash
ros2 control switch_controllers --controller-manager /follower/controller_manager --deactivate forward_position_controller --activate joint_trajectory_controller
```
```bash
ros2 topic pub --once follower/joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory /
"{joint_names:['joint1','joint2','joint3','joint4','joint5','joint6'],points: [{positions: [0.3, 0.1, 0.2, 0.3, 0.1, 0.2],time_from_start: {sec: 5, nanosec: 0}}]}"
```


### Control of the Robotic Arm using Moveit motion plannig

1. Launch MoveIt and RViz interface for motion planning:
```bash
ros2 launch so101_robot_moveit moveit_real.launch.py
```

2. Utiliser l'interface RVIZ pour génerer les commandes (via group_state)


### Example output - Control of the Robotic Arm using Moveit motion plannig

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
