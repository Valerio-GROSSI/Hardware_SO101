# Robot Pose Guidance (So101 Robotic Arm)

This project focuses on controlling the So101 Follower, a 6-DoF robotic arm, using several methods. It provides a complete pipeline for trajectory execution and (currently under development) manipulation tasks.

The So101 robot is part of the LeRobot ecosystem. The hardware interface used in this project follows the original LeRobot design but has been fully reimplemented in C++ instead of Python and integrated with ros2_control.

The robot can be controlled using standard ros2_control controllers, teleoperated using an So101 Leader arm, or driven by motion plans generated with MoveIt 2 from either joint-space targets or Cartesian end-effector targets.

One of the project’s current objective is to train the robot to perform manipulation tasks, such as pick-and-place using several approaches:
  - Detecting objects from camera images using an AI-based perception model, in order to determine a Cartesian end-effector goal, and planning the corresponding trajectory with MoveIt 2.
  - Training reinforcement-learning policies in simulation using Isaac Lab and RL libraries, then deploying the trained policies on the physical robot.
  - Recording demonstrations in simulation with Isaac Lab, augmenting the demonstration dataset using Isaac Lab Mimic, training a policy with NVIDIA Isaac GR00T, and deploying it on the physical robot.
  - Recording real-world demonstrations with LeRobot, training imitation-learning policies from the collected dataset, and deploying them on the physical robot.

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

<br>

### Script-Based Leader–Follower Teleoperation - (without ros2_control)

1. In a first terminal, run the So101 Publisher arm executable :
```bash
ros2 run so101_robot_hardware my_so101_ros2_pub_executable --ros-args -p robot_name:=so101_leader port:=/dev/ttyACM0 
```
2. In a second terminal, run the So101 Subscriber arm executable:
```bash
ros2 run so101_robot_hardware my_so101_ros2_sub_executable --ros-args -p robot_name:=so101_follower port:=/dev/ttyACM1
```
 
Either model can be configured to act as the Publisher or Subscriber arm, just swap `robot_name` values.  

``` --ros-args -p recalibrate:=true ``` to recalibrate the arm at the start of execution as well. 

<br>

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

It uses the Publisher arm’s joint states as position commands for the Subscriber arm (`forward_command_controller`).  
Either model can be configured to act as the Publisher or Subscriber arm, just use `arm` parameter (leader | follower).  
Make sure to preserve the `namespace` parameter at these exact values, it is used to avoid conflicts between identical topic and node names.  
`recalibrate` must be kept at false (default value). Indeed, recalibration implies `std::cin.get()` not supported by ROS 2 launch setup.  

<br>

### Control of the Robotic Arm using standard ros2_control controllers

1. In a first terminal, run the So101 Subscriber arm launch file :
```bash
ros2 launch so101_robot_bringup subscriber_arm.launch.py use_rviz:=true
```

This launch file can be run without real hardware in simulation (parameter bool `use_sim`), with either Gazebo Sim (parameter bool `gz_sim`) or Gazebo Classic (parameter bool `gazebo_classic`).  
For testing ROS 2 control pipeline without real hardware nor launching a simulator, use parameter bool `use_mock_hardware`.

2. In a second terminal, send a position command:
```bash
ros2 topic pub --once forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}"
```

To use the ROS 2 control `joint_trajectory_controller` instead:
```bash
ros2 control switch_controllers --controller-manager /controller_manager --deactivate forward_position_controller --activate joint_trajectory_controller
```
```bash
ros2 topic pub --once joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory /
"{joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6'], points: [{positions: [0.3, 0.1, 0.2, 0.3, 0.1, 0.2], time_from_start: {sec: 5, nanosec: 0}}]}"
```

<br>

### Control of the Robotic Arm using Moveit motion planning

1. 
```bash
ros2 launch so101_robot_bringup subscriber_arm.launch.py use_moveit:=true
```

2.
```bash
ros2 launch my_so101_robot_moveit_package move_group.launch.py
```

3. Use the RViz interface to generate commands (via group_state)

4.
```bash
ros2 action send_goal --feedback \
  /move_action \
  moveit_msgs/action/MoveGroup \
  "{
    request: {
      group_name: arm,
      num_planning_attempts: 10,
      allowed_planning_time: 5.0,
      max_velocity_scaling_factor: 0.05,
      max_acceleration_scaling_factor: 0.05,
      start_state: {
        is_diff: true
      },
      goal_constraints: [
        {
          name: joint_goal,
          joint_constraints: [
            {
              joint_name: joint1,
              position: 0.0,
              tolerance_above: 0.01,
              tolerance_below: 0.01,
              weight: 1.0
            },
            {
              joint_name: joint2,
              position: -1.0,
              tolerance_above: 0.01,
              tolerance_below: 0.01,
              weight: 1.0
            },
            {
              joint_name: joint3,
              position: 1.0,
              tolerance_above: 0.01,
              tolerance_below: 0.01,
              weight: 1.0
            },
            {
              joint_name: joint4,
              position: 0.5,
              tolerance_above: 0.01,
              tolerance_below: 0.01,
              weight: 1.0
            },
            {
              joint_name: joint5,
              position: 0.0,
              tolerance_above: 0.01,
              tolerance_below: 0.01,
              weight: 1.0
            }
          ]
        }
      ]
    },
    planning_options: {
      plan_only: false,
      look_around: false,
      replan: false
    }
  }"
```

5.
```bash
ros2 action send_goal --feedback \
  /move_action \
  moveit_msgs/action/MoveGroup \
  "{
    request: {
      group_name: arm,
      num_planning_attempts: 20,
      allowed_planning_time: 10.0,
      max_velocity_scaling_factor: 0.05,
      max_acceleration_scaling_factor: 0.05,
      start_state: {
        is_diff: true
      },
      goal_constraints: [
        {
          name: tcp_position,
          position_constraints: [
            {
              header: {
                frame_id: base
              },
              link_name: gripperframe,
              target_point_offset: {
                x: 0.0,
                y: 0.0,
                z: 0.0
              },
              constraint_region: {
                primitives: [
                  {
                    type: 2,
                    dimensions: [0.005]
                  }
                ],
                primitive_poses: [
                  {
                    position: {
                      x: 0.20,
                      y: 0.00,
                      z: 0.15
                    },
                    orientation: {
                      x: 0.0,
                      y: 0.0,
                      z: 0.0,
                      w: 1.0
                    }
                  }
                ]
              },
              weight: 1.0
            }
          ]
        }
      ]
    },
    planning_options: {
      plan_only: true,
      look_around: false,
      replan: false
    }
  }"
```

<br>

## Example - Control of the Robotic Arm using Moveit motion planning

<p align="center">
  <b>So101 Follower Arm</b><br>
  <img src="./so101.gif" width="80%" />
</p>

<p align="center">
  <b>Sample MoveIt Planning Groups Configuration</b><br>
  <img src="./so101_moveit.gif" width="80%" />
</p>
