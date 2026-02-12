#pragma once

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "so101.hpp"

class LeRobotJointStatePublisher : public rclcpp::Node
{
public:
    explicit LeRobotJointStatePublisher();

private:
    std::shared_ptr<SO101> init_lerobot_arm();

    std::string robot_name_;
    std::string port_;
    bool recalibrate_{false};
    
    std::shared_ptr<SO101> robot_;
};