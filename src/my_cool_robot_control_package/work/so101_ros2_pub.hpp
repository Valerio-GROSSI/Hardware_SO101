# pragma once

#include <string>
#include "SO101.hpp"

std::optional<SO101> init_lerobot_arm(
    const std::string& port,
    const std::string& robot_name,
    bool recalibrate
);

// Dans my_cool_robot_hardware.cpp MyCoolRobotHardware::on_configure(), ajouter:
// std::string port = info_.hardware_parameters.at("port");
// std::string robot_name = info_.hardware_parameters.at("robot_name");
// bool recalibrate = (info_.hardware_parameters.at("recalibrate") == "true");

// <ros2_control name="so101" type="system">
//  <hardware>
//   <plugin>my_pkg/MyCoolRobotHardware</plugin>
//    <param name="port">/dev/ttyACM0</param>
//    <param name="recalibrate">true</param>
//  </hardware>
// <ros2_control>