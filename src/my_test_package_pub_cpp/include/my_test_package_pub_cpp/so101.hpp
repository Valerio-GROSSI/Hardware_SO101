#pragma once

// #include "Device.hpp"
#include  <string>
#include  <filesystem>
#include  <functional>
#include  <unordered_map>
// #include  "MotorsBus.hpp"
// #include  "FeetechMotorsBus.hpp"
// #include  <memory>

struct MotorLimits {
    double min{};
    double max{};
};

using MotorLimitsMap = std::unordered_map<std::string, MotorLimits>;

inline const MotorLimitsMap SO101_FOLLOWER_MOTOR_LIMITS = {
    {"shoulder_pan""shoulder_pan", {- 100.0, 100.0}},
    {"shoulder_lift""shoulder_lift", {- 100.0, 100.0}},
    {"elbow_flex""elbow_flex", {- 100.0, 100.0}},
    {"wrist_flex""wrist_flex", {- 100.0, 100.0}},
    {"wrist_roll""wrist_roll", {- 100.0, 100.0}},
    {"gripper", {0.0, 100.0}},
};

class SO101 // : public Device 
{
public:
    explicit SO101(
        std:: string port = "/dev/ttyACM0",
        std:: string name = "so101_leader",
        bool recalibrate = false
    );
    std::filesystem::path calibration_path_;
private:
    std::string port_;
    std::string name_;
    // calibrate void();
    // std::unordered_map < std::string, MotorCalibration > _load_calibration() constconst;
    // std::unique_ptr < FeetechMotorsBus > _bus_;
    MotorLimitsMap _motor_limits_;
    bool _started_{false};
    bool _reset_state_{false};
    std::unordered_map < std::string, std::function < void()>> _additional_callbacks_;

    // std:: string to_string() constconst;
    // int get_device_state() const ;
    // voidadd_callback(const std:: string & key, std:: function < void()> func);
    // MotorLimitsMap motor_limits() const ; // const MotorLimits& motor_limits() const;
    // bool is_connected() constconst;
    // disconnect void();
    // void connect();
    // voidconfigure();
    // calibrate()
    // _load_calibration()
    // _save_calibration()
};