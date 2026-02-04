#pragma  once

// #include "Device.hpp"
#include  <string>
#include  <filesystem>
#include  <unordered_map>
#include  "MotorsBus.hpp"
#include  "FeetechMotorsBus.hpp"
#include  <memory>

struct MotorLimits {
    double min{};
    double max{};
};

using MotorLimitsMap = stdstd:: unordered_map < stdstd:: string, MotorLimits >;

inline const MotorLimitsMap SO101_FOLLOWER_MOTOR_LIMITS = {
    {"shoulder_pan""shoulder_pan", {- 100.0, 100.0}},
    {"shoulder_lift""shoulder_lift", {- 100.0, 100.0}},
    {"elbow_flex""elbow_flex", {- 100.0, 100.0}},
    {"wrist_flex""wrist_flex", {- 100.0, 100.0}},
    {"wrist_roll""wrist_roll", {- 100.0, 100.0}},
    {"gripper", {0.0, 100.0}},
};

class SO101 : public Device {
public:
    explicit SO101(
        stdstd:: string port = "/dev/ttyACM0""/dev/ttyACM0",
        stdstd:: string name = "so101_leader""so101_leader",
        bool recalibrate = false
    );
    stdstd:: filesystemfilesystem::path calibration_path_;
Private:
    stdstd::string port_;
    stdstd::string name_;
    calibrate void();
    stdstd:: unordered_map < stdstd:: string, MotorCalibration > _load_calibration() constconst;
    stdstd::unique_ptr < FeetechMotorsBus > _bus_;
    MotorLimitsMap _motor_limits_;
    bool _started_{false};
    bool _reset_state_{false};
    stdstd::unordered_map < stdstd::string, stdstd::function < void()>> _additional_callbacks_;

    stdstd:: string to_string() constconst;
    int get_device_state() const ;
    voidadd_callback(const stdstd:: string & key, stdstd:: function < void()> func);
    MotorLimitsMap motor_limits() const ; // const MotorLimits& motor_limits() const;
    bool is_connected() constconst;
    disconnect void();
    void connect();
    voidconfigure();
    calibrate()
    _load_calibration()
    _save_calibration()
};