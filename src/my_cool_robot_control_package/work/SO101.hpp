#pragma once

#include <string>
#include <unordered_map>
#include <functional>
#include <filesystem>

#include "Device.hpp"
#include "FeetechMotorsBus.hpp"
#include "Motor.hpp"
#include "MotorCalibration.hpp"
#include "MotorNormMode.hpp"

// Exemple: structure de limites (à adapter à ton vrai type)
struct MotorLimits {
    double min{};
    double max{};
};
using MotorLimitsMap = std::unordered_map<std::string, MotorLimits>;

// Déclaré ailleurs (comme ta constante Python)
extern const MotorLimitsMap SO101_FOLLOWER_MOTOR_LIMITS;

class SO101 : public Device {
public:
    explicit SO101(
        std::string port = "/dev/ttyACM0",
        std::string name = "so101_leader",
        bool recalibrate = false
    );

    // si besoin
    // void connect();

private:
    void calibrate(); // équivalent self.calibrate()
    std::unordered_map<std::string, MotorCalibration> load_calibration() const;

private:
    std::string port_;
    std::string name_;
    std::filesystem::path calibration_path_;

    FeetechMotorsBus bus_;                  // équivalent self._bus
    MotorLimitsMap motor_limits_;           // équivalent self._motor_limits

    bool started_{false};                   // self._started
    bool reset_state_{false};               // self._reset_state

    // équivalent self._additional_callbacks = {}
    // (ex: callbacks nommés -> fonctions)
    std::unordered_map<std::string, std::function<void()>> additional_callbacks_;
};
