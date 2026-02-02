////////// MotorsBus.cpp //////////
#include "MotorsBus.hpp"

void setCalibration(
    std::optional<std::unordered_map<std::string, MotorCalibration>>& calib
) {
    if (calib && !calib->empty()) {
        calibration_ = *calib;
    } else {
        calibration_ = std::unordered_map<std::string, MotorCalibration>{};
    }
}

MotorsBus::MotorsBus(
    const std::string& port,
    const std::unordered_map<std::string, Motor>& motors,
    const std::optional<std::unordered_map<std::string, MotorCalibration>>& calibration
): port_(port), motors_(motors)
{
    setCalibration(calibration);

    for (const auto& [name, motor] : motors_) {
        _id_to_model_dict_.emplace(motor.id, motor.model);
    }

    for (const auto& [name, motor] : motors_) {
        _id_to_name_dict_.emplace(motor.id, name);
    }

    for (const auto& [model, number] : feetech::model_number_table) {
        _model_nb_to_model_dict_.emplace(number, model);
    }

    models_.reserve(motors_.size());
    for (const auto& [name, motor] : motors_) {
        models_.push_back(motor.model);
    }

    _validate_motors();
}

bool MotorsBus::is_connected() const
{
    return port_handler_ && port_handler_->is_open();
}

void MotorsBus::connect(bool handshake)
{
    if (is_connected()) {
        throw DeviceAlreadyConnectedError(
            port_ + " is already connected. Do not call MotorBus::connect() twice."
        );

    _connect(handshake);
    set_timeout();

    logger.debug("MotorBus connected.");
    }

}

void MotorsBus::_connect(bool handshake)
{
    try {
        if (!port_handler_->openPort()) {
            throw std::runtime_error(
                "Failed to open port '" + port_ + "'."
            );
        }
        else if (handshake) {
            _handshake();
        }
    }
    catch (const std::exception& /*e*/) {
        throw ConnectionError(
            "\nCould not connect on port '" + port_ + "'. Make sure you are using the correct port."
            "\nTry running `python lerobot/find_port.py`\n"
        );
    }
}





