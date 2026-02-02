////////// FeetechMotorsBus.cpp //////////
#include "FeetechMotorsBus.hpp"
#include "tables.hpp"
#include "scservo_sdk.h"
#include <cstdint>

// void setCalibration(
//     std::optional<std::unordered_map<std::string, MotorCalibration>> calib
// ) {
//     if (calib && !calib->empty()) {
//         calibration_ = std::move(*calib);
//     } else {
//         calibration_ = std::unordered_map<std::string, MotorCalibration>{};
//     }
// }

FeetechMotorsBus::FeetechMotorsBus(
    const std::string& port,
    const std::unordered_map<std::string, Motor>& motors,
    const std::optional<std::unordered_map<std::string, MotorCalibration>>& calibration,
    int protocol_version
)
: MotorsBus(port, motors, calibration)
, available_baudrates_(feetech::SCAN_BAUDRATES)
, default_baudrate_(params::DEFAULT_BAUDRATE)
, default_timeout_(params::DEFAULT_TIMEOUT_MS)
, model_baudrate_table_(feetech::MODEL_BAUDRATE_TABLE)
, model_ctrl_table_(feetech::MODEL_CONTROL_TABLE)
, model_encoding_table_(feetech::MODEL_ENCODING_TABLE)
, model_number_table_(feetech::MODEL_NUMBER_TABLE)
, model_resolution_table_(feetech::MODEL_RESOLUTION)
, normalized_data_(feetech::NORMALIZED_DATA)
// , port_(port)
// , motors_(motors)
, protocol_version_(protocol_version)
{
    // setCalibration(calibration);
    _assert_same_protocol();
    port_handler_ = PortHandler(port_);
    // patch_setPacketTimeout(port_handler_);
    packet_handler_ = PacketHandler(protocol_version_);
    sync_reader_ = GroupSyncRead(port_handler_, packet_handler_, 0, 0);
    sync_writer_ = GroupSyncWrite(port_handler_, packet_handler_, 0, 0);
    _comm_success = scservo_def::COMM_SUCCESS;
    _no_error = 0x00;

    for (const auto& model : models_) {
    if (feetech::MODEL_PROTOCOL.at(model) != protocol_version_) {
        throw std::invalid_argument("Some motors are incompatible with protocol_version=" + std::to_string(protocol_version_))
    }
    }
}
