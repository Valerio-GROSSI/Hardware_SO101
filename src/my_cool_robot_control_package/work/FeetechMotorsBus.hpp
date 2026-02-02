////////// FeetechMotorsBus.hpp //////////
#pragma once

#include <string>
#include <optional>
#include <cstdint>
#include <unordered_map>

#include "MotorsBus.hpp"
#include "Motor.hpp"
#include "MotorCalibration.hpp"
#include "tables.hpp"
#include "scservo_sdk.h"

namespace params {
    inline constexpr int DEFAULT_PROTOCOL_VERSION = 0;
    inline constexpr int DEFAULT_BAUDRATE = 1'000'000;
    inline constexpr std::int32_t DEFAULT_TIMEOUT_MS = 1000;
} // namespace params

class FeetechMotorsBus : public MotorsBus {
public:
    static constexpr bool apply_drive_mode = true;

FeetechMotorsBus(
    const std::string& port,
    const std::unordered_map<std::string, Motor>& motors,
    const std::optional<std::unordered_map<std::string, MotorCalibration>>& calibration = std::nullopt,
    int protocol_version = params::DEFAULT_PROTOCOL_VERSION
    );

private:
    int protocol_version_{params::DEFAULT_PROTOCOL_VERSION};
    int default_baudrate_{params::DEFAULT_BAUDRATE};
    std::int32_t default_timeout_{params::DEFAULT_TIMEOUT_MS};

    scs::PortHandler port_handler;
    scs::PacketHandler packet_handler;
    scs::GroupSyncRead sync_reader;
    scs::GroupSyncWrite sync_writer;
    scs::COMM_SUCCESS _comm_success;
    std::uint8_t _no_error = 0x00; 

private:
    void _assert_same_protocol();
    static void patch_setPacketTimeout(scs::PortHandler& port_handler);
};