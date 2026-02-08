////////// FeetechMotorsBus.hpp //////////
#pragma once

#include <string>
#include <optional>
#include <cstdint>
#include <unordered_map>
#include <utility>

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

protected:
    // bool is_connected_impl() const override { return port_handler_.is_open; }
    // void _handshake_impl() override {
    //     { _assert_motors_exist(); 
    //       _assert_same_firmware(); }
    // }
    // bool open_port_impl() override { return port_handler_.openPort(); }

    // void set_timeout_impl(std::optional<int> timeout_ms) override
    // {
    //     const int timeout_ms_ = timeout_ms.value_or(default_timeout_);
    //     port_handler_.setPacketTimeout(timeout_ms_);
    // }

    void assert_same_firmware();
    void assert_same_protocol();
    void FeetechMotorsBus::_assert_protocol_is_compatible(const std::string& instruction_name);
    std::optional<std::unordered_map<int, int>> broadcast_ping(const int& num_retry = 0, bool raise_on_error = false);
    std::pair<std::unordered_map<int,int>, int> _broadcast_ping();
    bool _is_comm_success(int comm);
    std::unordered_map<int, int> _read_model_number(const std::vector<int>& motor_ids, bool raise_on_error);
    static void patch_setPacketTimeout(scs::PortHandler& port_handler);

private:
    int protocol_version_{params::DEFAULT_PROTOCOL_VERSION};
    int default_baudrate_{params::DEFAULT_BAUDRATE};
    std::int32_t default_timeout_{params::DEFAULT_TIMEOUT_MS};

    scs::PortHandler port_handler_;
    scs::PacketHandler packet_handler_;
    scs::GroupSyncRead sync_reader_;
    scs::GroupSyncWrite sync_writer_;
    scs::COMM_SUCCESS _comm_success_;
    std::uint8_t _no_error = 0x00; 

};