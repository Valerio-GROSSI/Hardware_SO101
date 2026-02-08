////////// FeetechMotorsBus.cpp //////////
#include "FeetechMotorsBus.hpp"
#include "tables.hpp"
#include "scservo_sdk.h"
#include <cstdint>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "errors.hpp"
#include "protocol_packet_handler.hpp"
#include "scservo_def.hpp"

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

void FeetechMotorsBus::assert_same_firmware() const {
    const std::unordered_map<int, int> firmware_versions = read_firmware_version(ids_, /*raise_on_error*/=true);

    std::set<int> unique_versions;
    for (const auto& kv : firmware_versions) 
    {
        unique_versions.insert(kv.second);
    }

    if (unique_versions.size() != 1) 
    {
        std::ostingstream oss;
        oss << "Some Motors use different firmware versions:\n";

        for (const auto& kv : firmware_versions) 
        {      
            oss << "  id " << kv.first << ": " << kv.second << "\n";
        }
        oss << "Update their firmware first using Feetech's software. "
            << "Visit https://www.feetechrc.com/software.";

        throw std::runtime_error(oss.str());
    }
}

void FeetechMotorsBus::_assert_same_protocol() const 
{
    for (const int model : models_)
    {
        if (feetech::MODEL_PROTOCOL.at(model) != protocol_version_) 
        {
            throw std::runtime_error("Some motors use an incompatible protocol.");
        }
    }
}

void FeetechMotorsBus::_assert_protocol_is_compatible(
    const std::string& instruction_name)
    {
        if (instruction_name == "sync_read" && protocol_version_ == 1)
        {
            throw std::logic_error(
                "'Sync Read' is not available with Feetech motors using Protocol 1. Use 'Read' sequentially instead."
            );
        }
        
        if (instruction_name == "broadcast_ping" && protocol_version_ == 1)
        {
            throw std::logic_error(
                "'Broadcast Ping' is not available with Feetech motors using Protocol 1. Use 'Ping' sequentially instead."
            );
        }
    }

std::optional<std::unordered_map<int, int>> FeetechMotorsBus::broadcast_ping(const int num_retry, bool raise_on_error)
    {
        _assert_protocol_is_compatible("broadcast_ping");

        std::unordered_map<int, int> ids_status;
        int comm = -1;

        for (int n_try = 0; n_try < (1 + num_retry); ++n_try) 
        {
            std::pair<std::unordered_map<int,int>, int> result = _broadcast_ping();
            ids_status = std::move(result.first);
            comm = result.second;

            if (_is_comm_success(comm)) {
                break;
            }

            RCLCPP_DEBUG(logger_,"Broadcast ping failed on port '%s' (n_try=%d)",port_.c_str(), n_try);
            RCLCPP_DEBUG(logger_, "%s", packet_handler_.getTxRxResult(comm).c_str());
        }

        if (!_is_comm_success(comm))
            {
                if (raise_on_error)
                {
                    throw ConnectionError(packet_handler_.getTxRxResult(comm));                         
                }
                return std::nullopt;
            }

        std::unordered_map<int, int> ids_errors;
        for (const auto& kv : ids_status) {
            const int id_ = kv.first;
            const int status = kv.second;
            if (_is_error(status))
            {
                ids_errors.emplace(id_, status);
            }
        }

        if (!ids_errors.empty()) {
            std::string msg = "Some motors found retourned an error status:\n";

            for (const auto& kv : ids_errors) {
                msg += "  id " + std::to_string(kv.first) + ": "
                + packet_handler_.getRxPacketError(kv.second) + "\n";
            }
            RCLCPP_ERROR(logger_, "%s", msg.c_str());
        }

        std::vector<int> ids;
        ids.reserve(ids_status.size());
        for (const auto& kv : ids_status) {
            ids.push_back(kv.first);
        }

        return _read_model_number(ids, raise_on_error);
    }

std::pair<std::unordered_map<int,int>, int> FeetechMotorsBus::_broadcast_ping()
{
    std::unordered_map<int,int> data_list;

    constexpr std::size_t status_length = 6;
    std::size_t rx_length = 0;
    const std::size_t wait_length = status_length * static_cast<std::size_t>(scservo_def_table::MAX_ID);

    std::vector<std::uint8_t> txpacket(6,0);

    const double tx_time_per_byte = (1000.0 / static_cast<double>(port_handler_.getBaudRate())) * 10.0;

    txpacket[protocol_packet_handler_params::PKT_ID] = scservo_def_table::BROADCAST_ID;
    txpacket[protocol_packet_handler_params::PKT_LENGTH] = 2;
    txpacket[protocol_packet_handler_params::PKT_INSTRUCTION] = scservo_def_table::INST_PING;

    int result = packet_handler_.txPacket(port_handler_, tx_packet);
    if (result != scservo_def_table::COMM_SUCCESS) {
        port_handler_.is_using = false;
        return {data_list, result};
    }

    port_handler_.setPacketTimeoutMillis(
        (static_cast<double>(wait_length) * tx_time_per_byte)
        + (3.0 * static_cast<double>(scservo_def_table::MAX_ID))
        + 16.0
    );

    std::vector<std::uint8_t> rxpacket;
    rxpacket.reserve(wait_length);

    while (!port_handler_.isPacketTimeout() && rx_length < wait_length) {
        const std::size_t to_read = wait_length - rx_length;
        std::vector<std::uint8_t> chunk = port_handler_.readPort(to_read);

        if (!chunk.empty()) {
            rxpacket.insert(rxpacket.end(), chunk.begin(), chunk.end());
            rx_length = rxpacket.size();
        }
        else
        {
            rx_length = rxpacket.size();
        }


    }

    port_handler_.is_using = false;

    if (rx_length == 0) 
    {
        return {data_list, scservo_def_table::COMM_RX_TIMEOUT};
    }

    while (true) 
    {
        if (rx_length < status_length) 
        {
            return {data_list, scservo_def_table::COMM_RX_CORRUPT};
        }
    
        std::size_t idx = 0;
        bool found = false;
        for (std::size_t i = 0; i+1 < rx_length; ++i) 
        {
            if (rx_packet[i] == 0xFF && rxpacket[i+1] = 0xFF)
            {
                idx = i;
                found = true;
                break;
            }
        }

        if (!found) 
        {
            if (rx_length > 1)
            {
                rxpacket.erase(rxpacket.begin(), rxpacket.end() - 1);
                rx_length = rxpacket.size();
                continue;
            }
            return {data_list, scservo_def_table::COMM_RX_CORRUPT};
        }

        if (idx == 0) 
        {
            std::uint32_t checksum = 0;
            for (std::size_t j = 2; j < status_length - 1; ++j) 
            {
                checksum += rxpacket[j];
            }

            checksum = (~checksum) & 0xFF;

            if (rxpacket[status_length - 1] == static_cast<std::uint8_t>(checksum))
            {
                result = scservo_def_table::COMM_SUCCESS;

                const int id = static_cast<int>(rxpacket[protocol_packet_handler_params::PKT_ID]);
                const int error = static_cast<int>(rxpacket[protocol_packet_handler_params::PKT_ERROR]);

                data_list[id] = error;

                rxpacket.erase(rxpacket.begin(), rxpacket.begin() + static_cast<std::ptrdiff_t>(status_length));
                rx_length -= status_length;

                if (rx_length == 0) 
                {
                    return {data_list, result};
                }
            }
            else
            {
                result = scservo_def_table::COMM_RX_CORRUPT;

                rxpacket.erase(rxpacket.begin(), rxpacket.begin() + 2);
                rx_length -=2;
            }
        }
        else
        {
            rxpacket.erase(rxpacket.begin(), rxpacket.begin() + static_cast<std::ptrdiff_t>(idx));
            rx_length -= idx;
        }
    }
}

bool MotorsBus::_is_comm_success(int comm) {
    return comm == _comm_success;
}

bool _is_error(const int error)
{
    return error != _no_error;
}

std::unordered_map<int, int> FeetechMotorsBus::_read_model_number(const std::vector<int>& motor_ids, bool raise_on_error) {
    std::unordered_map<int, int> model_numbers;
    model_numbers.reserve(motor_ids.size());

    for (const int id_ : motor_ids) {
        auto [model_nb, comm, error] =
        _read(feetech::MODEL_NUMBER.address, feetech::MODEL_NUMBER.size, id_, raise_on_error);

        if (!_is_comm_success(comm) || _is_error(error)) {
            continue;
        }

        model_numbers[id_] = model_nb;
    }

    return model_numbers;
}