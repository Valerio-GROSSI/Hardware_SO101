#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>
#include <optional>

#include "Tables.hpp"

// struct Motor;
// struct MotorCalibration;

// class PacketHandler;
// class GroupSyncRead;
// class GroupSyncWrite;

// class PortHandler {
// public:
//     PortHandler(
//         const std::string& port_name
//     );

// private:

//     bool is_open_;
//     int baudrate_;
//     double packet_start_time_;
//     double packet_timeout_;
//     double tx_time_per_byte_;
//     bool is_using_;
//     std::string port_name_;
//     // self.ser: serial.Serial

//     bool openPort();
//     void closePort();
//     void ClearPort();
//     void setPortName(const std::string& port_name);
//     const std::string& getPortName() const;
//     bool setBaudRate(int baudrate);
//     int getBaudRate() const;
//     std::size_t getBytesAvailable() const;
//     std::vector<uint8_t> readPort(std::size_t length);
//     std::size_t writePort(const std::vector<uint8_t>& packet);
//     void setPacketTimeout(std::size_t packet_length);
//     void setPacketTimeoutMillis(double msec);
//     bool isPacketTimeout() const;
//     double getCurrentTime() const;
//     double getTimeSinceStart() const;

// protected:
//     bool setupPort(int cflag_baud);
//     int getCFlagBaud(int baudrate) const;

// };

class MotorsBus {
public:
    // struct Reg {
    //     std::uint16_t address{};
    //     std::uint8_t size{};
    // };
    // bool apply_drive_mode;
    // std::vector<int> available_baudrates;
    // int default_baudrate;
    // int default_timeout;
    // std::unordered_map<std::string, std::unordered_map<int, int>> model_baudrate_table;
    // std::unordered_map<std::string, std::unordered_map<std::string, Reg>> model_ctrl_table;
    // std::unordered_map<std::string, std::unordered_map<std::string, int>> model_encoding_table;
    // std::unordered_map<std::string, int> model_number_table;
    // std::unordered_map<std::string, std::uint32_t> model_resolution_table;
    // std::vector<std::string> normalized_data;

    MotorsBus(
        std::string& port,
        std::unordered_map<std::string, Motor>& motors,
        std::optional<std::unordered_map<std::string, MotorCalibration>>& calibration = std::nullopt
    );

    // virtual ~MotorsBus() = default; // base polymorphique

protected:
    std::string port_;
    std::unordered_map<std::string, Motor> motors_;
    std::unordered_map<std::string, MotorCalibration> calibration_;
    // PortHandler port_handler_;
    // PacketHandler packet_handler_;
    // GroupSyncRead sync_reader_;
    // GroupSyncWrite sync_writer_;
    // int _comm_success_;
    // int _no_error_;

    std::unordered_map<int, std::string> _id_to_model_dict_;
    std::unordered_map<int, std::string> _id_to_name_dict_;
    std::unordered_map<int, std::string> _model_nb_to_model_dict_;
    std::vector<std::string> models_;

    void _validate_motors();

    void connect(bool handshake = true);
    // bool is_connected() const;
    bool is_connected() const { return is_connected_impl(); }
    void _connect(bool handshake = true);
    // self.packet_handler.openPort()
    virtual bool open_port_impl() = 0;
    bool open_port_impl() { return port_handler_.openPort(); }
    // void _handshake();
    virtual void _handshake_impl() = 0;
    void _handshake() { _handshake_impl(); }
    // void set_timeout();
    virtual void set_timeout_impl(std::optional<int> timeout_ms) = 0;
    void set_timeout(std::optional<int> timeout_ms = std::nullopt) {
        set_timeout_impl(timeout_ms); }
    void _assert_motors_exist();
};
 