#pragma once

#include <cstdint>
#include <string>
// #include <vector>
// #include <chrono>
// #include <boost/asio.hpp>

namaspace params {
    inline constexpr int DEFAULT_BAUDRATE = 1000000;
    inline constexpr int LATENCY_TIMER = 50;
} // namespace params

class PortHandler {
public:
    explicit PortHandler(const std::string& port_name);
    bool openPort();
    void closePort();
    void clearPort();
    void setPortName(const std::string& port_name);
    const std::string& getPortName() const;
    bool setBaudRate(int baudrate);
    int getBaudRate() const;
    std::size_t getBytesAvailable();
    std::vector<std::uint8_t> readPort(std::size_t length);
    std::size_t writePort(const std::vector<std::uint8_t>& packet);
    void setPacketTimeout(std::size_t packet_length);
    void setPacketTimeoutMillis(double msec);
    bool isPacketTimeout();
    double getCurrentTimeMs() const;
    double getTimeSinceStartMs();
    double getCurrentTimeMs() const;
    double getTimeSinceStartMs();
    bool is_open() const { return is_open; }

protected:
    bool setupPort (int cflag_baud);
    int getCFlagBaud(int baudrate) const;

private:
    bool is_open_{false};
    int baudrate_{DEFAULT_BAUDRATE};
    double packet_start_time_ms_{0.0};
    double packet_timeout_ms_{0.0};
    double tx_time_per_byte_ms_{0.0};
    bool is_using_{false};
    std::string port_name_;
    boost::asio::io_context io_;
    boost::asio::serial_port ser_{io_};
};