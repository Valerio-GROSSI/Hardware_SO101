// #ifndef PORT_HANDLER_HPP
// #define PORT_HANDLER_HPP

// 

#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <chrono>
#include "../lib/FTServo_Linux/src/SCServo.h"

namespace params_port_handler {
    constexpr int DEFAULT_BAUDRATE = 1'000'000;
    constexpr int LATENCY_TIMER = 50;
}; // namespace params_port_handler

class PortHandler {
public:
    // Constructeur
    explicit PortHandler(const std::string& port_name);

    //Méthodes publiques
    bool openPort();
    void closePort();

private:
    bool is_open_ = false;
    int baudrate_{params_port_handler::DEFAULT_BAUDRATE};
    double packet_start_time_ = 0.0;
    double packet_timeout_ = 0.0;
    double tx_time_per_byte_ = 0.0;
    bool is_using_ = false;
    std::string port_name_;
    SMS_STS servo;

    // Méthodes privées
    void clearPort();
    void setPortName(const std::string& port_name);
    const std::string getPortName() const;
    bool setBaudRate(int baudrate);
    int getBaudRate() const;
    int getBytesAvailable() const;
    std::vector<uint8_t> readPort(std::size_t length);
    int writePort(const std::vector<uint8_t>& packet);
    void setPacketTimeout(std::size_t packet_length);
    void setPicketTimeoutMillis(double msec);
    bool isPacketTimeout();
    double getCurrentTime() const;
    double getTimeSinceStart();
    bool setupPort(int cflag_baud);
    int getCFlagBaud(int baudrate);
};

// #endif // PORT_HANDLER_HPP