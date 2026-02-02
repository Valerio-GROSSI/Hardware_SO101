#include "port_handler.hpp"

PortHandler::PortHandler(std::string port_name)
: port_name_(std::move(port_name)) {}

bool PortHandler::openPort() {
    return setBaudRate(baudrate_);
}

void PortHandler::closePort() {
    ser.close();
    is_open_ = false;
}

bool PortHandler::setupPort(int /*cflag_baud*/) {
    if (is_open_) {
        closePort();
    }

    boost::system::error_code ec;
    ser_.open(port_name_, ec);
    if (ec) {
        is_open_ = false;
        return false;
    }

    // Configuration minimale proche de pyserial:
    // bytesize=8, parity none, stopbits 1, timeout=0 (non-bloquant)
    ser_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_), ec);
    ser_.set_option(boost::asio::serial_port_base::character_size(8), ec);
    ser_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none), ec);
    ser_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one), ec);
    ser_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none), ec);

    if (ec) {
        closePort();
        return false;
    }

    is_open_ = true;

    // "reset_input_buffer" : on vide ce qu'il y a déjà
    clearPort();

    // Python: (1000.0 / baudrate) * 10.0  -> ms/byte pour 10 bits (start+8data+stop)
    tx_time_per_byte_ms_ = (1000.0 / static_cast<double>(baudrate_)) * 10.0;

    return true;
}