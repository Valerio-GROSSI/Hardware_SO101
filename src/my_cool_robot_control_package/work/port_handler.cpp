// #include "../lib/FTServo_Linux/include/SCServo.h"
// #include <string>

#include "port_handler.hpp"

PortHandler::PortHandler(const std::string& port_name):
    port_name_(port_name){}

bool PortHandler::openPort()
{
    return setBaudRate(baudrate_);
}

void PortHandler::closePort()
{
    servo.end(); // attention modifier fonction end() dans SCSerial.cpp
    is_open_ = false;
}

void PortHandler::clearPort() // equivalent to ser.flush() (ne vide pas le buffer d'écriture)
{
    servo.drainOutput(); // fonction rajoutée dans SCSerial.cpp
}

void PortHandler::setPortName(const std::string& port_name)
{
    port_name_ = port_name;
}

const std::string PortHandler::getPortName() const
{
    return port_name_;
}

bool PortHandler::setBaudRate(int baudrate)
{
    const int baud = getCFlagBaud(baudrate);
    
    if (baud <= 0) {
        // setupPort(38400);
        // baudrate_ = baudrate;
        return false;
    }
    else
    {
        baudrate_ = baudrate;
        return setupPort(baud);
    }
}

int PortHandler::getBaudRate() const
{
    return baudrate_;
}

int PortHandler::getBytesAvailable() const
{
    return servo.bytesAvailable(); // fonction rajoutée dans SCSerial.cpp
}

std::vector<uint8_t> PortHandler::readPort(std::size_t length)
{
    std::vector<uint8_t> out(length);
    const int n = servo.readSCS(out.data(), static_cast<int>(length));
    if (n <= 0) {
        out.clear();
        return out;
    }
    out.resize(static_cast<std::size_t>(n));
    return out;
}

int PortHandler::writePort(const std::vector<uint8_t>& packet)
{
    if (packet.empty()) {
        return 0;
    }

    // Écriture dans le buffer applicatif FTServo
    servo.writeSCS(
        const_cast<unsigned char*>(packet.data()),
        static_cast<int>(packet.size())
    );

    // Commit vers le kernel (équivalent ser.write)
    servo.wFlushSCS();

    // Nombre d’octets écrits
    return static_cast<int>(packet.size());
}

void PortHandler::setPacketTimeout(std::size_t packet_length)
{
    packet_start_time_ = getCurrentTime(); // en ms

    packet_timeout_ =
        (tx_time_per_byte_ * static_cast<double>(packet_length)) +
        (tx_time_per_byte_ * 3.0) +
        LATENCY_TIMER;
}

void PortHandler::setPacketTimeoutMillis(double msec)
{
    packet_start_time_ = getCurrentTime(); // en millisecondes
    packet_timeout_ = msec;
}

bool PortHandler::isPacketTimeout()
{
    if (getTimeSinceStart() > packet_timeout_) {
        packet_timeout_ = 0;
        return true;
    }
    return false;
}

double PortHandler::getCurrentTime() const
{
    using namespace std::chrono;

    auto now = steady_clock::now().time_since_epoch();

    // conversion en millisecondes (double)
    return duration<double, std::milli>(now).count();
}

double PortHandler::getTimeSinceStart()
{
    const double now = getCurrentTime();
    double time_since = now - packet_start_time_;

    // Sécurité (utile si l'implémentation change un jour)
    if (time_since < 0.0) {
        packet_start_time_ = now;
        return 0.0;
    }

    return time_since;
}

bool PortHandler::setupPort(int /*cflag_baud*/)
    {
        // if (is_open_) {
        //     closePort();
        // }

        if(!servo.begin(baudrate_, port_name_.c_str())) {
            return false;
        }
        
        is_open_ = true;
        servo.rFlushSCS();

        tx_time_per_byte_ = (1000.0 / static_cast<double>(baudrate_)) * 10.0;

        return true;
    }

int getCFlagBaud(int baudrate)
{
    switch (baudrate) {
        case 4800:
        case 9600:
        case 14400:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
        case 128000:
        case 250000:
        case 500000:
        case 1000000:
            return baudrate;
        default: return -1;
    }
}