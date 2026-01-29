#ifndef PORT_HANDLER_HPP
#define PORT_HANDLER_HPP

#include <string>
#include "../lib/FTServo_Linux/include/SCServo.h"

// Constante équivalente au Python
constexpr int DEFAULT_BAUDRATE = 1000000;

// Wrapper pour exposer le flush RX/TX
class SMS_STS_WithFlush : public SMS_STS {
public:
    void resetInputBuffer();
    void resetOutputBuffer();
};

class PortHandler
{
public:
    // Attributs (équivalent Python)
    bool is_open;
    int baudrate;
    double packet_start_time;
    double packet_timeout;
    double tx_time_per_byte;
    bool is_using;
    std::string port_name;

    // Servo Feetech (STS3215)
    SMS_STS_WithFlush servo;

    // Constructeur
    explicit PortHandler(const std::string& port);

    // Méthodes
    void closePort();
    int getCFlagBaud(int baudrate);
    bool setupPort();
    bool setBaudRate(int baudrate);
};

#endif // PORT_HANDLER_HPP