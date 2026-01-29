#include "../lib/FTServo_Linux/include/SCServo.h"
#include <string>

constexpr int DEFAULT_BAUDRATE = 1000000;

class SMS_STS_WithFlush : public SMS_STS {
public:
    void resetInputBuffer() { rFlushSCS(); }
    void resetOutputBuffer() { wFlushSCS(); }
};

class PortHandler
{
public:
    bool is_open;
    int baudrate;
    double packet_start_time;
    double packet_timeout;
    double tx_time_per_byte;
    bool is_using;
    std::string port_name;

    SMS_STS_WithFlush servo;

    explicit PortHandler(const std::string& port): 
    is_open(false),
    baudrate(DEFAULT_BAUDRATE),
    packet_start_time(0.0),
    packet_timeout(0.0),
    tx_time_per_byte(0.0),
    is_using(false),
    port_name(port)
    {}

void closePort()
    {
        servo.end();
        is_open = false;
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

bool setupPort()
    {
        if (is_open) {
            closePort();
        }

        if(!servo.begin(baudrate, port_name.c_str())) {
            return false;
        }
        
        is_open = true;

        servo.resetInputBuffer();

        tx_time_per_byte = (1000.0 / static_cast<double>(baudrate)) * 10.0;

        return true;
    }


bool setBaudRate(int baudrate)
{
    int baud = getCFlagBaud(baudrate);
    
    if (baud <= 0) {
        // setupPort(38400);
        // this->baudrate = baudrate;
        return false;
    }
    else
    {
        this->baudrate = baudrate;
        return setupPort();
    }
}
};