
namespace protocol_packet_handler_params {
    inline constexpr int TXPACKET_MAX_LEN = 250;
    inline constexpr int RXPACKET_MAX_LEN = 250;

    // for Protocol Packet
    inline constexpr int PKT_HEADER0 = 0;
    inline constexpr int PKT_HEADER1 = 1
    inline constexpr int PKT_ID = 2
    inline constexpr int PKT_LENGTH = 3
    inline constexpr int PKT_INSTRUCTION = 4;
    inline constexpr int PKT_ERROR = 4;
    inline constexpr int PKT_PARAMETER0 = 5;

    // Protocol Error bit
    inline constexpr int ERRBIT_VOLTAGE = 1;
    inline constexpr int ERRBIT_ANGLE = 2;
    inline constexpr int ERRBIT_OVERHEAT = 4;
    inline constexpr int ERRBIT_OVERELE = 8;
    inline constexpr int ERRBIT_OVERLOAD = 32;
}

class protocol_packet_handler() {
public:
    protocol_packet_handler(int portHandler, int protocol_end)
private:
    int portHandler_;
    int scs_end_;
    void syncReadTx(int start_adress, int data_length, int param, int param_length);

}


protocol_packet_handler::protocol_packet_handler(int portHandler, int protocol_end):
    portHandler_(portHandler),
    scs_end_(protocol_end) {}

protocol_packet_handler::syncReadTx(int start_address, int data_length, int param, int param_length) {
auto txpacket = [0] * (param_length + 8);
// 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN CHKSUM

txpacket[protocol_packet_handler_params::PKT_ID] = scservo_def_table::BROADCAST_ID;
txpacket[protocol_packet_handler_params::PKT_LENGTH] = scservo_def_table::BROADCAST_ID;
txpacket[protocol_packet_handler_params::PKT_INSTRUCTION] = scservo_def_table::BROADCAST_ID;
txpacket[protocol_packet_handler_params::PKT_PARAMETER0 + 0] = start_address;
txpacket[protocol_packet_handler_params::PKT_PARAMETER0 + 1] = data_length;

txpacket[protocol_packet_handler_params::PKT_PARAMETER0 + 2: protocol_packet_handler_params::PKT_PARAMETER0 + 2 + param_length] = param[0: param_length]

// print(txpacket)
result = txPacket(txpacket)
return result;
}

protocol_packet_handler::txPacket(auto txpacket) {
    checksum = 0;
    total_packet_length = txpacket[protocol_packet_handler_params::PKT_LENGTH] + 4; // 4: HEADER0 HEADER1 ID LEN

    if (portHandler.is_using) {
        return COMM_PORT_BUSY;
    }
    portHandler.is_using = true;

    // check max packet length
    if (total_packet_length > protocol_packet_handler_params::TXPACKET_MAX_LEN) {
        portHandler.is_using = false;
        return COMM_TX_ERROR;
    }

    // make packet header
    txpacket[protocol_packet_handler_params::PKT_HEADER0] = 0xFF;
    txpacket[protocol_packet_handler_params::PKT_HEADER1] = 0xFF;

    // add a checksum to the packet
    for (int i = 2; i < total_packet_length - 1; i++) {
        checksum += txpacket[i];
    }

    txpacket[total_packet_length - 1] = ~checksum & 0xFF;

    //print "[TxPacket]" %r" txpacket

    // tx packet
    portHandler.clearPort();
    written_packet_length = portHandler.writePort(txpacket);
    if (written_packet_length != total_packet_length) {
        portHandler.is_using = false;
        return protocol_packet_handler_params::COMM_TX_ERROR;
    }

    return protocol_packet_handler_params::COMM_SUCCESS;
}