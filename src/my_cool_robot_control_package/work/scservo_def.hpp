#pragma once

namespace scservo_def_table {

inline constexpr int BROADCAST_ID = 0xFE;  // 254
inline constexpr int MAX_ID = 0xFC;  // 252
inline constexpr int SCS_END = 0;

// Instruction for SCS Protocol
inline constexpr int INST_PING = 1;
inline constexpr int INST_READ = 2;
inline constexpr int INST_WRITE = 3;
inline constexpr int INST_REG_WRITE = 4;
inline constexpr int INST_ACTION = 5;
inline constexpr int INST_SYNC_WRITE = 131;  //0x83
inline constexpr int INST_SYNC_READ = 130;  // 0x82
inline constexpr int INST_RESET = 10;  // 0x0A
inline constexpr int INST_OFSCAL = 11;  // 0x0B

// Communication Result
inline constexpr int COMM_SUCCESS = 0;  // tx or rx packet communication success
inline constexpr int COMM_PORT_BUSY = -1;  // Port is busy (in use)
inline constexpr int COMM_TX_FAIL = -2;  // Failed transmit instruction packet
inline constexpr int COMM_RX_FAIL = -3;  // Failed get status packet
inline constexpr int COMM_TX_ERROR = -4;  // Incorrect instruction packet
inline constexpr int COMM_RX_WAITING = -5;  // Now recieving status packet
inline constexpr int COMM_RX_TIMEOUT = -6;  // There is no status packet
inline constexpr int COMM_RX_CORRUPT = -7;  // Incorrect status packet
inline constexpr int COMM_NOT_AVAILABLE = -9;  //

}