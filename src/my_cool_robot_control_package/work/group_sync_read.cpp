#include "group_sync_read.hpp"
#include "scservo_def.hpp"


GroupSyncRead::GroupSyncRead(PortHandler& ph, int& start_adress, int& data_length):
    ph_(ph),
    start_adress_(start_adress),
    data_length_(data_length) 
    {
        clearParam();
    }

GroupSyncRead::txRxPacket():
    {
        result = txPacket();
        if result != scservo_def_table::COMM_SUCCESS {
            return result;
        }

        result = rxPacket();
    }

CommResult GroupSyncRead::rxPacket()
{
        last_result_ = true;

        int result = scservo_def_table::COMM_RX_FAIL;

        if (data_dict_.empty()) {
            return scservo_def_table::COMM_NOT_AVAILABLE;
        }

        auto [result, rxpacket] = ph_.syncReadRx(data_length_, data_dict_.size());

        if (rxpacket.size() >= (data_length_ + 6)) {
            for (auto& kv : data_dict_) {
                const int scs_id = kv.first;

                auto [value, result] = readRx(rxpacket, scs_id, data_length_);
                kv.second = value;

                if (result != scservo_def_table::COMM_SUCCESS) {
                    last_result_ = false;
                }
            }
        }
        else
        {
            last_result_ = false;
        }

        return result;
    }