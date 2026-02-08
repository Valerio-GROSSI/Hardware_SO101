
#include "port_handler.hpp"
#include <vector>


class GroupSyncRead {
public:
    GroupSyncRead(
        PortHandler ph,
        int start_adress,
        int data_length);

private:
        PortHandler ph;
        int start_adress_;
        int data_length_;
        bool last_result_ = false;
        bool is_param_changed_ = false;
        std::vector<int> param = {};
        std::vector<int> data_dict = {};
        void clearParam();
}
