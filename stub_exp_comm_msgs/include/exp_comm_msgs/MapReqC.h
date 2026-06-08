#pragma once
#include <vector>
#include <cstdint>

namespace exp_comm_msgs {
struct MapReqC {
    std::vector<uint16_t> f_id;
    std::vector<uint8_t>  block_id;
    uint8_t               flag = 0;
};
}
