#pragma once
#include <vector>
#include <cstdint>

namespace exp_comm_msgs {
struct MapC {
    uint16_t             f_id        = 0;
    uint8_t              block_id    = 0;
    uint8_t              block_state = 0;
    std::vector<uint8_t> flags;
};
}
