#pragma once
#include <vector>
#include <deque>
#include <geometry_msgs/msg/pose.hpp>
#include <exp_comm_msgs/MapC.h>

// Offline single-robot stub — all multi-robot members are no-ops.
class SwarmDataManager {
public:
    bool     is_ground_    = false;
    uint8_t  drone_num_    = 1;
    uint8_t  self_id_      = 1;
    bool     statistic_    = false;
    bool     req_flag_     = false;
    int      finish_num_   = 0;
    double   finish_thresh_= 1.0;

    std::vector<geometry_msgs::msg::Pose> Poses_;
    std::vector<double>                   Pose_t_;
    std::vector<bool>                     finish_list_{false};

    void SetDTGFn(uint16_t /*f_id*/, std::vector<uint8_t>& /*vps*/,
                  uint8_t /*flag*/, bool /*alive*/) {}
    void SetFrontierVpNum(int /*vp_num*/) {}

    struct MapReq {
        std::vector<uint16_t> f_id;
        std::vector<uint8_t>  block_id;
    };
    MapReq mreq_;

    // Used when is_ground_==true (never true in offline mode)
    std::deque<exp_comm_msgs::MapC> swarm_sub_map_;

    template<typename T> void SetMap(T& /*msg*/) {}
    template<typename T> void SetMapReq(T& /*mq*/) {}

    struct CoverageStats { void SetVolume(double /*v*/, int /*i*/) {} };
    CoverageStats CS_;
};
