#ifndef MAPPING_STRUCT_H_
#define MAPPING_STRUCT_H_
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <deque>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <list>
#include <bitset>
#include <memory>
#include <math.h>
#include <std_msgs/msg/color_rgba.hpp>
using namespace std;
using namespace Eigen;

namespace BlockMapStruct{

enum GBSTATE{
    UNKNOWN,
    MIXED,
    OCCUPIED,
    FREE
};

enum VoxelState{
    unknown, 
    free, 
    occupied,
    out
};

struct SwarmBlock{
    uint16_t id_;
    Eigen::Vector3d up_, down_;
    vector<double> exploration_rate_;
    vector<double> last_pub_rate_;
    vector<bool> to_pub_;
    uint8_t sub_num_;
};

struct Grid_Block{
    Grid_Block() {state_ = UNKNOWN;};
    ~Grid_Block() {};
    void Awake(float occ, float free){       //if state == UNKNOWN/OCCUPIED/FREE, init odds_log_ of this block 
        
        const int N = block_size_.x() * block_size_.y() * block_size_.z();

        if(state_ == UNKNOWN){
            odds_log_.resize(block_size_.x() * block_size_.y() * block_size_.z(), free - 999.0);
            flags_.resize(odds_log_.size(), 0);

            // === Traversability per-voxel ===
            n_pts_.assign(N, 0);

            mean_.assign(N, Vector3f::Zero());  // mean
            M2_.assign(N, Matrix3f::Zero());    // second-order central moment accumulator

            normal_.assign(N, Vector3f(0,0,1)); // surface normal, defaults to up
            slope_rad_.assign(N, -10.f);
            roughness_.assign(N, -1.f);
            score_.assign(N, -1.f);

            dirty_.assign(N, 0);
            is_surface_.assign(N, 0);


        }
        else if(state_ == OCCUPIED){
            odds_log_.resize(block_size_.x() * block_size_.y() * block_size_.z(), occ);
            flags_.resize(odds_log_.size(), 0);
        }
        else if(state_ == FREE){
            odds_log_.resize(block_size_.x() * block_size_.y() * block_size_.z(), free);
            flags_.resize(odds_log_.size(), 0);
        }
        state_ = MIXED;



    }
    Vector3i origin_;
    Vector3i block_size_;
    unsigned state_;
    vector<float> odds_log_;
    vector<uint8_t> flags_;  //0000 0_(need to be show)_(is ray end occupied flag)_(casted)
    bool show_;
    int free_num_, occ_num_, unk_num_;
    int free_max_num_, occ_max_num_;

    // traversability
    vector<int> n_pts_; //point number
    vector<Vector3f> mean_;      // Welford mean
    vector<Matrix3f> M2_;        // M2 covariance

    vector<Vector3f> normal_;
    vector<float> slope_rad_;    // slope rad
    vector<float> roughness_;    // res -> roughness
    vector<float> score_;        // traversability score

    vector<uint8_t> dirty_;      // 
    vector<int> is_surface_; // surface
    
    int innerPointsNumThr = 5;

};

struct FFD_Grid{
    double far_depth_;
    double close_depth_;
    double max_depth_;

    double dist2depth_;
    bool is_frontier_;
    bool new_iter_;
};
}

#endif