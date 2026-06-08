#include <block_map/block_map.h>

using namespace traversability;

void BlockMap::init(rclcpp::Node::SharedPtr node){

    node_ = node;

    // helper lambda: declare if absent, then read
    auto gp = [&](const std::string &name, auto def) {
        if (!node_->has_parameter(name)) node_->declare_parameter(name, def);
        return node_->get_parameter(name).get_value<decltype(def)>();
    };

    vector<double> CR, CB, CG;
    Eigen::Quaterniond cam2bodyrot;
    Eigen::Vector3d robots_scale;

    cam2body_.setZero();
    int drone_num, self_id;

    CR              = gp("block_map.HeightcolorR",  std::vector<double>{});
    CG              = gp("block_map.HeightcolorG",  std::vector<double>{});
    CB              = gp("block_map.HeightcolorB",  std::vector<double>{});
    origin_.x()     = gp("block_map.minX",          -10.0);
    origin_.y()     = gp("block_map.minY",          -10.0);
    origin_.z()     = gp("block_map.minZ",           0.0);
    map_upbd_.x()   = gp("block_map.maxX",           10.0);
    map_upbd_.y()   = gp("block_map.maxY",           10.0);
    map_upbd_.z()   = gp("block_map.maxZ",           0.0);
    block_size_.x() = gp("block_map.blockX",         5);
    block_size_.y() = gp("block_map.blockY",         5);
    block_size_.z() = gp("block_map.blockZ",         3);
    resolution_     = gp("block_map.resolution",     0.2);
    max_range_      = gp("block_map.sensor_max_range", 4.5);
    depth_          = gp("block_map.depth",          false);
    cam2bodyrot.x() = gp("block_map.CamtoBody_Quater_x", 0.0);
    cam2bodyrot.y() = gp("block_map.CamtoBody_Quater_y", 0.0);
    cam2bodyrot.z() = gp("block_map.CamtoBody_Quater_z", 0.0);
    cam2bodyrot.w() = gp("block_map.CamtoBody_Quater_w", 1.0);
    cam2body_(0,3)  = gp("block_map.CamtoBody_x",   0.0);
    cam2body_(1,3)  = gp("block_map.CamtoBody_y",   0.0);
    cam2body_(2,3)  = gp("block_map.CamtoBody_z",   0.0);
    update_interval_= gp("block_map.update_freq",   5.0);
    show_freq_      = gp("block_map.show_freq",      2.0);
    depth_step_     = gp("block_map.depth_step",     2);
    thr_max_        = gp("block_map.occ_max",        0.9);
    thr_min_        = gp("block_map.occ_min",        0.1);
    pro_hit_        = gp("block_map.pro_hit_occ",    0.7);
    pro_miss_       = gp("block_map.pro_miss_free",  0.8);
    stat_           = gp("block_map.statistic_v",    false);
    min_finish_t_   = gp("block_map.min_finish_t",   30.0);
    stat_upbd_.x()  = gp("Exp.maxX",                -10.0);
    stat_upbd_.y()  = gp("Exp.maxY",                -10.0);
    stat_upbd_.z()  = gp("Exp.maxZ",                 0.0);
    stat_lowbd_.x() = gp("Exp.minX",                 10.0);
    stat_lowbd_.y() = gp("Exp.minY",                 10.0);
    stat_lowbd_.z() = gp("Exp.minZ",                  0.0);
    robots_scale.x()= gp("block_map.RobotVisX",      0.5);
    robots_scale.y()= gp("block_map.RobotVisY",      0.5);
    robots_scale.z()= gp("block_map.RobotVisZ",      0.3);
    swarm_pub_thresh_= gp("block_map.swarm_pub_thresh", 0.95);
    bline_occ_range_= gp("block_map.bline_occ_range", 0.7);
    vis_mode_       = gp("block_map.vis_mode",       false);
    swarm_tol_      = gp("block_map.swarm_tol",      0.2);
    swarm_send_delay_= gp("block_map.swarm_send_delay", 0.2);
    show_block_     = gp("block_map.show_block",     true);
    traversability_compute_ = gp("block_map.traversability_compute", false);
    traversability_vis_     = gp("block_map.traversability_vis",     false);

    // --- NormalSmoothParams ---
    nsp_.enabled    = gp("block_map.normal_smooth_enable",      true);
    nsp_.iterations = gp("block_map.normal_smooth_iterations",  1);
    nsp_.radius_xy  = gp("block_map.normal_smooth_radius_xy",   1);
    nsp_.half_z     = gp("block_map.normal_smooth_half_z",      2);
    nsp_.sigma_s    = (float)gp("block_map.normal_smooth_sigma_s",       0.06);
    nsp_.sigma_ang_deg = (float)gp("block_map.normal_smooth_sigma_ang_deg", 20.0);
    nsp_.sigma_r    = (float)gp("block_map.normal_smooth_sigma_r",       0.02);
    nsp_.n_ref      = gp("block_map.normal_smooth_n_ref",       5);

    // --- TraversabilityScoreParams ---
    tsp_.min_pts_thr  = gp("block_map.trav_min_pts",       5);
    tsp_.slope_max_deg= (float)gp("block_map.trav_slope_max_deg", 60.0);
    tsp_.rough_max    = (float)gp("block_map.trav_rough_max",      0.10);
    tsp_.w_slope      = (float)gp("block_map.trav_w_slope",        0.9);
    tsp_.w_rough      = (float)gp("block_map.trav_w_rough",        0.1);

    start_t_ = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    req_flag_ = false;
    finish_flag_ = false;
    if(SDM_ != nullptr && SDM_->drone_num_ > 1){  //swarm exploration
        use_swarm_ = true;
        drone_num = SDM_->drone_num_;
        swarm_filter_dict_ = new tr1::unordered_map<int, int>;
        swarm_pose_.resize(drone_num);
        cout<<"robots_scale_:"<<robots_scale.transpose()<<endl;
        for(int i = 0; i < drone_num; i++) robots_scale_.emplace_back(robots_scale);
        for(auto &sp : swarm_pose_) sp.first = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - 1000.0;
        self_id_ = SDM_->self_id_;
    }
    else {
        drone_num = 1;
        use_swarm_ = false;
        self_id = 1;
    }
    stat_n_ = 0;

    cout<<pro_hit_<<" "<<pro_miss_<<endl;

    cam2body_.block(0, 0, 3, 3) = cam2bodyrot.matrix();
    // cam2body_.block(0, 3, 3, 1) = cam2bodyrot.matrix();

    cam2body_(3, 3) = 1.0;

    map_upbd_.x() = ceil((map_upbd_.x() - origin_.x())/resolution_) * resolution_;
    map_upbd_.y() = ceil((map_upbd_.y() - origin_.y())/resolution_) * resolution_;
    map_upbd_.z() = ceil((map_upbd_.z() - origin_.z())/resolution_) * resolution_;

    double dx = origin_.x() - (floor((origin_.x())/resolution_)) * resolution_;
    double dy = origin_.y() - (floor((origin_.y())/resolution_)) * resolution_;
    double dz = origin_.z() - (floor((origin_.z())/resolution_)) * resolution_;

    origin_.x() -= dx;
    origin_.y() -= dy;
    origin_.z() -= dz;

    map_upbd_.x() += resolution_;
    map_upbd_.y() += resolution_;
    map_upbd_.z() += resolution_;
    voxel_num_.x() = ceil((map_upbd_.x())/resolution_);
    voxel_num_.y() = ceil((map_upbd_.y())/resolution_);
    voxel_num_.z() = ceil((map_upbd_.z())/resolution_);
    map_upbd_ = origin_ + map_upbd_ - Vector3d(1e-4, 1e-4, 1e-4);
    map_lowbd_ = origin_ + Vector3d(1e-4, 1e-4, 1e-4);

    block_num_.x() = ceil(double(voxel_num_.x()) / block_size_.x());
    block_num_.y() = ceil(double(voxel_num_.y()) / block_size_.y());
    block_num_.z() = ceil(double(voxel_num_.z()) / block_size_.z());
    blockscale_.x() = resolution_*block_size_.x();
    blockscale_.y() = resolution_*block_size_.y();
    blockscale_.z() = resolution_*block_size_.z();

    edgeblock_size_.x() = voxel_num_.x() - floor(voxel_num_.x() / double(block_size_.x()))*block_size_.x();
    edgeblock_size_.y() = voxel_num_.y() - floor(voxel_num_.y() / double(block_size_.y()))*block_size_.y();
    edgeblock_size_.z() = voxel_num_.z() - floor(voxel_num_.z() / double(block_size_.z()))*block_size_.z();

    if(edgeblock_size_.x() == 0) edgeblock_size_.x() = block_size_.x();
    if(edgeblock_size_.y() == 0) edgeblock_size_.y() = block_size_.y();
    if(edgeblock_size_.z() == 0) edgeblock_size_.z() = block_size_.z();
    

    edgeblock_scale_.x() = resolution_*edgeblock_size_.x();
    edgeblock_scale_.y() = resolution_*edgeblock_size_.y();
    edgeblock_scale_.z() = resolution_*edgeblock_size_.z();
    GBS_.resize(block_num_.x()*block_num_.y()*block_num_.z());
    for(int x = 0; x < block_num_.x(); x++){
        for(int y = 0; y < block_num_.y(); y++){
            for(int z = 0; z < block_num_.z(); z++){
                int idx = x + y * block_num_.x() + z * block_num_.x() * block_num_.y();
                GBS_[idx] = make_shared<Grid_Block>();
                GBS_[idx]->origin_.x() = block_size_.x() * x;
                GBS_[idx]->origin_.y() = block_size_.y() * y;
                GBS_[idx]->origin_.z() = block_size_.z() * z;
                GBS_[idx]->show_ = false;
                if(x == block_num_.x() - 1) GBS_[idx]->block_size_.x() = edgeblock_size_.x();
                else GBS_[idx]->block_size_.x() = block_size_.x();
                if(y == block_num_.y() - 1) GBS_[idx]->block_size_.y() = edgeblock_size_.y();
                else GBS_[idx]->block_size_.y() = block_size_.y();
                if(z == block_num_.z() - 1) GBS_[idx]->block_size_.z() = edgeblock_size_.z();
                else GBS_[idx]->block_size_.z() = block_size_.z();
            }
        }
    }

    cout<<"edgeblock_size_:"<<edgeblock_size_.transpose()<<endl;
    cout<<"origin_:"<<origin_.transpose()<<endl;
    cout<<"block_num_:"<<block_num_.transpose()<<endl;
    cout<<"block_num_:"<<block_num_.transpose()<<endl;


    std_msgs::msg::ColorRGBA color;
    colorhsize_ = (map_upbd_(2) - origin_(2)) / (CG.size() - 1);
    for(int i = 0; i < (int)CG.size(); i++){
        color.a = 1.0;
        color.r = CR[i]/255;
        color.g = CG[i]/255;
        color.b = CB[i]/255;
        color_list_.push_back(color);
    }

    // have_odom_ = false;
    have_cam_param_ = false;
    last_odom_ = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() + 100000.0;
    last_update_ = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - 10.0;
    cout<<"last_update_:"<<last_update_<<endl;
    update_interval_ = 1 / update_interval_;
    cout<<"update_interval_:"<<update_interval_<<endl;

    thr_max_ = log(thr_max_ / (1 - thr_max_));
    thr_min_ = log(thr_min_ / (1 - thr_min_));
    pro_hit_ = log(pro_hit_ / (1 - pro_miss_));
    pro_miss_ = log((1 - pro_miss_) / pro_miss_);
    cout<<"thr_max_:"<<thr_max_<<endl;
    cout<<"thr_min_:"<<thr_min_<<endl;
    cout<<"pro_hit_:"<<pro_hit_<<endl;
    cout<<"pro_miss_:"<<pro_miss_<<endl;

    vox_pub_   = node_->create_publisher<visualization_msgs::msg::MarkerArray>("block_map/voxvis", 10);
    debug_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("block_map/debug", 10);

    if(stat_) {
        statistic_pub_   = node_->create_publisher<std_msgs::msg::Float32>("block_map/stat_v", 1);
        statistic_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&BlockMap::StatisticV, this));
    }
    if(depth_){
        camparam_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/block_map/caminfo", 10, std::bind(&BlockMap::CamParamCallback, this, std::placeholders::_1));
    }
    if(show_block_)
        show_timer_ = node_->create_wall_timer(
            std::chrono::duration<double>(1.0 / show_freq_), std::bind(&BlockMap::ShowMapCallback, this));
    if(SDM_ != nullptr) {
        if(SDM_->is_ground_)
            swarm_timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(200), std::bind(&BlockMap::SwarmMapCallback, this));
        else
            swarm_timer_ = node_->create_wall_timer(
                std::chrono::seconds(1), std::bind(&BlockMap::SwarmMapCallback, this));
    }

    
    gravity_dir_unit_ = Eigen::Vector3f(0,0,1);

}

void BlockMap::InitSwarmBlock(vector<uint16_t> &id_l, vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &bound){
    SBS_.resize(id_l.size());
    for(int i = 0; i < id_l.size(); i++){
        SBS_[i].id_ = id_l[i];
        SBS_[i].down_ = bound[i].second + Eigen::Vector3d::Ones() * 1e-3;
        SBS_[i].up_ = bound[i].first - Eigen::Vector3d::Ones() * 1e-3;
        SBS_[i].sub_num_ = 8;
        SBS_[i].exploration_rate_.resize(8, 0);
        SBS_[i].last_pub_rate_.resize(8, 0);
        SBS_[i].to_pub_.resize(8, false);
    }
}

void BlockMap::OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom){

    // geometry_msgs::msg::Point pt;
    // visualization_msgs::msg::Marker debug1, debug2, debug3;
    // debug1.action = visualization_msgs::msg::Marker::ADD;
    // debug1.pose.orientation.w = 1.0;
    // debug1.type = visualization_msgs::msg::Marker::CUBE_LIST;
    // debug1.scale.x = resolution_;
    // debug1.scale.y = resolution_;
    // debug1.scale.z = resolution_;
    // debug1.color.a = 0.3;
    // debug1.color.r = 1.0;
    // debug1.header.frame_id = "world";
    // debug1.header.stamp = node_->now();
    // debug1.id = 0;

    Eigen::Matrix4d body2world = Matrix4d::Identity();
    Eigen::Quaterniond rot;
    rot.x() = odom->pose.pose.orientation.x;
    rot.y() = odom->pose.pose.orientation.y;
    rot.z() = odom->pose.pose.orientation.z;
    rot.w() = odom->pose.pose.orientation.w;

    body2world(0, 3) = odom->pose.pose.position.x;
    body2world(1, 3) = odom->pose.pose.position.y;
    body2world(2, 3) = odom->pose.pose.position.z;

    body2world.block(0, 0, 3, 3) = rot.matrix();

    cam2world_ = body2world * cam2body_;
    AwakeBlocks(cam2world_.block(0, 3, 3, 1), max_range_);
    bline_ = false;
    for(double x = 0; x < 0.51; x += resolution_){
        for(double y = -0.3; y < 0.31; y += resolution_){
            for(double z = -0.3; z < 0.31; z += resolution_){
                Eigen::Vector3d pos = body2world.block(0, 0, 3, 3) * Vector3d(x, y, z) + body2world.block(0, 3, 3, 1);
                // pt.x = pos.x();
                // pt.y = pos.y();
                // pt.z = pos.z();
                // debug1.points.push_back(pt);
                if(GetVoxState(pos) == occupied) {
                    bline_ = true;
                    // RCLCPP_ERROR(rclcpp::get_logger("block_map"), "hl");
                }
            }
        }
    }

    // debug_pub_->publish(debug1);
    // have_odom_ = true;
    last_odom_ = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
}

void BlockMap::InsertPCLCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl){
    if(!bline_ && have_cam_param_ && rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - last_odom_ < 0.02 && rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - last_update_ > update_interval_){
        last_update_ = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();

        InsertPcl(pcl);
        // have_odom_ = false;
    }
}

void BlockMap::InsertDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img){
    if(!bline_ && have_cam_param_ && rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - last_odom_ < 0.02 && rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - last_update_ > update_interval_){
        last_update_ = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();

        InsertImg(img);
        // cout<<"cost:"<<rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - last_update_<<"s"<<endl;
        // have_odom_ = false;
    }
}

void BlockMap::CamParamCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &param){
    RCLCPP_WARN(rclcpp::get_logger("block_map"), "get param!");

    fx_ = param->k[0];
    cx_ = param->k[2];
    fy_ = param->k[4];
    cy_ = param->k[5];

    have_cam_param_ = true;
    u_max_ = param->width;
    v_max_ = param->height;

    downsample_size_ = (int)floor(resolution_ * fx_ / max_range_);
    downsample_size_ = min((int)floor(resolution_ * fy_ / max_range_), downsample_size_);
    

    u_down_max_ = (int)floor((u_max_) / downsample_size_);
    v_down_max_ = (int)floor((v_max_) / downsample_size_);
    u_max_ = u_down_max_ * downsample_size_;
    v_max_ = v_down_max_ * downsample_size_;
    // depth_step_ = min(depth_step_, downsample_size_);
    cout<<"fx_:"<<fx_<<endl;
    cout<<"fy_:"<<fy_<<endl;
    cout<<"cx_:"<<cx_<<endl;
    cout<<"cy_:"<<cy_<<endl;
    cout<<"depth_step_:"<<depth_step_<<endl;
    cout<<"u_down_max_:"<<u_down_max_<<endl;
    cout<<"v_down_max_:"<<v_down_max_<<endl;
    cout<<"downsample_size_:"<<downsample_size_<<endl;
    downsampled_img_.resize(u_down_max_ * v_down_max_);
    for(int u = 0; u < u_down_max_; u++){
        for(int v = 0; v < v_down_max_; v++){
            Eigen::Vector3d pt(((u + 0.5) * downsample_size_ - cx_) * max_range_ / fx_,((v + 0.5) * downsample_size_ - cy_) * max_range_ / fy_, max_range_);
            pt = pt.normalized() * max_range_;
            downsampled_img_[u + v*u_down_max_].max_depth_ = pt.z();
            downsampled_img_[u + v*u_down_max_].close_depth_ = pt.z();

        }
    }
    camparam_sub_.reset();
}


void BlockMap::ShowMapCallback(){
    visualization_msgs::msg::MarkerArray mks;
    visualization_msgs::msg::Marker mk_stand;
    mk_stand.action = visualization_msgs::msg::Marker::ADD;
    mk_stand.pose.orientation.w = 1.0;
    mk_stand.type = visualization_msgs::msg::Marker::POINTS;
    mk_stand.scale.x = resolution_;
    mk_stand.scale.y = resolution_;
    mk_stand.scale.z = resolution_;
    mk_stand.color.a = 1.0;
    mk_stand.header.frame_id = "world";
    mk_stand.header.stamp = node_->now();
    // mk_stand.id = *block_it;


    int i = 0;
    geometry_msgs::msg::Point pt;
    // RCLCPP_ERROR(rclcpp::get_logger("block_map"), "show size:%d", (int)changed_blocks_.size());
    for(vector<int>::iterator block_it = changed_blocks_.begin(); block_it != changed_blocks_.end(); block_it++, i++){
        mks.markers.push_back(mk_stand);
        mks.markers.back().id = *block_it;

        GBS_[*block_it]->show_ = 0;
        Eigen::Vector3d block_end = GBS_[*block_it]->origin_.cast<double>() * resolution_ + resolution_ * GBS_[*block_it]->block_size_.cast<double>()
           + origin_;
        
        double x, y, z;
        int idx = 0;
        int debug_bk, debug_id;
        if(GBS_[*block_it]->state_ == MIXED){//debug
        for( z = resolution_ * (GBS_[*block_it]->origin_(2) + 0.5) + origin_(2); z < block_end(2); z += resolution_){
            for( y = resolution_ * (GBS_[*block_it]->origin_(1)  + 0.5) + origin_(1); y < block_end(1); y += resolution_){
                for( x = resolution_ * (GBS_[*block_it]->origin_(0) + 0.5) + origin_(0); x < block_end(0); x += resolution_){
                    if(GBS_[*block_it]->state_ == OCCUPIED || GBS_[*block_it]->odds_log_[idx] > 0){
                        // cout<<"Awake"<< GBS_[*block_it]->state_<<" !!!!!!!!!!!!!"<<endl;

                        pt.x = x;
                        pt.y = y; 
                        pt.z = z;
                        mks.markers.back().points.push_back(pt);

                        //marker color,Z height
                        // mks.markers.back().colors.push_back(Getcolor(z));
                        // mks.markers.back().colors.push_back(Getcolor(0.1+(GBS_[*block_it]->n_pts_[idx])*0.2));
                        // if(GBS_[*block_it]->n_pts_[idx] < 5 || !GBS_[*block_it]-> is_surface_[idx]){
                            // mks.markers.back().colors.push_back(Getcolor(10));
                        // }
                        // else{
                            // mks.markers.back().colors.push_back(Getcolor(0.1+(GBS_[*block_it]->slope_rad_[idx])/1.5*map_upbd_.z()));
                        // }
                        // cout<<GBS_[*block_it]->n_pts_[idx]<<endl;


                        if(traversability_compute_ && traversability_vis_){
                            mks.markers.back().colors.push_back(Getcolor((1.01-(GBS_[*block_it]->score_[idx])/1.0)*map_upbd_.z()));
                            // cout<<"Score: "<<GBS_[*block_it]->roughness_[idx]<<" "<<GBS_[*block_it]->slope_rad_[idx]<<" "<<GBS_[*block_it]->score_[idx]<<endl;
                            // cout<<"slope_rad_: "<<GBS_[*block_it]->slope_rad_[idx]<<" "<<GBS_[*block_it]->n_pts_[idx]<<endl;

                        }
                        else{
                            mks.markers.back().colors.push_back(Getcolor(z));
                        }
                        if(GBS_[*block_it]->slope_rad_[idx]!=-1){
                            // mks.markers.back().colors.push_back(Getcolor(0.1+abs(GBS_[*block_it]->slope_rad_[idx])*2));

                        }

                    }
                    GetVox(debug_bk, debug_id, Eigen::Vector3d(x, y, z));
                    if(debug_bk != *block_it || debug_id != idx) {
                        cout<<debug_bk<<";"<<debug_id<<"  "<<Eigen::Vector3d(x, y, z).transpose()<<"origin:"<<GBS_[*block_it]->origin_.transpose()
                            <<"  "<<block_end.transpose()<<" .."<<GBS_[*block_it]->block_size_.transpose()<<endl;
                        cout<<*block_it<<" "<<idx<<endl;
                    }
                    idx++;
                }
            }
        }
        }
        if(mks.markers.back().points.size() == 0){
            mks.markers.back().action = visualization_msgs::msg::Marker::DELETE;
        } 
    }
    if(mks.markers.size() > 0) vox_pub_->publish(mks);
    changed_blocks_.clear();
}

void BlockMap::SwarmMapCallback(){
    if(SDM_ == nullptr) return;
    if(SDM_->is_ground_){
        while (!SDM_->swarm_sub_map_.empty()){
            InsertSwarmPts(SDM_->swarm_sub_map_.front());
            SDM_->swarm_sub_map_.pop_front();
        }
        
        if((SDM_->req_flag_ && vis_mode_) || double(SDM_->finish_num_) / SDM_->drone_num_ > SDM_->finish_thresh_ && !finish_flag_ && rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - start_t_ > min_finish_t_){
            finish_flag_ = true;

            exp_comm_msgs::MapReqC mq;
            Eigen::Vector3d up, down;
            list<Eigen::Vector3d> pts;
            for(int f_id = 0; f_id < SBS_.size(); f_id++){
                for(int i = 0; i < 8; i++){
                    GetSBSBound(f_id, i, up, down);//debug

                    if(SBS_[f_id].exploration_rate_[i] < swarm_pub_thresh_){
                        mq.f_id.emplace_back(f_id);
                        mq.block_id.emplace_back(i);
                        pts.push_back((up + down) / 2);
                    }
                }
            }
            for(int i = 0; i < 5; i++){
                cout<<"ground finish!!!===="<<endl;
            }
            Debug(pts);
            mq.flag = 1;
            if(vis_mode_ && SDM_->req_flag_) {
                SDM_->req_flag_ = false;
                mq.flag = 0;
            }
            SDM_->SetMapReq(mq);
        }

        if(SDM_->req_flag_ && !finish_flag_){
            SDM_->req_flag_ = false;
            exp_comm_msgs::MapReqC mq;
            Eigen::Vector3d up, down;
            // list<Eigen::Vector3d> pts;
            for(int f_id = 0; f_id < SBS_.size(); f_id++){
                for(int i = 0; i < 8; i++){
                    GetSBSBound(f_id, i, up, down);//debug
                    // if(vis_mode_){
                    //     if(SBS_[f_id].exploration_rate_[i] < swarm_pub_thresh_){
                    //         mq.f_id.emplace_back(f_id);
                    //         mq.block_id.emplace_back(i);
                    //         // pts.push_back((up + down) / 2);
                    //     }
                    // }
                    // else{
                    if(SBS_[f_id].exploration_rate_[i] < swarm_pub_thresh_ && SBS_[f_id].exploration_rate_[i] > 5e-3){
                        mq.f_id.emplace_back(f_id);
                        mq.block_id.emplace_back(i);
                        // pts.push_back((up + down) / 2);
                    }
                    // }
                }
            }

            // Debug(pts);
            mq.flag = 0; 
            SDM_->SetMapReq(mq);
        }
        if(SDM_->statistic_ && stat_){
            SDM_->CS_.SetVolume(stat_n_ * pow(resolution_, 3), 0);
        }
    }
    else{
        double cur_t = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
        if(SDM_->finish_list_[SDM_->self_id_ - 1] && !finish_flag_ && SDM_->req_flag_){
            finish_flag_ = true;
            list<Eigen::Vector3d> pts;
            // Eigen::Vector3d up, down;
            for(int i = 0; i < SDM_->mreq_.block_id.size(); i++){
                // GetSBSBound(SDM_->mreq_.f_id[i], SDM_->mreq_.block_id[i], up, down);//debug
                // pts.push_back((up + down) / 2);

                if(SBS_[SDM_->mreq_.f_id[i]].to_pub_[SDM_->mreq_.block_id[i]]) continue;
                SBS_[SDM_->mreq_.f_id[i]].to_pub_[SDM_->mreq_.block_id[i]] = true;
                double pub_t = cur_t - swarm_send_delay_ - 0.5;
                swarm_pub_block_.push_back({{SDM_->mreq_.f_id[i], SDM_->mreq_.block_id[i]}, pub_t});
            }
            for(int i = 0; i < 10; i++) RCLCPP_ERROR(rclcpp::get_logger("block_map"), "finishhhhhh");
            // Debug(pts, 5);
            SDM_->mreq_.block_id.clear();
            SDM_->mreq_.f_id.clear();
        }

        if(SDM_->req_flag_ && !SDM_->finish_list_[SDM_->self_id_ - 1] && !finish_flag_){
            SDM_->req_flag_ = false;
            for(int i = 0; i < SDM_->mreq_.block_id.size(); i++){

                if(SBS_[SDM_->mreq_.f_id[i]].to_pub_[SDM_->mreq_.block_id[i]]) continue;
                SBS_[SDM_->mreq_.f_id[i]].to_pub_[SDM_->mreq_.block_id[i]] = true;
                double pub_t = cur_t;
                if(vis_mode_) pub_t -= swarm_send_delay_ + 0.5;
                swarm_pub_block_.push_back({{SDM_->mreq_.f_id[i], SDM_->mreq_.block_id[i]}, pub_t});
            }
        }

            // list<swarm_exp_msgs::DtgHFEdge>::iterator erase_it = hfe_it;
            // hfe_it--;
            // SDM_->swarm_sub_hfe_.erase(erase_it);
        for(auto pub_it = swarm_pub_block_.begin(); pub_it != swarm_pub_block_.end(); pub_it++){
            bool t_o_pub = false;
            exp_comm_msgs::MapC msg;
            if(pub_it->second + swarm_send_delay_ < cur_t) t_o_pub = true; // time out
            Vox2Msg(msg, pub_it->first.first, pub_it->first.second);
            if(((!finish_flag_ ||vis_mode_) && SBS_[msg.f_id].last_pub_rate_[msg.block_id] + 0.15 > SBS_[msg.f_id].exploration_rate_[msg.block_id]
                || finish_flag_ && SBS_[msg.f_id].last_pub_rate_[msg.block_id] + 0.01 > SBS_[msg.f_id].exploration_rate_[msg.block_id])
                 && t_o_pub){
                SBS_[msg.f_id].to_pub_[msg.block_id] = false;
                auto erase_it = pub_it;
                pub_it--;
                swarm_pub_block_.erase(erase_it);
                continue;
            }

            if(msg.block_state != 0 && SBS_[msg.f_id].exploration_rate_[msg.block_id] > swarm_pub_thresh_ || t_o_pub){
                SDM_->SetMap(msg);
                SBS_[msg.f_id].last_pub_rate_[msg.block_id] = SBS_[msg.f_id].exploration_rate_[msg.block_id];
                SBS_[msg.f_id].to_pub_[msg.block_id] = false;
                auto erase_it = pub_it;
                pub_it--;
                swarm_pub_block_.erase(erase_it);
            }

        }

        // while(!swarm_pub_block_.empty()){
        //     if(swarm_pub_block_.front().second + swarm_send_delay_ < cur_t){
        //     exp_comm_msgs::MapC msg;
        //     SBS_[swarm_pub_block_.front().first.first].to_pub_[swarm_pub_block_.front().first.second] = false;
        //     // for(uint8_t i = 0; i < 8; i++){
        //     Vox2Msg(msg, swarm_pub_block_.front().first.first, swarm_pub_block_.front().first.second);
        //     if(msg.block_state != 0){
        //         SDM_->SetMap(msg);
        //     }
        //     // }
        //     swarm_pub_block_.pop_front();
        //     }
        //     else break;
        // }
    }
}
// Offline batch insert: marks every point inside the map bounds as occupied.
// No sensor origin, no max_range clipping, no free-space raycasting.
void BlockMap::OfflineInsertPcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl){
    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pcl, *points, indices);

    newly_register_idx_.clear();
    cur_pcl_.clear();

    // Deduplicate: track which voxels we've already scheduled for update
    vector<int> block_ids, vox_ids;
    int block_id, vox_id;

    for (const auto &pt : *points) {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);

        if (!InsideMap(p)) continue;

        block_id = GetBlockId(p);
        if (block_id == -1) continue;

        // Initial state is UNKNOWN; must Awake() before writing voxels
        if (GBS_[block_id]->state_ != MIXED)
            GBS_[block_id]->Awake((float)thr_max_, (float)thr_min_);

        vox_id = GetVoxId(p, GBS_[block_id]);

        cur_pcl_.emplace_back(p);

        // Update traversability stats for every point (Welford needs all points)
        if (traversability_compute_) {
            Eigen::Vector3i idx3 = PostoId3(p);
            Eigen::Vector3d vox_center = (idx3.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)) * resolution_ + origin_;
            Eigen::Vector3f q_local = (p - vox_center).cast<float>();
            updateVoxelStats(*GBS_[block_id], vox_id, q_local);
        }

        // flags_[vox_id] bit1 = already queued this batch
        if (GBS_[block_id]->flags_[vox_id] & 1) continue;
        GBS_[block_id]->flags_[vox_id] |= 1;
        block_ids.push_back(block_id);
        vox_ids.push_back(vox_id);
    }

    // Apply occupied log-odds update for every unique hit voxel
    for (size_t i = 0; i < block_ids.size(); i++) {
        int bid = block_ids[i], vid = vox_ids[i];
        float odds = GBS_[bid]->odds_log_[vid];
        if (odds < thr_min_ - 1.0f) {
            // First time this voxel is observed
            newly_register_idx_.push_back(Id2LocalPos(GBS_[bid], vid));
            odds = 0.0f;
        }
        GBS_[bid]->odds_log_[vid] = std::min(odds + pro_hit_, thr_max_);
        GBS_[bid]->flags_[vid] = 0;  // reset batch flag
        if (!GBS_[bid]->show_ && show_block_) {
            changed_blocks_.push_back(bid);
            GBS_[bid]->show_ = true;
        }
    }

    // After all points are inserted, run the traversability pipeline on all MIXED blocks
    if (traversability_compute_) {
        Eigen::Vector3f gravity(0.f, 0.f, -1.f);
        for (int bid = 0; bid < (int)GBS_.size(); bid++) {
            if (GBS_[bid]->state_ == MIXED)
                processDirtyVoxels(*GBS_[bid], (float)resolution_, gravity, nsp_, tsp_, 0.f);
        }
    }
}

void BlockMap::CollectTraversability(std::vector<Eigen::Vector3d> &pts,
                                     std::vector<float> &scores) {
    pts.clear();
    scores.clear();
    for (int block_id = 0; block_id < (int)GBS_.size(); block_id++) {
        if (GBS_[block_id]->state_ != MIXED) continue;
        const auto &GB = GBS_[block_id];
        Eigen::Vector3d block_end = GB->origin_.cast<double>() * resolution_
            + resolution_ * GB->block_size_.cast<double>() + origin_;
        int idx = 0; double x, y, z;
        for (z = resolution_*(GB->origin_(2)+0.5)+origin_(2); z < block_end(2); z+=resolution_)
            for (y = resolution_*(GB->origin_(1)+0.5)+origin_(1); y < block_end(1); y+=resolution_)
                for (x = resolution_*(GB->origin_(0)+0.5)+origin_(0); x < block_end(0); x+=resolution_) {
                    if (GB->is_surface_[idx] && GB->score_[idx] >= 0.0f) {
                        pts.emplace_back(x, y, z);
                        scores.push_back(GB->score_[idx]);
                    }
                    idx++;
                }
    }
}

void BlockMap::PatchTraversabilityMap(int fill_radius) {
    // Candidates: occupied voxels with no traversability score that are
    // geometrically surface (have at least one free in-block neighbor).
    // This catches sparse voxels that markSurfaceVoxels skipped due to min_pts.
    static const int DX6[6] = {1,-1, 0, 0, 0, 0};
    static const int DY6[6] = {0, 0, 1,-1, 0, 0};
    static const int DZ6[6] = {0, 0, 0, 0, 1,-1};

    std::vector<Eigen::Vector3d> candidates;
    for (int bid = 0; bid < (int)GBS_.size(); bid++) {
        if (GBS_[bid]->state_ != MIXED) continue;
        const auto& GB = GBS_[bid];
        const int Sx = GB->block_size_.x();
        const int Sy = GB->block_size_.y();
        const int Sz = GB->block_size_.z();
        auto IDX = [&](int x, int y, int z){ return z*(Sx*Sy) + y*Sx + x; };

        for (int z = 0; z < Sz; ++z)
        for (int y = 0; y < Sy; ++y)
        for (int x = 0; x < Sx; ++x) {
            int vid = IDX(x, y, z);
            if (GB->odds_log_[vid] <= 0.0f) continue;  // not occupied
            if (GB->score_[vid]    >= 0.0f) continue;  // already has score

            // Geometric surface check (within block, 6-connectivity)
            bool geo_surface = false;
            for (int k = 0; k < 6; ++k) {
                int xx = x+DX6[k], yy = y+DY6[k], zz = z+DZ6[k];
                if (xx<0||yy<0||zz<0||xx>=Sx||yy>=Sy||zz>=Sz) continue;
                if (GB->odds_log_[IDX(xx,yy,zz)] <= 0.0f) { geo_surface = true; break; }
            }
            if (geo_surface)
                candidates.push_back(Id2LocalPos(GB, vid));
        }
    }

    if (candidates.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("block_map"), "[PatchTrav] no candidates, skipping");
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("block_map"), "[PatchTrav] %zu candidates", candidates.size());

    int total_filled = 0;
    while (!candidates.empty()) {
        std::vector<Eigen::Vector3d> remaining;
        for (const auto& pos : candidates) {
            float sum = 0.0f;
            int   cnt = 0;
            for (int dz = -fill_radius; dz <= fill_radius; dz++)
            for (int dy = -fill_radius; dy <= fill_radius; dy++)
            for (int dx = -fill_radius; dx <= fill_radius; dx++) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                Eigen::Vector3d npos = pos + Eigen::Vector3d(dx, dy, dz) * resolution_;
                int nbid = GetBlockId(npos);
                if (nbid < 0 || GBS_[nbid]->state_ != MIXED) continue;
                int nvid = GetVoxId(npos, GBS_[nbid]);
                if (GBS_[nbid]->score_[nvid] >= 0.0f) {
                    sum += GBS_[nbid]->score_[nvid];
                    cnt++;
                }
            }
            if (cnt > 0) {
                int bid = GetBlockId(pos);
                int vid = GetVoxId(pos, GBS_[bid]);
                GBS_[bid]->score_[vid]      = sum / cnt;
                GBS_[bid]->is_surface_[vid] = 1;   // expose to CollectTraversability
                total_filled++;
            } else {
                remaining.push_back(pos);
            }
        }
        if (remaining.size() == candidates.size()) break;  // no progress, stop
        candidates = std::move(remaining);
    }

    RCLCPP_INFO(rclcpp::get_logger("block_map"),
        "[PatchTrav] filled %d voxels, %zu isolated (unreachable)", total_filled, candidates.size());
}

void BlockMap::OfflineFillUnknownFree() {
    int filled_blk = 0, filled_vox = 0;
    for (int bid = 0; bid < (int)GBS_.size(); bid++) {
        if (GBS_[bid]->state_ == GBSTATE::UNKNOWN) {
            GBS_[bid]->state_ = GBSTATE::FREE;   // no allocation needed
            filled_blk++;
        } else if (GBS_[bid]->state_ == GBSTATE::MIXED) {
            for (auto &odds : GBS_[bid]->odds_log_) {
                if (odds < thr_min_ - 1.0f) {
                    odds = thr_min_;
                    filled_vox++;
                }
            }
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("block_map"),
        "OfflineFillUnknownFree: %d unknown blocks -> FREE, %d unobserved voxels -> free",
        filled_blk, filled_vox);
}

void BlockMap::CollectVoxels(std::vector<Eigen::Vector3d> &occ_pts,
                             std::vector<std_msgs::msg::ColorRGBA> &occ_colors,
                             std::vector<Eigen::Vector3d> &free_pts) {
    occ_pts.clear();
    occ_colors.clear();
    free_pts.clear();

    for (int block_id = 0; block_id < (int)GBS_.size(); block_id++) {
        if (GBS_[block_id]->state_ != MIXED) continue;

        const auto &GB = GBS_[block_id];
        Eigen::Vector3d block_end =
            GB->origin_.cast<double>() * resolution_
            + resolution_ * GB->block_size_.cast<double>()
            + origin_;

        int idx = 0;
        double x, y, z;
        for (z = resolution_ * (GB->origin_(2) + 0.5) + origin_(2); z < block_end(2); z += resolution_) {
            for (y = resolution_ * (GB->origin_(1) + 0.5) + origin_(1); y < block_end(1); y += resolution_) {
                for (x = resolution_ * (GB->origin_(0) + 0.5) + origin_(0); x < block_end(0); x += resolution_) {
                    float odds = GB->odds_log_[idx];
                    if (odds > 0.0f) {
                        occ_pts.emplace_back(x, y, z);
                        occ_colors.push_back(Getcolor(z));
                    } else if (odds < 0.0f && odds >= thr_min_) {
                        free_pts.emplace_back(x, y, z);
                    }
                    idx++;
                }
            }
        }
    }
}

//TR:  rosPointcloud2 -> PCL -> voxel occupency
//TR: Integrates an incoming PointCloud2 message into the occupancy grid by raycasting from the sensor origin,
//TR: updating voxel log-odds for hits and misses, applying swarm-based filtering, and tracking newly observed voxels.
void BlockMap::InsertPcl(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl){
    vector<int> block_ids, vox_ids;
    vector<int>::iterator block_it, vox_it;

    Eigen::Vector3i cam3i, end3i;
    Eigen::Vector3d end_point, dir, cam, ray_iter;
    Eigen::Vector3d half_res = Eigen::Vector3d(0.5, 0.5, 0.5) * resolution_;
    RayCaster rc;
    int block_id, vox_id;

    newly_register_idx_.clear();
    cam3i = PostoId3(cam2world_.block(0,3,3,1));
    if(InsideMap(cam3i)){
        LoadSwarmFilter();
        cam = cam2world_.block(0,3,3,1);
        
        std::vector<int> indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pcl, *points);
        pcl::removeNaNFromPointCloud(*points, *points, indices);
        cur_pcl_.clear();

        for(pcl::PointCloud<pcl::PointXYZ>::const_iterator pcl_it = points->begin(); pcl_it != points->end(); pcl_it++){
            bool occ;
            end_point(0) = pcl_it->x;
            end_point(1) = pcl_it->y;
            end_point(2) = pcl_it->z;
            // end_point = cam2world_.block(0, 0, 3, 3) * end_point + cam2world_.block(0, 3, 3, 1);

            dir = end_point - cam;
            end3i = PostoId3(end_point);            
            occ = dir.norm() <= max_range_;
            if(!occ)
                end_point = cam + (end_point - cam).normalized() * max_range_;

            GetRayEndInsideMap(cam, end_point, occ);
            if(!GetVox(block_id, vox_id, end_point)) continue;
            if(occ){
                if(use_swarm_ && swarm_filter_dict_->find(PostoId(end_point)) != swarm_filter_dict_->end()) continue;
                cur_pcl_.emplace_back(end_point);
                if(!(GBS_[block_id]->flags_[vox_id] & 2)){

                    GBS_[block_id]->flags_[vox_id] |= 2;

                    if(GBS_[block_id]->flags_[vox_id] & 1){
                    }
                    else{
                        GBS_[block_id]->flags_[vox_id] |= 1;
                        block_ids.push_back(block_id);
                        vox_ids.push_back(vox_id);
                    }
                }
                else{
                    continue;
                }
            }
            else{
                if((GBS_[block_id]->flags_[vox_id] & 1)){
                    continue;
                }
            }
            rc.setInput((end_point - origin_) / resolution_, (cam - origin_) / resolution_);
            
            while (rc.step(ray_iter))
            {
                ray_iter = (ray_iter) * resolution_ + origin_ + half_res;
                if(use_swarm_ && swarm_filter_dict_->find(PostoId(ray_iter)) != swarm_filter_dict_->end()) continue;
                if(GetVox(block_id, vox_id, ray_iter)){

                    if((GBS_[block_id]->flags_[vox_id] & 1)){
                        continue;
                    }
                    else{
                        GBS_[block_id]->flags_[vox_id] |= 1;
                        block_ids.push_back(block_id);
                        vox_ids.push_back(vox_id);
                    }
                }
            }
        }
        Eigen::Vector3d p_it;
        for(block_it = block_ids.begin(), vox_it = vox_ids.begin(); block_it != block_ids.end(); block_it++, vox_it++){
            float odds_origin = GBS_[*block_it]->odds_log_[*vox_it];
            if(GBS_[*block_it]->odds_log_[*vox_it] < thr_min_ - 1.0){
                p_it = Id2LocalPos(GBS_[*block_it], *vox_it);
                if(stat_){
                    if(p_it(0) > stat_lowbd_(0) && p_it(1) > stat_lowbd_(1) && p_it(2) > stat_lowbd_(2) &&
                    p_it(0) < stat_upbd_(0) && p_it(1) < stat_upbd_(1) && p_it(2) < stat_upbd_(2))
                    stat_n_++;
                }
                newly_register_idx_.push_back(p_it);
                odds_origin = 0.0;
            }
            if(GBS_[*block_it]->flags_[*vox_it] & 2){
                GBS_[*block_it]->odds_log_[*vox_it] = min(odds_origin + pro_hit_, thr_max_);
            }
            else{
                // end_point = Id2LocalPos(GBS_[*block_it], *vox_it);
                p_it = Id2LocalPos(GBS_[*block_it], *vox_it);
                if((p_it - cam).norm() > bline_occ_range_ || GBS_[*block_it]->odds_log_[*vox_it] < 0)
                    GBS_[*block_it]->odds_log_[*vox_it] = max(odds_origin + pro_miss_, thr_min_);
            }
            GBS_[*block_it]->flags_[*vox_it] = 0;
            if(!GBS_[*block_it]->show_ && show_block_){
                changed_blocks_.push_back(*block_it);
                GBS_[*block_it]->show_ = true;
            }
        }
    }
}

// TR:  Depth image -> downsample with hash-> occupency voxel
void BlockMap::InsertImg(const sensor_msgs::msg::Image::ConstSharedPtr &depth){
    if(!have_cam_param_) return;
    vector<int> block_ids, vox_ids;
    vector<int>::iterator block_it, vox_it;
    tr1::unordered_map<int,int> trav_dirty_set; // block_id → 1, for dedup
    
    Eigen::Vector3i cam3i;
    RayCaster rc;

    cam3i = PostoId3(cam2world_.block(0,3,3,1));
    newly_register_idx_.clear();

    if(InsideMap(cam3i)){
        LoadSwarmFilter();
        double pix_depth;
        int downsamp_u, downsamp_v, downsamp_nex_u, downsamp_nex_v, downsamp_idx;
        int block_id, vox_id;
        Eigen::Vector3i  end3i;
        Eigen::Vector3d end_point, dir, cam, ray_iter;
        Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
        Eigen::Vector3d half_res = Eigen::Vector3d(0.5, 0.5, 0.5) * resolution_;

        cur_pcl_.clear();
        cam = cam2world_.block(0, 3, 3, 1);

        uint16_t* row_ptr;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth, depth->encoding);
        if(depth->encoding != sensor_msgs::image_encodings::TYPE_16UC1){
            (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, 1000.0);
        }
        tr1::unordered_map<int, Eigen::Vector3d> cast_pts;
        int k_l = ceil(max_range_ * 10.0 / resolution_); 
        for(int v = 0; v < v_max_; v += depth_step_){
            row_ptr = cv_ptr->image.ptr<uint16_t>(v);
            for(int u = 0; u < u_max_; u += depth_step_, row_ptr += depth_step_){

                pix_depth = (*row_ptr) / 1000.0;
                if(pix_depth == 0 && bline_) continue;

                if(pix_depth == 0)
                    pix_depth = max_range_ + 0.1;

                downsamp_u = floor(u / downsample_size_);
                downsamp_v = floor(v / downsample_size_);
                downsamp_idx = downsamp_u + downsamp_v * u_down_max_;

                end_point.x() = ((u + 0.5)  - cx_) * pix_depth / fx_;
                end_point.y() =  ((v + 0.5)  - cy_) * pix_depth / fy_;
                end_point.z() = pix_depth;
                end_point = cam2world_.block(0, 0, 3, 3) * end_point + cam;
                int key = floor((end_point(2) - cam(2) - max_range_ * 2)/resolution_)*k_l*k_l + 
                    floor((end_point(1)- cam(1) - max_range_ * 2)/resolution_)*k_l + 
                    floor((end_point(0)- cam(0) - max_range_ * 2)/resolution_);

                if(cast_pts.find(key) == cast_pts.end())
                    cast_pts.insert({key, end_point});

            }
        }

        // for(int u = 0; u < u_down_max_; u++){
        //     for(int v = 0; v < v_down_max_; v++){
        //         pix_depth = downsampled_img_[u + v * u_down_max_].close_depth_;
        for(auto &k_p : cast_pts){
                bool occ;
                end_point = k_p.second;
                occ = (k_p.second - cam).norm() < max_range_;
                if(!occ)
                    end_point = cam + (end_point - cam).normalized() * max_range_;
                else{
                    if(!use_swarm_ || (swarm_filter_dict_->find(PostoId(end_point)) == swarm_filter_dict_->end()))
                        cur_pcl_.emplace_back(end_point);
                }

                Vector3d end_point_true = end_point;
                end_point = (PostoId3(end_point).cast<double>() + half) * resolution_ + origin_;
                Vector3d center_d = end_point; // correct voxel center cordinate

                GetRayEndInsideMap(cam, end_point, occ);

                if(!GetVox(block_id, vox_id, end_point)) continue;

                Vector3d q_local_d = (end_point_true - center_d);
                Vector3f q_local = q_local_d.cast<float>(); 

                if(occ){
                    if(use_swarm_ && swarm_filter_dict_->find(PostoId(end_point)) != swarm_filter_dict_->end()) continue;

                    // cout<<end_point_true<<"\n % "<<end_point<<"\n / "<< center_d<< "}"<<endl;
                    updateVoxelStats(*GBS_[block_id], vox_id, q_local);
                    // GBS_[block_id]->n_pts_[vox_id]++;
                    
                    if(!(GBS_[block_id]->flags_[vox_id] & 2)){
                    // if(!(GBS_[block_id]->flags_[vox_id] & 2) || GBS_[block_id]->n_pts_[vox_id]<GBS_[block_id]->innerPointsNumThr){

                        GBS_[block_id]->flags_[vox_id] |= 2;

                        if(GBS_[block_id]->flags_[vox_id] & 1){
                        }
                        else{
                            GBS_[block_id]->flags_[vox_id] |= 1;
                            block_ids.push_back(block_id);
                            vox_ids.push_back(vox_id);
                        }
                    }
                    else{
                        continue;
                    }
                }


                rc.setInput((end_point - origin_) / resolution_, (cam - origin_) / resolution_);
                while (rc.step(ray_iter))
                {
                    ray_iter = (ray_iter) * resolution_ + origin_ + half_res;
                    if(use_swarm_ && swarm_filter_dict_->find(PostoId(ray_iter)) != swarm_filter_dict_->end()) continue;
                    if(GetVox(block_id, vox_id, ray_iter)){

                        if((GBS_[block_id]->flags_[vox_id] & 1)){
                            continue;
                        }
                        else{
                            GBS_[block_id]->flags_[vox_id] |= 1;
                            block_ids.push_back(block_id);
                            vox_ids.push_back(vox_id);
                        }
                    }
                }
        }

        Eigen::Vector3d p_it;
        for(block_it = block_ids.begin(), vox_it = vox_ids.begin(); block_it != block_ids.end(); block_it++, vox_it++){
            float odds_origin = GBS_[*block_it]->odds_log_[*vox_it];
            if(GBS_[*block_it]->odds_log_[*vox_it] < thr_min_ - 1.0){
                p_it = Id2LocalPos(GBS_[*block_it], *vox_it);
                // if(abs(p_it(2) - 6.5) < 0.01){
                //     int x = *vox_it % GBS_[*block_it]->block_size_(0);
                //     int y = ((*vox_it - x)/GBS_[*block_it]->block_size_(0)) % GBS_[*block_it]->block_size_(1);
                //     int z = ((*vox_it - x) - y*GBS_[*block_it]->block_size_(0))/GBS_[*block_it]->block_size_(1)/GBS_[*block_it]->block_size_(0);
                //     cout<<"x:"<<x<<" y:"<<y<<" z:"<<z<<endl;
                //     cout<<"p_it:"<<p_it.transpose()<<"origin:"<<origin_.transpose()<<endl;
                //     RCLCPP_ERROR(rclcpp::get_logger("block_map"), "error------");
                //     rclcpp::shutdown();
                //     return;
                // }
                if(stat_){
                    if(p_it(0) > stat_lowbd_(0) && p_it(1) > stat_lowbd_(1) && p_it(2) > stat_lowbd_(2) &&
                    p_it(0) < stat_upbd_(0) && p_it(1) < stat_upbd_(1) && p_it(2) < stat_upbd_(2))
                    stat_n_++;
                }
                newly_register_idx_.push_back(p_it);
                odds_origin = 0.0;
            }
            if(GBS_[*block_it]->flags_[*vox_it] & 2){
                GBS_[*block_it]->odds_log_[*vox_it] = min(odds_origin + pro_hit_, thr_max_);

                if(traversability_compute_){
                    trav_dirty_set[*block_it] = 1; // mark; actual processing after loop
                }
            }
            else{
                // end_point = Id2LocalPos(GBS_[*block_it], *vox_it);
                GBS_[*block_it]->odds_log_[*vox_it] = max(odds_origin + pro_miss_, thr_min_);
            }
            GBS_[*block_it]->flags_[*vox_it] = 0;
            if(!GBS_[*block_it]->show_ && show_block_){
                changed_blocks_.push_back(*block_it);
                GBS_[*block_it]->show_ = true;
            }
            // Process each dirty block exactly once per frame

        }
        if(traversability_compute_){
            for(auto& kv : trav_dirty_set)
                processDirtyVoxels(*GBS_[kv.first], resolution_, gravity_dir_unit_, nsp_, tsp_, 0);
        }
    }
    // Debug();
}

void BlockMap::ProjectToImg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl, vector<double> &depth_img){
    Eigen::Vector2i uv;
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*pcl, *points);
    pcl::removeNaNFromPointCloud(*points, *points, indices);

    depth_img = void_img_;
    
    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator pcl_it = points->begin(); pcl_it != points->end(); pcl_it++){
        SpointToUV(pcl_it->x / pcl_it->z, pcl_it->y / pcl_it->z, uv);
        if(uv(0) >= 0 && uv(0) < u_max_ && uv(1) >= 0 && uv(1) < v_max_){
            int idx = uv(0) + uv(1) * u_max_;
            depth_img[idx] = min(depth_img[idx], double(pcl_it->z));
        }
    }
}

void BlockMap::AwakeBlocks(const Eigen::Vector3d &center, const double &range){
    Eigen::Vector3d upbd, lowbd;
    Eigen::Vector3i upbd_id3, lowbd_id3; 
    int idx;
    upbd.x() = min(range + center(0) + resolution_, map_upbd_.x());
    upbd.y() = min(range + center(1) + resolution_, map_upbd_.y());
    upbd.z() = min(range + center(2) + resolution_, map_upbd_.z());

    lowbd.x() = max(center(0) - range - resolution_, map_lowbd_.x());
    lowbd.y() = max(center(1) - range - resolution_, map_lowbd_.y());
    lowbd.z() = max(center(2) - range - resolution_, map_lowbd_.z());

    upbd_id3.x() = floor((upbd.x() - origin_(0)) / blockscale_.x());
    upbd_id3.y() = floor((upbd.y() - origin_(1)) / blockscale_.y());
    upbd_id3.z() = floor((upbd.z() - origin_(2)) / blockscale_.z());
    lowbd_id3.x() = floor((lowbd.x() - origin_(0)) / blockscale_.x());
    lowbd_id3.y() = floor((lowbd.y() - origin_(1)) / blockscale_.y());
    lowbd_id3.z() = floor((lowbd.z() - origin_(2)) / blockscale_.z());

    for(int x = lowbd_id3.x(); x <= upbd_id3.x(); x++){
        for(int y = lowbd_id3.y(); y <= upbd_id3.y(); y++){
            for(int z = lowbd_id3.z(); z <= upbd_id3.z(); z++){
                idx = x + y * block_num_(0) + z * block_num_(0) * block_num_(1);
                GBS_[idx]->Awake((float)thr_max_, (float)thr_min_);
                GBS_[idx]->innerPointsNumThr = tsp_.min_pts_thr;
            }
        }
    }
}

//TR: Merge a remote block update: 
// wake blocks, decode flags→voxels, insert, and refresh exploration stats.

void BlockMap::InsertSwarmPts(exp_comm_msgs::MapC &msg){
    // debug_l_<<"f:"<<int(msg.f_id)<<"  b:"<<int(msg.block_id)<<"  "<<int(msg.block_state)<<endl;
    if(msg.f_id < 0 || msg.f_id >= SBS_.size()) return;
    if(msg.block_id < 0 || msg.block_id >= 8) return;
    if(msg.block_state == 0) return;
    Eigen::Vector3d up, down;
    if(!GetSBSBound(msg.f_id, msg.block_id, up, down)){
        RCLCPP_ERROR(rclcpp::get_logger("block_map"), "error InsertSwarmPts GetSBSBound");
        return;
    }

    Eigen::Vector3d center = (up + down) / 2;
    double range = 0;
    for(int dim = 0; dim < 3; dim++) range = max(range, up(dim) - down(dim) + 0.5);
    AwakeBlocks(center, range);

    vector<Eigen::Vector3d> pts;
    vector<uint8_t> states;

    Flags2Vox(msg.flags, up, down, msg.block_state, pts, states);
    // if(pts.size() > 0)cout<<"success!"<<endl;
    InseartVox(pts, states);
    UpdateSBS(msg.f_id, msg.block_id);
    // debug_l_<<"f:"<<int(msg.f_id)<<"  b:"<<int(msg.block_id)<<"  rate:"<<SBS_[msg.f_id].exploration_rate_[msg.block_id]<<"  "<<finish_flag_<<endl;
}

//TR: Publish 8 sub-blocks of SBS[f_id] now or enqueue for deferred, deduped by to_pub_.
void BlockMap::SendSwarmBlockMap(const int &f_id, const bool &send_now){
    if(f_id < 0 || f_id >= SBS_.size()) return;
    if(send_now){
        exp_comm_msgs::MapC msg;
        for(uint8_t i = 0; i < 8; i++){
            SBS_[f_id].to_pub_[i] = false;
            Vox2Msg(msg, f_id, i);
            if(msg.block_state != 0){
                SDM_->SetMap(msg);
            }
        }
    }
    else{
        for(uint8_t i = 0; i < 8; i++){
            if(!SBS_[f_id].to_pub_[i]){
                swarm_pub_block_.push_back({{f_id, i}, rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds()});
                SBS_[f_id].to_pub_[i] = true;
            }
        }
    }
}

//TR: bonding box check
bool BlockMap::PosBBXOccupied(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx){
    Eigen::Vector3d lowbd, upbd, v_it;
    VoxelState state;
    lowbd = pos - bbx / 2;
    upbd = pos + bbx / 2 + Eigen::Vector3d::Ones() * (resolution_ - 1e-3);
    for(v_it(0) = lowbd(0); v_it(0) < upbd(0); v_it(0) += resolution_){
        for(v_it(1) = lowbd(1); v_it(1) < upbd(1); v_it(1) += resolution_){
            for(v_it(2) = lowbd(2); v_it(2) < upbd(2); v_it(2) += resolution_){
                state = GetVoxState(v_it);
                if(state == VoxelState::occupied) return true;
            }
        }
    }
    return false;
}

bool BlockMap::PosBBXFree(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx){
    Eigen::Vector3d lowbd, upbd, v_it;
    VoxelState state;
    lowbd = pos - bbx / 2;
    upbd = pos + bbx / 2 + Eigen::Vector3d::Ones() * (resolution_ - 1e-3);
    for(v_it(0) = lowbd(0); v_it(0) < upbd(0); v_it(0) += resolution_){
        for(v_it(1) = lowbd(1); v_it(1) < upbd(1); v_it(1) += resolution_){
            for(v_it(2) = lowbd(2); v_it(2) < upbd(2); v_it(2) += resolution_){
                state = GetVoxState(v_it);
                if(state != VoxelState::free) return false;
            }
        }
    }
    return true;
}

void BlockMap::StatisticV(){
    std_msgs::msg::Float32 msg;
    msg.data = stat_n_ * pow(resolution_, 3);
    statistic_pub_->publish(msg);
}

void BlockMap::Debug(list<Eigen::Vector3d> &pts, int ddd){
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = node_->now();
    mk.id = 2 + ddd;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    mk.pose.position.x = SDM_->self_id_ * 0.1 - 0.5;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    if(SDM_->is_ground_)
        mk.color.r = 1.0;
    else{
        if(ddd == 1){
            mk.color.g = 1.0;
        }
        else if(ddd == 2){
            mk.color.g = 1.0;
            mk.color.r = 0.5;
        }
        else{
            mk.color.b = 1.0;
            mk.pose.position.z = 0.1;
        }
    }
    mk.color.a = 0.5;
    geometry_msgs::msg::Point pt;
    if(!finish_flag_) mk.lifetime = rclcpp::Duration::from_seconds(1.0);
    RCLCPP_WARN(rclcpp::get_logger("block_map"), "id:%d Debug", SDM_->self_id_);
    for(auto &p : pts){
        // Eigen::Vector3d p = IdtoPos(p_id.first);
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        mk.points.emplace_back(pt);
    }
    debug_pub_->publish(mk);
}