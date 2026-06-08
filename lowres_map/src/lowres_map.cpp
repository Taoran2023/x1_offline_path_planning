#include <lowres_map/lowres_map.h>
#include <algorithm>
using namespace std;
using namespace Eigen;

namespace lowres{

void LowResMap::init(rclcpp::Node::SharedPtr node){
    node_ = node;

    auto gp = [&](const std::string &name, auto def) {
        if (!node_->has_parameter(name)) node_->declare_parameter(name, def);
        return node_->get_parameter(name).get_value<decltype(def)>();
    };

    Eigen::Vector3d localgraph_scale;
    localgraph_scale.x()  = gp("LowResMap.localgraph_sizex", 14.0);
    localgraph_scale.y()  = gp("LowResMap.localgraph_sizey", 14.0);
    localgraph_scale.z()  = gp("LowResMap.localgraph_sizez",  8.0);
    node_scale_.x()       = gp("LowResMap.node_x",   0.5);
    node_scale_.y()       = gp("LowResMap.node_y",   0.5);
    node_scale_.z()       = gp("LowResMap.node_z",   0.3);
    Robot_size_.x()       = gp("Exp.robot_sizeX",    0.5);
    Robot_size_.y()       = gp("Exp.robot_sizeY",    0.5);
    Robot_size_.z()       = gp("Exp.robot_sizeZ",    0.3);
    lambda_heu_           = gp("LowResMap.lambda_heu", 1.5);
    origin_.x()           = gp("Exp.minX",          -10.0);
    origin_.y()           = gp("Exp.minY",          -10.0);
    origin_.z()           = gp("Exp.minZ",            0.0);
    corridor_exp_r_.x()   = gp("LowResMap.corridor_expX", 1);
    corridor_exp_r_.y()   = gp("LowResMap.corridor_expY", 1);
    corridor_exp_r_.z()   = gp("LowResMap.corridor_expZ", 1);
    mapscale_.x()         = gp("Exp.maxX",           10.0);
    mapscale_.y()         = gp("Exp.maxY",           10.0);
    mapscale_.z()         = gp("Exp.maxZ",            0.0);
    block_size_.x()       = gp("LowResMap.blockX",   5);
    block_size_.y()       = gp("LowResMap.blockY",   5);
    block_size_.z()       = gp("LowResMap.blockZ",   3);
    resolution_           = gp("LowResMap.resolution", 0.2);
    showmap_              = gp("LowResMap.showmap",  false);
    debug_                = gp("LowResMap.debug",    false);
    seg_length_           = gp("LowResMap.seg_length", 3.0);
    prune_seg_length_     = gp("LowResMap.prune_seg_length", 3.0);
    show_dtg_             = gp("LowResMap.show_dtg", false);
    occ_duration_         = gp("block_map.occ_duration",   1.0);
    unknown_duration_     = gp("block_map.unknown_duration", 0.5);
    drone_num_            = gp("Exp.drone_num", 1);

    //pub
    if(showmap_){
        node_pub_   = node_->create_publisher<visualization_msgs::msg::MarkerArray>("LowResMap/Nodes", 10);
        show_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(150), std::bind(&LowResMap::ShowGridLocal, this));
    }
    if(debug_){
        debug_pub_        = node_->create_publisher<visualization_msgs::msg::Marker>("LowResMap/debug", 10);
        bimodel_debug_pub_= node_->create_publisher<visualization_msgs::msg::Marker>("LowResMap/bimodel_debug", 10);
    }

    // unit: node_scale in meter
    // defalt: node_size = (0.5/0.2)+1 = 3.5 -> 4
    node_size_(0) = ceil(node_scale_(0) / resolution_) + 1;
    node_size_(1) = ceil(node_scale_(1) / resolution_) + 1;
    node_size_(2) = ceil(node_scale_(2) / resolution_) + 1;

    // defalt: node_size = 3*0.2 = 0.8
    node_scale_ = node_size_.cast<double>() * resolution_;
    expand_r_ = Robot_size_ * 0.5;

    // defalt: mapscale = (10-(-10))/0.6*0.6
    mapscale_.x() = ceil((mapscale_.x() - origin_.x())/node_scale_(0)) * node_scale_(0);
    mapscale_.y() = ceil((mapscale_.y() - origin_.y())/node_scale_(1)) * node_scale_(1);
    mapscale_.z() = ceil((mapscale_.z() - origin_.z())/node_scale_(2)) * node_scale_(2);

    double dx = origin_.x() - (floor((origin_.x())/node_scale_(0))) * node_scale_(0);
    double dy = origin_.y() - (floor((origin_.y())/node_scale_(1))) * node_scale_(1);
    double dz = origin_.z() - (floor((origin_.z())/node_scale_(2))) * node_scale_(2);

    origin_.x() -= dx;
    origin_.y() -= dy;
    origin_.z() -= dz;
    // mapscale_.x() += node_scale_(0);
    // mapscale_.y() += node_scale_(1);
    // mapscale_.z() += node_scale_(2);
    voxel_num_.x() = ceil((mapscale_.x()-1e-3)/node_scale_(0));
    voxel_num_.y() = ceil((mapscale_.y()-1e-3)/node_scale_(1));
    voxel_num_.z() = ceil((mapscale_.z()-1e-3)/node_scale_(2));
    v_n_.x() = voxel_num_.x();
    v_n_.y() = voxel_num_.y() * voxel_num_.x();
    v_n_.z() = voxel_num_.z() * voxel_num_.y() * voxel_num_.x();

    map_upbd_ = origin_+mapscale_ - Eigen::Vector3d(1e-4, 1e-4, 1e-4);
    map_lowbd_ = origin_ + Eigen::Vector3d(1e-4, 1e-4, 1e-4);



    block_num_.x() = ceil(double(voxel_num_.x()) / block_size_.x());
    block_num_.y() = ceil(double(voxel_num_.y()) / block_size_.y());
    block_num_.z() = ceil(double(voxel_num_.z()) / block_size_.z());
    b_n_.x() = block_num_.x();
    b_n_.y() = block_num_.y() * block_num_.x();
    b_n_.z() = block_num_.z() * block_num_.y() * block_num_.x();
    blockscale_.x() = node_scale_(0)*block_size_.x();
    blockscale_.y() = node_scale_(1)*block_size_.y();
    blockscale_.z() = node_scale_(2)*block_size_.z();

    edgeblock_size_.x() = voxel_num_.x() - floor(voxel_num_.x() / double(block_size_.x()))*block_size_.x();
    edgeblock_size_.y() = voxel_num_.y() - floor(voxel_num_.y() / double(block_size_.y()))*block_size_.y();
    edgeblock_size_.z() = voxel_num_.z() - floor(voxel_num_.z() / double(block_size_.z()))*block_size_.z();

    if(edgeblock_size_.x() == 0) edgeblock_size_.x() = block_size_.x();
    if(edgeblock_size_.y() == 0) edgeblock_size_.y() = block_size_.y();
    if(edgeblock_size_.z() == 0) edgeblock_size_.z() = block_size_.z();

    edgeblock_scale_.x() = node_scale_(0)*edgeblock_size_.x();
    edgeblock_scale_.y() = node_scale_(1)*edgeblock_size_.y();
    edgeblock_scale_.z() = node_scale_(2)*edgeblock_size_.z();
    gridBLK_.resize(block_num_.x()*block_num_.y()*block_num_.z());


    localgraph_size_(0) = ceil(localgraph_scale(0)/node_scale_(0));
    localgraph_size_(1) = ceil(localgraph_scale(1)/node_scale_(1));
    localgraph_size_(2) = ceil(localgraph_scale(2)/node_scale_(2));
    
    Inc_list_.resize(4);
    open_set_.resize(4);
    for(int i = 0; i < 4; i++) {
        Astar_worktable_.push_back(true);
    }
    
    Eternal_bid_ = -1;
    Eternal_nid_ = -1;
    workable_ = true;
    Outnode_ = make_shared<LR_node>();
    Expandnode_ = make_shared<LR_node>();
    cur_root_h_id_ = 0;
    // caster_.setParams()
    cout<<"origin_:"<<origin_.transpose()<<endl;
    cout<<"map_upbd_:"<<map_upbd_.transpose()<<endl;
    cout<<"map_lowbd_:"<<map_lowbd_.transpose()<<endl;
    cout<<"mapscale_:"<<mapscale_.transpose()<<endl;
    cout<<"blockscale_:"<<blockscale_.transpose()<<endl;
    cout<<"edgeblock_scale_:"<<edgeblock_scale_.transpose()<<endl;
    cout<<"block_size_:"<<block_size_.transpose()<<endl;
    cout<<"voxel_num_:"<<voxel_num_.transpose()<<endl;
    cout<<"block_num_:"<<block_num_.transpose()<<endl;
    cout<<"node_size_:"<<node_size_.transpose()<<endl;
    cout<<"node_scale_:"<<node_scale_.transpose()<<endl;
    cout<<"localgraph_scale:"<<localgraph_scale.transpose()<<endl;
    cout<<"drone_num:"<<drone_num_<<endl;
    cout<<"localgraph_size_:"<<localgraph_size_.transpose()<<endl;
}

void LowResMap::UpdateLocalBBX(const Eigen::Matrix4d rob_pose, vector<Eigen::Vector3d> &occ_list){
    mtx_.lock();
    Robot_pose_ = rob_pose;
    auto start = std::chrono::steady_clock::now();

    Robot_pos_ = Robot_pose_.block(0,3,3,1);
    Eigen::Vector3i robot_pid;
    PostoId3(Robot_pos_, robot_pid);
    for(int i = 0; i < 3; i++){
        local_up_idx_(i) = min(voxel_num_(i)-1, robot_pid(i)+int(ceil(localgraph_size_(i)/2)));
        local_low_idx_(i) = max(0, robot_pid(i)-int(ceil(localgraph_size_(i)/2)));
    }

    ClearInfeasible(occ_list);
    ExpandLocalMap();
    PruneBlock();

    vector<Eigen::Vector4d> pts;
    // Debug(pts);
    mtx_.unlock();
    if(showmap_){
        thread t1(&LowResMap::LoadShowList, this);
        t1.detach();
    }
}

bool LowResMap::UpdateLocalTopo(const Eigen::Matrix4d rob_pose, vector<Eigen::Vector3d> &occ_list, bool clear_x){
    mtx_.lock();
    Robot_pose_ = rob_pose;
    auto start = std::chrono::steady_clock::now();
    Robot_pos_ = Robot_pose_.block(0,3,3,1);
    Eigen::Vector3i robot_pid;
    PostoId3(Robot_pos_, robot_pid);
    for(int i = 0; i < 3; i++){
        local_up_idx_(i) = min(voxel_num_(i)-1, robot_pid(i));
        local_low_idx_(i) = max(0, robot_pid(i));
    }
    if(clear_x) ClearXNodes();
    UpdateFOV();
    ClearInfeasibleTopo(occ_list);
    ExpandTopoMap();
    PruneTopoBlock();
    mtx_.unlock();
    if(showmap_){
        thread t1(&LowResMap::LoadShowList, this);
        t1.detach();
    }
    // cout<<"2 cur_root_h_id_:"<<cur_root_h_id_<<"  prep_idx_:"<<prep_idx_<<endl;
    // cout<<(prep_idx_ == cur_root_h_id_)<<endl;
    return prep_idx_ == cur_root_h_id_;
}
 
void LowResMap::ExpandLocalMap(){
    for(std::vector<int>::iterator lastit = localnode_list_.begin(); lastit != localnode_list_.end(); lastit++){
        shared_ptr<LR_node> node = GetNode(*lastit);
        if(node != NULL){
            node->flags_.reset(1);
            node->flags_.reset(2);
        }
    }
    H_Topolist_.clear();
    Topolist_.clear();
    localnode_list_.clear();
    Robot_pos_ = Robot_pose_.block(0,3,3,1);

    Rootnode_ = GetNode(Robot_pos_);
    int nodeid = PostoId(Robot_pos_);
    std::list<int> TSList, NTSList;
    if(Rootnode_ == NULL){
        NTSList.push_back(nodeid);
        std::cout <<Robot_pos_.transpose()<<"\033[0;34m New root \033[0m" << std::endl;
        SetEXPNode(Robot_pos_);
    }
    else if(Rootnode_ == Outnode_){
        std::cout <<Robot_pos_.transpose()<<"\033[0;31m Outside!!!!! \033[0m" << std::endl;
        return;
    }
    else if(Rootnode_->flags_[0]){
        std::cout <<Robot_pos_.transpose()<<"\033[0;31m collide!!!!! \033[0m" << std::endl;
        return;
    }
    else{
        TSList.push_back(nodeid);
    }
    Eigen::Vector3i node3i;
    Eigen::Vector3d pos, offset;
    offset = Eigen::Vector3d(0.5,0.5,0.5);
    double time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    
    while((!NTSList.empty() || !TSList.empty())){
        while(!NTSList.empty()){
            node3i(0) = NTSList.front() % v_n_(0);
            node3i(1) = ((NTSList.front() - node3i(0))/v_n_(0)) % voxel_num_(1);
            node3i(2) = ((NTSList.front() - node3i(0)) - node3i(1)*voxel_num_(0))/v_n_(1);
            // cout<<"node3i:"<<node3i.transpose()<<";;;"<<NTSList.front()<<endl;
            pos = (node3i.cast<double>() + offset).cwiseProduct(node_scale_)+origin_;
            int node_status = CheckNode(node3i);
            if(node_status == 2){
                SetXNode(pos, time + unknown_duration_);
            }
            else if(node_status == 1){
                SetXNode(pos, time + occ_duration_);
            }
            else{
                GetExpandNeighbours(NTSList.front(), node3i, NTSList, TSList);
                SetNode(pos);
                localnode_list_.push_back(NTSList.front());    
            }
            NTSList.pop_front();
        }
        while(!TSList.empty()){
            GetExistNeighbours(TSList.front(), TSList, NTSList);
            TSList.pop_front();
        }
    }   
}

void LowResMap::ExpandTopoMap(){
    for(auto &lastit : localnode_list_){
        shared_ptr<LR_node> node = GetNode(lastit);
        if(node != NULL){
            node->topo_sch_ = NULL;
            node->flags_.reset(1);
        }
    }
    localnode_list_.clear();

    cur_root_h_id_ = 0;
    cur_h_cost_ = 999999;
    Topolist_.clear();
    H_Topolist_.clear();
    id_idx_dist_.clear();
    id_Hpos_dist_.clear();
    Eigen::Vector3i robot_pid;
    PostoId3(Robot_pos_, robot_pid);

    Rootnode_ = GetNode(Robot_pos_);
    shared_ptr<LR_node> lr_node;
    prio_D empty_TS, empty_NTS; 
    shared_ptr<sch_node> c_node, ep_node;

    open_TS_.swap(empty_TS);
    open_NTS_.swap(empty_NTS);
    c_node = make_shared<sch_node>();
    c_node->pos_ = robot_pid;

    if(Rootnode_ == NULL){
        open_NTS_.push(c_node);
        std::cout <<Robot_pos_.transpose()<<"\033[0;34m New root \033[0m" << std::endl;
        SetTopoNode(Robot_pos_, Rootnode_, c_node);
    }
    else if(Rootnode_ == Outnode_){
        std::cout <<Robot_pos_.transpose()<<"\033[0;31m Outside!!!!! \033[0m" << std::endl;
        return;
    }
    else if(Rootnode_->flags_[0]){
        std::cout <<Robot_pos_.transpose()<<"\033[0;31m collide!!!!! \033[0m" << std::endl;
        return;
    }
    else{
        Rootnode_->topo_sch_ = c_node;
        open_TS_.push(c_node);
    }
    Eigen::Vector3i node3i;
    Eigen::Vector3d pos, offset;
    offset = Eigen::Vector3d(0.5, 0.5, 0.5);
    double time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    
    while((!open_NTS_.empty() || !open_TS_.empty())){
        while(!open_NTS_.empty()){
            ep_node = open_NTS_.top();
            open_NTS_.pop();
            if(ep_node->status_ == in_close) continue;
            ep_node->status_ = in_close;
            pos = (ep_node->pos_.cast<double>() + offset).cwiseProduct(node_scale_)+origin_;
            PostoId3(pos, node3i);
            int node_status = CheckNode(node3i);
            if(node_status == 2){
                SetUNode(pos, time + unknown_duration_);
            }
            else if(node_status == 1){
                SetXNode(pos, time + occ_duration_);
            }
            else {
                int id = ep_node->pos_(0) + ep_node->pos_(1) * v_n_(0) + ep_node->pos_(2) * v_n_(1);
                GetTopoNeighbours(ep_node);
                SetNode(pos, true);
                localnode_list_.push_back(id);
                for(int i = 0; i < 3; i++){
                    local_up_idx_(i) = max(ep_node->pos_(i), local_up_idx_(i));
                    local_low_idx_(i) = min(ep_node->pos_(i), local_low_idx_(i));
                }
            }
        }

        while(!open_TS_.empty()){
            ep_node = open_TS_.top();
            open_TS_.pop();
            if(ep_node->status_ == in_close) continue;
            ep_node->status_ = in_close;
            int id = ep_node->pos_(0) + ep_node->pos_(1) * v_n_(0) + ep_node->pos_(2) * v_n_(1);
            pos = (ep_node->pos_.cast<double>() + offset).cwiseProduct(node_scale_)+origin_;
            GetTopoNeighbours(ep_node);
            SetNode(pos, true);
            localnode_list_.push_back(id);
            for(int i = 0; i < 3; i++){
                local_up_idx_(i) = max(ep_node->pos_(i), local_up_idx_(i));
                local_low_idx_(i) = min(ep_node->pos_(i), local_low_idx_(i));
            }
        }
    }   
    local_up_bd_ = IdtoPos(local_up_idx_) + node_scale_ * 0.499;
    local_low_bd_ = IdtoPos(local_low_idx_) - node_scale_ * 0.499;
}


void LowResMap::SetEternalNode(const Eigen::Vector3d &pos){

    Eigen::Vector3i blockid;
    if(GetBlock3Id(pos, blockid)){
        Eternal_bid_ = blockid(2)*b_n_(1) + blockid(1)*block_num_(0) + blockid(0);
        if(gridBLK_[Eternal_bid_] != NULL){
            Eternal_nid_ = GetNodeId(pos, gridBLK_[Eternal_bid_]);
        }
        else{
            Eternal_bid_ = -1;
            Eternal_nid_ = -1;
        }
    }
    else{
        Eternal_bid_ = -1;
        Eternal_nid_ = -1;
    }
}

void LowResMap::ClearInfeasible(vector<Eigen::Vector3d> &occ_list){
    idx_tie_clear_.clear();
    double time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    h_id_clear_.clear();
    for(vector<Eigen::Vector3d>::iterator ocit = occ_list.begin(); ocit != occ_list.end(); ocit++){
        SetXNode(*ocit, time + occ_duration_);
    }
}

void LowResMap::ClearInfeasibleTopo(vector<Eigen::Vector3d> &occ_list){
    list<pair<Eigen::Vector3i, lr_root>> idx_tie_clear_temp_;
    shared_ptr<LR_node> cn;
    Eigen::Vector3i idx;
    double length;
    u_char dir;
    idx_tie_clear_.clear();
    double time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    h_id_clear_.clear();
    for(vector<Eigen::Vector3d>::iterator ocit = occ_list.begin(); ocit != occ_list.end(); ocit++){
        SetXNode(*ocit, time + occ_duration_);
    }
    idx_tie_clear_temp_ = idx_tie_clear_;
    //clear the outdirs of the root to the cleared nodes
    for(auto &it : idx_tie_clear_){
        idx = it.first;
        if(TopoDirIter(it.second.in_dir_, idx, length)){
            int node_id = idx(2)*v_n_(1) + idx(1)*voxel_num_(0) + idx(0);
            cn = GetNode(node_id);
            for(auto &tie : cn->ties_){
                if(tie.root_id_ == it.second.root_id_){
                    dir = ~InverseDir(it.second.in_dir_);
                    tie.out_dir_ &= dir;
                    break;
                }
            }
        }
    }

    vector<Eigen::Vector3i> neis;
    //clear the leaves of the cleared nodes
    for(auto &it : idx_tie_clear_temp_){
        MultiTopoDirIter(it.second.out_dir_, it.first, neis);
        int c_id = it.first(2)*v_n_(1) + it.first(1)*voxel_num_(0) + it.first(0);
        cn = GetNode(c_id);
        for(int dim = 0; dim < 3; dim++){
                local_up_idx_(dim) = max(it.first(dim), local_up_idx_(dim));
                local_low_idx_(dim) = min(it.first(dim), local_low_idx_(dim));
        }

        for(list<lr_root>::iterator tie_it = cn->ties_.begin(); tie_it != cn->ties_.end(); tie_it++){
            if(tie_it->root_id_ == it.second.root_id_){
                cn->ties_.erase(tie_it);
                break;
            }
        }
        for(auto &nei: neis){
            int n_id = nei(2)*v_n_(1) + nei(1)*voxel_num_(0) + nei(0);
            cn = GetNode(n_id);
            for(auto &tie : cn->ties_){
                if(tie.root_id_ == it.second.root_id_){
                    idx_tie_clear_temp_.push_back({nei, tie});
                    break;
                }
            }
        }
    }

    local_up_bd_ = IdtoPos(local_up_idx_);
    local_low_bd_ = IdtoPos(local_low_idx_);
}

void LowResMap::PruneBlock(){
    //clear Xnodes
    double time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    for(list<pair<int, int>>::iterator x_it = Xlist_.begin(); x_it != Xlist_.end(); x_it++){
        if(gridBLK_[x_it->first] != NULL){
            if(gridBLK_[x_it->first]->local_grid_[x_it->second] != NULL){
                if(gridBLK_[x_it->first]->local_grid_[x_it->second]->flags_[0]){
                    if(time_now - gridBLK_[x_it->first]->local_grid_[x_it->second]->last_update_ > 0.0){
                        gridBLK_[x_it->first]->alive_num_--;
                        gridBLK_[x_it->first]->local_grid_[x_it->second] = NULL;
                        if(gridBLK_[x_it->first]->alive_num_ == 0) DeadBlockList_.push_back(x_it->first);
                        list<pair<int, int>>::iterator erase_it = x_it;
                        x_it--;
                        Xlist_.erase(erase_it);
                    }
                }
                else{//debug
                    RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "Error PruneBlock xnode");
                }
            }
        }
        else{
            list<pair<int, int>>::iterator erase_it = x_it;
            x_it--;
            Xlist_.erase(erase_it);
        }
    }
    //clear dead block
    for(vector<int>::iterator deadit = DeadBlockList_.begin(); deadit != DeadBlockList_.end(); deadit++){
        if(gridBLK_[*deadit] != NULL){
            if(gridBLK_[*deadit]->alive_num_ == 0){
                gridBLK_[*deadit] = NULL;
            }
            else{
                std::cout << "\033[0;35m Error PruneBlock1(), alive_num_ != 0! \033[0m"<<*deadit<< std::endl;
            }
        }
    }
    DeadBlockList_.clear();
}

void LowResMap::PruneTopoBlock(){
    //clear Xnodes
    double time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();

    for(list<pair<int, int>>::iterator x_it = Xlist_.begin(); x_it != Xlist_.end(); x_it++){
        if(gridBLK_[x_it->first] != NULL){
            if(gridBLK_[x_it->first]->local_grid_[x_it->second] != NULL){
                //delete sch_node
                gridBLK_[x_it->first]->local_grid_[x_it->second]->topo_sch_ = NULL;
                if(gridBLK_[x_it->first]->local_grid_[x_it->second]->flags_[0]){
                    //delete expired xnode
                    if(time_now - gridBLK_[x_it->first]->local_grid_[x_it->second]->last_update_ > 0.0){
                        gridBLK_[x_it->first]->alive_num_--;
                        gridBLK_[x_it->first]->local_grid_[x_it->second] = NULL;
                        if(gridBLK_[x_it->first]->alive_num_ == 0) DeadBlockList_.push_back(x_it->first);
                        list<pair<int, int>>::iterator erase_it = x_it;
                        x_it--;
                        Xlist_.erase(erase_it);
                    }
                }
                else{//debug, error happens, doing
                    RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "Error PruneBlock xnode");
                    rclcpp::shutdown();
                }
            }
            else{
                list<pair<int, int>>::iterator erase_it = x_it;
                x_it--;
                Xlist_.erase(erase_it);
            }
        }
        else{
            list<pair<int, int>>::iterator erase_it = x_it;
            x_it--;
            Xlist_.erase(erase_it);
        }
    }

    //get paths to frontier viewpoints
    frontier_path_.clear();
    // cout<<"frontier_list_:"<<frontier_list_.size()<<endl;
    for(auto &i_v : frontier_list_){
        shared_ptr<LR_node> n;
        shared_ptr<sch_node> best_vp;
        Eigen::Vector3d debug_pos;
        int best_v_id;
        double length = 99999.0;
        //search viewpoint with the shortest path length to the robot
        // RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "PruneTopoBlock0");
        for(auto &vp : i_v.second){
            n = GetNode(vp.second);
            if(n != NULL  && n->topo_sch_ != NULL && n->topo_sch_->g_score_ < length){
                length = n->topo_sch_->g_score_;
                best_vp = n->topo_sch_;
                best_v_id = vp.first;
                debug_pos = vp.second;
            }
        }
        // RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "PruneTopoBlock1");
        if(length < 99998.0){
            list<Eigen::Vector3d> path;
            RetrievePath(path, best_vp);
            reverse(path.begin(), path.end());
            path.emplace_back(IdtoPos(best_vp->pos_));
            if(!IsFeasible(debug_pos)) RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "error debug pos");
            frontier_path_.push_back({{i_v.first, length}, {best_v_id ,path}});
        }
        // RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "PruneTopoBlock2");
    }
    frontier_list_.clear();

    //clear dead block
    for(vector<int>::iterator deadit = DeadBlockList_.begin(); deadit != DeadBlockList_.end(); deadit++){
        if(gridBLK_[*deadit] != NULL){
            if(gridBLK_[*deadit]->alive_num_ == 0){
                gridBLK_[*deadit] = NULL;
            }
            else{//debug
                std::cout << "\033[0;35m Error PruneBlock2(), alive_num_ != 0! \033[0m"<<*deadit<<"  "<<int(gridBLK_[*deadit]->alive_num_)<< std::endl;
                int al = 0;
                for(auto &n : gridBLK_[*deadit]->local_grid_){
                    if(n != NULL) {
                        cout<<n->flags_<<endl;
                        al++;
                    }
                }
                cout<<al<<endl;
                rclcpp::shutdown();
            }
        }
    }
    DeadBlockList_.clear();

    // cout<<"cur_root_h_id_:"<<cur_root_h_id_<<"  prep_idx_:"<<prep_idx_<<endl;
    if(cur_root_h_id_ != 0) 
    {   
        DjkstraLocal(cur_root_h_idx_); 
        //debug djkstra
        for(auto &topo : H_Topolist_){
            UpdateTie(topo.first, topo.second, cur_root_h_id_, false);
        }
    }
    else{
        cur_root_h_id_ = prep_idx_;
        //update topological relationship
        for(auto &topo : Topolist_){
            UpdateTie(topo.first, topo.second, prep_idx_, true);
        }
    }

    for(auto &lastit : localnode_list_){
        shared_ptr<LR_node> node = GetNode(lastit);
        if(node != NULL){
            node->topo_sch_ = NULL;
            node->flags_.reset(2);
        }
    }

}  

void LowResMap::ClearXNodes(){
    double time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();

    for(list<pair<int, int>>::iterator x_it = Xlist_.begin(); x_it != Xlist_.end(); x_it++){
        if(gridBLK_[x_it->first] != NULL){
            if(gridBLK_[x_it->first]->local_grid_[x_it->second] != NULL){
                //delete sch_node
                gridBLK_[x_it->first]->local_grid_[x_it->second]->topo_sch_ = NULL;
                if(gridBLK_[x_it->first]->local_grid_[x_it->second]->flags_[0]){
                    gridBLK_[x_it->first]->alive_num_--;
                    gridBLK_[x_it->first]->local_grid_[x_it->second] = NULL;
                    if(gridBLK_[x_it->first]->alive_num_ == 0) DeadBlockList_.push_back(x_it->first);
                    list<pair<int, int>>::iterator erase_it = x_it;
                    x_it--;
                    Xlist_.erase(erase_it);
                }
                else{//debug, error happens, doing
                    RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "Error PruneBlock xnode");
                    rclcpp::shutdown();
                }
            }
            else{
                list<pair<int, int>>::iterator erase_it = x_it;
                x_it--;
                Xlist_.erase(erase_it);
            }
        }
        else{
            list<pair<int, int>>::iterator erase_it = x_it;
            x_it--;
            Xlist_.erase(erase_it);
        }
    }

    //clear dead block
    for(vector<int>::iterator deadit = DeadBlockList_.begin(); deadit != DeadBlockList_.end(); deadit++){
        if(gridBLK_[*deadit] != NULL){
            if(gridBLK_[*deadit]->alive_num_ == 0){
                gridBLK_[*deadit] = NULL;
            }
            else{//debug
                std::cout << "\033[0;35m Error PruneBlock2(), alive_num_ != 0! \033[0m"<<*deadit<<"  "<<int(gridBLK_[*deadit]->alive_num_)<< std::endl;
                int al = 0;
                for(auto &n : gridBLK_[*deadit]->local_grid_){
                    if(n != NULL) {
                        cout<<n->flags_<<endl;
                        al++;
                    }
                }
                cout<<al<<endl;
                rclcpp::shutdown();
            }
        }
    }
    DeadBlockList_.clear();
}

bool LowResMap::PathCheck(list<Eigen::Vector3d> &path, bool allow_uknown){
    RayCaster rc;
    Eigen::Vector3d inv_res, ray_iter;
    Eigen::Vector3d half_res = 0.5 * node_scale_;
    list<Eigen::Vector3d>::iterator ps_it, pe_it;
    for(int dim = 0; dim < 3; dim++) inv_res(dim) = 1.0 / node_scale_(dim);

    if(path.size() <= 1) {
        // RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "strange path! %ld", path.size());
        return false;
    }

    ps_it = path.begin();
    pe_it = path.begin();
    pe_it++;
    while(pe_it != path.end()){
        rc.setInput((*pe_it - origin_).cwiseProduct(inv_res), (*ps_it - origin_).cwiseProduct(inv_res));
        while(rc.step(ray_iter)){
            ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
            if(!IsFeasible(ray_iter, allow_uknown)) {
                return false;
            }
        }
        ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
        if(!IsFeasible(ray_iter, allow_uknown)) {
            return false;
        }
        ps_it++;
        pe_it++;
    }
    return true;
}

bool LowResMap::FindCorridors(const vector<Eigen::Vector3d> path, 
                vector<Eigen::MatrixX4d> &corridors, 
                vector<Eigen::Matrix3Xd> &corridorVs,
                vector<Eigen::Vector3d> &pruned_path,
                double prune_length){
    vector<Eigen::Vector3d> path_cast;
    Eigen::Vector3i cor_size, cor_sidx, cor_eidx;
    Eigen::Vector3d cor_start, cor_end;
    double cur_total_length = 0, cur_length = 0;
    bool newseg_flag = true;
    int end_idx = 0, start_idx = 0;
    pruned_path.clear();
    corridors.clear();
    corridorVs.clear(); 
    if(!path.empty()){
        path_cast.emplace_back(path.front());
        // path_cast.emplace_back(GetStdPos(path.front()));
    }
    RayCaster rc;

    for(auto pt : path){
        if(!InsideMap(pt)){
            RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "out!!!");
            return false;
        }
        int last_id = PostoId(path_cast.back());
        if(PostoId(pt) != last_id){
            Eigen::Vector3d rc_s, rc_e, it;
            rc_s = path_cast.back() - origin_;
            rc_e = pt - origin_;
            for(int dim = 0; dim < 3; dim ++){
                rc_s(dim) /= node_scale_(dim);
                rc_e(dim) /= node_scale_(dim);
            }

            rc.setInput(rc_s, rc_e);
            while (rc.step(it)) {
                it = it.cwiseProduct(node_scale_) + origin_ + node_scale_/2;
                int id = PostoId(it);
                if(id == last_id) continue;
                path_cast.emplace_back(it);
            }
            it = it.cwiseProduct(node_scale_) + origin_ + node_scale_/2;
            int id = PostoId(it);
            if(id != last_id) path_cast.emplace_back(it);
        }
    }
    path_cast.emplace_back(path.back());

    pruned_path.emplace_back(path[0]);
    // cout<<"pruned_path"<<pruned_path.size()<<"  :"<<pruned_path.back().transpose()<<endl;

    while(1){
        if(newseg_flag){
            cor_size.setOnes();
            cur_length = 0;
            start_idx = end_idx;
            cor_start = path_cast[start_idx];
            cor_end = cor_start;
            // pruned_path.emplace_back(cor_start);
            // cout<<"pruned_path_n"<<pruned_path.size()<<"  :"<<pruned_path.back().transpose()<<endl;
            PostoId3(cor_start, cor_sidx);
            cor_eidx = cor_sidx;
            newseg_flag = false;
        }

        end_idx++;
        cur_length += (path_cast[end_idx] - path_cast[end_idx - 1]).norm();
        cur_total_length += (path_cast[end_idx] - path_cast[end_idx - 1]).norm();

        bool success = ExpandPath(cor_sidx, cor_eidx, path_cast[end_idx]);


        if(!success || cur_length >= seg_length_ || end_idx + 1 >= path_cast.size() || start_idx == 0){
            
            if(end_idx == start_idx && cur_length < seg_length_){
                RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "error path");
                return false;
            }

            Eigen::Vector3d up_corner, down_corner;
            CoarseExpand(cor_sidx, cor_eidx);
            FineExpand(cor_sidx, cor_eidx, up_corner, down_corner);
            Eigen::MatrixX4d h(6, 4);
            Eigen::Matrix3Xd p(3, 8);
            h.setZero();
            p.setZero();
            for(int dim = 0; dim < 3; dim++){
                up_corner(dim) -= Robot_size_(dim) * 0.52;
                down_corner(dim) += Robot_size_(dim) * 0.52;
                if(up_corner(dim) <= down_corner(dim)) {
                    RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "narrow!!!");
                    return false;
                }
                h(dim*2, dim) = 1;
                h(dim*2, 3) = -up_corner(dim);
                h(dim*2 + 1, dim) = -1;
                h(dim*2 + 1, 3) = down_corner(dim);
            }

            for(int dim1 = 0; dim1 <= 1; dim1++){
                for(int dim2 = 0; dim2 <= 1; dim2++){
                    for(int dim3 = 0; dim3 <= 1; dim3++){
                        p(0, 4*dim3 + 2*dim2 + dim1) = dim1 ? down_corner(0) : up_corner(0);
                        p(1, 4*dim3 + 2*dim2 + dim1) = dim2 ? down_corner(1) : up_corner(1);
                        p(2, 4*dim3 + 2*dim2 + dim1) = dim3 ? down_corner(2) : up_corner(2);
                    }
                }
            }


            if(!success) end_idx--;
            corridors.emplace_back(h);
            corridorVs.emplace_back(p);
            pruned_path.emplace_back(path_cast[end_idx]);
            // cout<<"pruned_path_f"<<pruned_path.size()<<"  :"<<pruned_path.back().transpose()<<endl;
            newseg_flag = true;
            if(end_idx + 1 >= path_cast.size() || cur_total_length >= prune_length){
                break;
            }
        }
    }
    return true;
}

bool LowResMap::ExpandPath(Eigen::Vector3i &cor_start, Eigen::Vector3i &cor_end, const Eigen::Vector3d &pos){
    Eigen::Vector3i corp_idx, d_corsize, cor_start_temp, cor_end_temp, cor_it;
    cor_start_temp = cor_start;
    cor_end_temp = cor_end;
    PostoId3(pos, corp_idx);
    for(int i = 0; i < 3; i++){
        d_corsize = cor_end_temp - cor_start_temp + Eigen::Vector3i(1, 1, 1);
        cor_it = cor_start_temp;
        
        if(corp_idx(i) < cor_start_temp(i)){
            d_corsize(i) = cor_start_temp(i) - corp_idx(i);
            cor_it(i) = corp_idx(i);
            cor_start_temp(i) = corp_idx(i);
        }
        else if(corp_idx(i) > cor_end_temp(i)){
            d_corsize(i) = corp_idx(i) - cor_end_temp(i);
            cor_it(i) = corp_idx(i);
            cor_end_temp(i) = corp_idx(i);
        } 
        else{
            continue;
        }


        for(int x = 0; x < d_corsize(0); x++){
            for(int y = 0; y < d_corsize(1); y++){
                for(int z = 0; z < d_corsize(2); z++){
                    Eigen::Vector3i chk_idx = cor_it + Eigen::Vector3i(x, y, z);
                    if(!IsFeasible(chk_idx)) return false;
                }
            }
        }   
    }
    cor_start = cor_start_temp;
    cor_end = cor_end_temp;
    return true;
}

void LowResMap::CoarseExpand(Eigen::Vector3i &coridx_start, Eigen::Vector3i &coridx_end){
    vector<Eigen::Vector3i> corners(2);
    vector<bool> expand(6, true);
    vector<int> expanded(6, 0);
    vector<Eigen::Vector3d> debug_pts;
    corners[0] = coridx_end;
    corners[1] = coridx_start;
    // RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "CoarseExpand0");
    while(1){
        bool expandable = false;
        // RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "CoarseExpand0.1");

        for(int dim = 0; dim < 3; dim++){
            for(int dir = -1; dir <= 1; dir += 2){
                if(!expand[dim*2 + int((1-dir)/2)]) continue;
                Eigen::Vector3i corridor_scale = corners[0] - corners[1] + Eigen::Vector3i::Ones();
                Eigen::Vector3i chk_it1, chk_it2;
                Eigen::Vector3i start_p = corners[1];
                start_p(dim) = corners[int((1-dir)/2)](dim) + dir;

                if(!InsideMap(start_p)) expand[dim*2 + int((1-dir)/2)] = false;
                // RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "CoarseExpand0.112");

                for(chk_it1(0) = 0; chk_it1(0) < (dim == 0 ? 1 : corridor_scale(0)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(0)++){
                    for(chk_it1(1) = 0; chk_it1(1) < (dim == 1 ? 1 : corridor_scale(1)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(1)++){
                        for(chk_it1(2) = 0; chk_it1(2) < (dim == 2 ? 1 : corridor_scale(2)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(2)++){
                            // RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "CoarseExpand0.113");

                            chk_it2 = start_p + chk_it1;
                            if(!InsideMap(chk_it2)) RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "CoarseExpand");
                            // RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "CoarseExpand0.1134");//
                            Eigen::Vector3d p = IdtoPos(chk_it2);
                            int blockid = GetBlockId(p);
                            // cout<<p.transpose()<<"--"<<chk_it2.transpose()<<" ;;; "<<blockid<<endl;

                            if(IsFeasible(chk_it2) != VoxelState::free){
                                // RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "CoarseExpand0.1135");
                                expand[dim*2 + int((1-dir)/2)] = false;
                            }
                            // RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "CoarseExpand0.114");
                        }
                    }
                }

                expandable |= expand[dim*2 + int((1-dir)/2)];

                if(expand[dim*2 + int((1-dir)/2)]){
                    corners[int((1-dir)/2)](dim) += dir;
                    expanded[dim*2 + int((1-dir)/2)] += 1;
                    expand[dim*2 + int((1-dir)/2)] = (expanded[dim*2 + int((1-dir)/2)] < corridor_exp_r_(dim)) ? 1 : 0;
                }

            }
        }
        
        if(!expandable) break;
    }
    coridx_end = corners[0];
    coridx_start = corners[1];
    // RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "CoarseExpand1");

}

void LowResMap::FineExpand(Eigen::Vector3i &coridx_start, Eigen::Vector3i &coridx_end,  Eigen::Vector3d &up_corner, Eigen::Vector3d &down_corner){
    vector<Eigen::Vector3d> corners(2);
    vector<bool> expand(6, true);
    vector<double> expanded(6, 0.0);
    corners[0] = (coridx_end.cast<double>() + Eigen::Vector3d::Ones()).cwiseProduct(node_scale_) + origin_ - Eigen::Vector3d::Ones() * 1e-4;
    corners[1] = (coridx_start.cast<double>()).cwiseProduct(node_scale_) + origin_ + Eigen::Vector3d::Ones() * 1e-4;

    while(1){
        bool expandable = false;
        for(int dim = 0; dim < 3; dim++){
            for(int dir = -1; dir <= 1; dir += 2){
                if(!expand[dim*2 + int((1-dir)/2)]) continue;
                Eigen::Vector3d corridor_scale = corners[0] - corners[1];
                Eigen::Vector3d corridor_center = (corners[0] + corners[1]) * 0.5;
                Eigen::Vector3d chk_it1, chk_it2;
                Eigen::Vector3d start_p = corners[1] + Eigen::Vector3d::Ones() * resolution_ * 0.5;
                start_p(dim) = corridor_center(dim) + dir * 0.5 * (corridor_scale(dim) + resolution_);
                if(!InsideMap(start_p)) expand[dim*2 + int((1-dir)/2)] = false;
                for(chk_it1(0) = 0.0; chk_it1(0) < (dim == 0 ? resolution_ : corridor_scale(0)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(0) += resolution_){
                    for(chk_it1(1) = 0.0; chk_it1(1) < (dim == 1 ? resolution_ : corridor_scale(1)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(1) += resolution_){
                        for(chk_it1(2) = 0.0; chk_it1(2) < (dim == 2 ? resolution_ : corridor_scale(2))  - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(2) += resolution_){
                            chk_it2 = start_p + chk_it1;
                            // debug_pts.push_back(chk_it2);
                            if(map_->GetVoxState(chk_it2) != VoxelState::free){
                                // RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "occ");
                                expand[dim*2 + int((1-dir)/2)] = false;
                            }
                        }
                    }
                }

                expandable |= expand[dim*2 + int((1-dir)/2)];
                if(expand[dim*2 + int((1-dir)/2)]){
                    corners[int((1-dir)/2)](dim) += resolution_ * dir;
                    expanded[dim*2 + int((1-dir)/2)] += resolution_;
                    expand[dim*2 + int((1-dir)/2)] = (expanded[dim*2 + int((1-dir)/2)] < expand_r_(dim) * 0.5 - 1e-4) ? 1 : 0;
                }
            }
        }
        if(!expandable) break;
    }
    up_corner = corners[0];
    down_corner = corners[1];
}

void LowResMap::UpdateFOV(){
    Eigen::Vector3d nor1, nor2, nor3, nor4, xdir; 
    double d1, d2, d3, d4;
    Eigen::Matrix3d rot = Robot_pose_.block(0, 0, 3, 3);
    xdir = Robot_pose_.block(0, 0, 3, 1);
    //up
    nor1 = rot * Eigen::Vector3d(cos(ver_up_dir_), 0, sin(ver_up_dir_));
    nor1 = (- nor1 + nor1.dot(xdir) * xdir).normalized();
    d1 = nor1.dot(Robot_pos_);
    FOV_ieqs_.block(0, 0, 3, 1) = -nor1;
    FOV_ieqs_(3, 0) = d1;
    //down
    nor2 = rot * Eigen::Vector3d(cos(ver_down_dir_), 0, -sin(ver_down_dir_));
    nor2 = (- nor2 + nor2.dot(xdir) * xdir).normalized();
    d2 = nor2.dot(Robot_pos_);
    FOV_ieqs_.block(0, 1, 3, 1) = -nor2;
    FOV_ieqs_(3, 1) = d2;
    //left
    nor3 = rot * Eigen::Vector3d(cos(hor_left_dir_), sin(hor_left_dir_), 0);
    nor3 = (- nor3 + nor3.dot(xdir) * xdir).normalized();
    d3 = nor3.dot(Robot_pos_);
    FOV_ieqs_.block(0, 2, 3, 1) = -nor3;
    FOV_ieqs_(3, 2) = d3;
    //right
    nor4 = rot * Eigen::Vector3d(cos(hor_right_dir_), -sin(hor_right_dir_), 0);
    nor4 = (- nor4 + nor4.dot(xdir) * xdir).normalized();
    d4 = nor4.dot(Robot_pos_);
    FOV_ieqs_.block(0, 3, 3, 1) = -nor4;
    FOV_ieqs_(3, 3) = d4;
}

bool LowResMap::PrunePath(const list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &pruned_path, double &length){
    return PrunePath(path, pruned_path, length, prune_seg_length_);
}

bool LowResMap::PrunePath(const list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &pruned_path, double &length, double window){
    length = 0;
    if(path.size() == 0) return false;
    else if(path.size() == 1){
        pruned_path = path;
        return true;
    }
    pruned_path.clear();
    Eigen::Vector3d inv_res, ray_iter;
    Eigen::Vector3d half_res = 0.5 * node_scale_;
    Eigen::Vector3i p_i;
    list<Eigen::Vector3d> std_off_path;
    list<Eigen::Vector3d>::iterator ps_it, pe_it;
    bool free_ray;
    RayCaster rc;

    for(auto &p : path){
        PostoId3(p, p_i);
        std_off_path.push_back(IdtoPos(p_i) + Eigen::Vector3d::Ones() * 1e-4);
    }

    pruned_path.push_back(std_off_path.front());
    for(int dim = 0; dim < 3; dim++) inv_res(dim) = 1.0 / node_scale_(dim);
    for(list<Eigen::Vector3d>::iterator ps_it = std_off_path.begin(); pe_it != std_off_path.end(); pe_it++){
        pe_it = ps_it;
        double seg_length = 0;
        for(list<Eigen::Vector3d>::iterator pf_it = pe_it; pe_it != std_off_path.end() && seg_length < window; pe_it++) {
            seg_length += ((*pf_it) - (*pe_it)).norm();
            pf_it = pe_it;
        }
        if(pe_it != std_off_path.end()) pe_it--;
        pe_it--;

        while (1)
        {
            if(ps_it == pe_it) return false;
            free_ray = true;
            rc.setInput((*ps_it - origin_).cwiseProduct(inv_res), (*pe_it - origin_).cwiseProduct(inv_res));
            while(rc.step(ray_iter)){
                ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
                if(!IsFeasible(ray_iter)) {
                    free_ray = false;
                    break;
                }
            }
            ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
            if(!IsFeasible(ray_iter)) {
                free_ray = false;
            }

            if(free_ray) {
                length += (pruned_path.back() - (*pe_it)).norm();
                pruned_path.push_back(*pe_it);
                break;
            }
            pe_it--;
        }
        ps_it = pe_it;
    }
    length += (pruned_path.back() - path.back()).norm();
    length += (pruned_path.front() - path.front ()).norm();
    pruned_path.push_back(path.back());
    pruned_path.push_front(path.front());
    return true;
}

void LowResMap::GetSafePath(list<Eigen::Vector3d> &danger_path, list<Eigen::Vector3d> &safe_path, 
            list<Eigen::Vector3d> &unknown_path, const double &max_dist, const double &unknown_dist){
    safe_path.clear();
    double dist = 0;
    double u_dist = 0;
    if(!danger_path.empty()){
        safe_path.emplace_back(danger_path.front());
    }
    else{
        RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "empty/infeasible path");
        return;
    }
    RayCaster rc;
    for(auto pt : danger_path){
        if(!InsideMap(pt)){
            RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "GetSafePath out!!!");
            return;
        }
        if((pt - safe_path.back()).norm() > 1e-6){
            Eigen::Vector3d rc_s, rc_e, it;
            rc_s = safe_path.back() - origin_;
            rc_e = pt - origin_;

            for(int dim = 0; dim < 3; dim ++){
                rc_s(dim) /= node_scale_(dim);
                rc_e(dim) /= node_scale_(dim);
            }

            rc.setInput(rc_s, rc_e);
            while (rc.step(it)) {
                it = it.cwiseProduct(node_scale_) + origin_ + node_scale_/2;
                dist += (it - safe_path.back()).norm();
                if(dist < 1e-6) continue;
                if(IsFeasible(it) && dist < max_dist){
                    safe_path.emplace_back(it);
                    // cout<<"dang push1:"<<it.transpose()<<endl;
                }
                else{
                    unknown_path.emplace_back(it);
                    // cout<<"dang unknown_path1:"<<it.transpose()<<endl;
                    return;
                }
            }
            it = it.cwiseProduct(node_scale_) + origin_ + node_scale_/2;
            dist += (it - safe_path.back()).norm();
            if(IsFeasible(it) && dist < max_dist){
                safe_path.emplace_back(it);
                // cout<<"dang push2:"<<it.transpose()<<endl;
            }
            else{
                unknown_path.emplace_back(it);
                // cout<<"dang unknown_path2:"<<it.transpose()<<endl;
                return;
            }
        }
    }
}

bool LowResMap::PrunePathTempt(const list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &pruned_path, double &length){
    length = 0;
    if(path.size() == 0) return false;
    else if(path.size() == 1){
        pruned_path = path;
        return true;
    }
    pruned_path.clear();
    Eigen::Vector3d inv_res, ray_iter;
    Eigen::Vector3d half_res = 0.5 * node_scale_;
    Eigen::Vector3i p_i;
    list<Eigen::Vector3d> std_off_path;
    list<Eigen::Vector3d>::iterator ps_it, pe_it;
    bool free_ray;
    RayCaster rc;

    for(auto &p : path){
        PostoId3(p, p_i);
        std_off_path.push_back(IdtoPos(p_i) + Eigen::Vector3d::Ones() * 1e-4);
    }

    pruned_path.push_back(std_off_path.front());
    for(int dim = 0; dim < 3; dim++) inv_res(dim) = 1.0 / node_scale_(dim);
    for(list<Eigen::Vector3d>::iterator ps_it = std_off_path.begin(); pe_it != std_off_path.end(); pe_it++){
        pe_it = ps_it;
        double seg_length = 0;
        for(list<Eigen::Vector3d>::iterator pf_it = pe_it; pe_it != std_off_path.end() && seg_length < prune_seg_length_; pe_it++) {
            seg_length += ((*pf_it) - (*pe_it)).norm();
            pf_it = pe_it;
        }
        if(pe_it != std_off_path.end()) pe_it--;
        pe_it--;

        while (1)
        {
            if(ps_it == pe_it) return false;
            free_ray = true;
            rc.setInput((*ps_it - origin_).cwiseProduct(inv_res), (*pe_it - origin_).cwiseProduct(inv_res));
            while(rc.step(ray_iter)){
                ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
                if(!IsLocalFeasible(ray_iter)) {
                    free_ray = false;
                    break;
                }
            }
            ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
            if(!IsLocalFeasible(ray_iter)) {
                free_ray = false;
            }

            if(free_ray) {
                length += (pruned_path.back() - (*pe_it)).norm();
                pruned_path.push_back(*pe_it);
                break;
            }
            pe_it--;
        }
        ps_it = pe_it;
    }
    length += (pruned_path.back() - path.back()).norm();
    length += (pruned_path.front() - path.front ()).norm();
    pruned_path.push_back(path.back());
    pruned_path.push_front(path.front());
    return true;
}


void LowResMap::LoadShowList(){
    mtx_.lock();
    Eigen::Vector3i startblck, endblck, searchidx;
    bool add;
    startblck(0) = max(0.0, floor(local_low_idx_(0)/block_size_(0))-1);
    startblck(1) = max(0.0, floor(local_low_idx_(1)/block_size_(1))-1);
    startblck(2) = max(0.0, floor(local_low_idx_(2)/block_size_(2))-1);
    endblck(0) = min(block_num_(0)-1.0, ceil(local_up_idx_(0)/block_size_(0))+1);
    endblck(1) = min(block_num_(1)-1.0, ceil(local_up_idx_(1)/block_size_(1))+1);
    endblck(2) = min(block_num_(2)-1.0, ceil(local_up_idx_(2)/block_size_(2))+1);

    for(searchidx(0) = startblck(0); searchidx(0) <= endblck(0); searchidx(0)+=1){
        for(searchidx(1) = startblck(1); searchidx(1) <= endblck(1); searchidx(1)+=1){
            for(searchidx(2) = startblck(2); searchidx(2) <= endblck(2); searchidx(2)+=1){
                int idx = GetBlockId(searchidx);
                if(idx != -1){
                    add = true;
                    for(list<int>::iterator slit = Showblocklist_.begin(); slit != Showblocklist_.end(); slit++){
                        if(*slit == idx){
                            add = false;
                            break;
                        }
                    }
                    if(add){
                        Showblocklist_.push_front(idx);
                    }
                }
            }
        }
    }
    mtx_.unlock();
}

void LowResMap::ShowGridLocal(){
    mtx_.lock();
    if(Showblocklist_.size() > 0 && node_pub_->get_subscription_count() > 0){
        visualization_msgs::msg::MarkerArray MKArray;
        MKArray.markers.resize(Showblocklist_.size()*2);
        //load makers
        int i = 0;
        for(list<int>::iterator idit = Showblocklist_.begin(); idit != Showblocklist_.end(); idit++){
            MKArray.markers[i].action = visualization_msgs::msg::Marker::ADD;
            MKArray.markers[i].pose.orientation.w = 1.0;
            MKArray.markers[i].type = visualization_msgs::msg::Marker::SPHERE_LIST;      //nodes
            MKArray.markers[i].scale.x = resolution_/2;
            MKArray.markers[i].scale.y = resolution_/2;
            MKArray.markers[i].scale.z = resolution_/2;
            MKArray.markers[i].header.frame_id = "world";
            MKArray.markers[i].header.stamp = node_->now();
            MKArray.markers[i].id = (*idit)*2;
            i++;
            MKArray.markers[i] = MKArray.markers[i-1];
            MKArray.markers[i].id++;
            MKArray.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;        //Xnodes

            MKArray.markers[i].scale.x *= 2;
            MKArray.markers[i].scale.y *= 2;
            MKArray.markers[i].scale.z *= 2;
            // MKArray.markers[i].color.r = 0.2;
            // MKArray.markers[i].color.g = 0.25;
            // MKArray.markers[i].color.b = 0.65;
            // MKArray.markers[i].color.a = 0.5;
            i++;
        }
        i = 0;
        Eigen::Vector3d pos;
        Eigen::Vector3i iterp;
        geometry_msgs::msg::Point pt;
        int nodeid;
        std_msgs::msg::ColorRGBA localcolor, globalcolor, Xcolor, Ecolor;
        Ecolor.a = 1.0;
        Ecolor.g = 1.0;

        Xcolor.a = 1.0;
        Xcolor.r = 1.0;

//  localcolor is the lowres map color
        localcolor.a = 1.0;
        localcolor.b = 0.3;
        localcolor.g = 0.5;
        localcolor.r = 0.8;
        globalcolor.a = 1.0;
        globalcolor.b = 0.8;
        globalcolor.g = 0.5;
        globalcolor.r = 0.3;
        //publish nodes
        for(list<int>::iterator idit = Showblocklist_.begin(); idit != Showblocklist_.end(); idit++){
            if(gridBLK_[*idit] == NULL){
                MKArray.markers[i].color.a = 0.2;
                MKArray.markers[i].type = visualization_msgs::msg::Marker::CUBE;
                MKArray.markers[i].action = visualization_msgs::msg::Marker::DELETE;
            }
            else{
                //clear flag
                // gridBLK_[*idit]->show_flag_ = false;
                //load points in block
                for(iterp(0) = 0; iterp(0) < gridBLK_[*idit]->block_size_(0); iterp(0)++){
                    for(iterp(1) = 0; iterp(1) < gridBLK_[*idit]->block_size_(1); iterp(1)++){
                        for(iterp(2) = 0; iterp(2) < gridBLK_[*idit]->block_size_(2); iterp(2)++){
                            nodeid = iterp(2)*gridBLK_[*idit]->block_size_(0)*gridBLK_[*idit]->block_size_(1)+
                                iterp(1)*gridBLK_[*idit]->block_size_(0) + iterp(0);
                            
                            if(gridBLK_[*idit]->local_grid_[nodeid] != NULL){
                                pt.x = (iterp(0)+gridBLK_[*idit]->origin_(0)+0.5)*node_scale_(0)+origin_(0);
                                pt.y = (iterp(1)+gridBLK_[*idit]->origin_(1)+0.5)*node_scale_(1)+origin_(1);
                                pt.z = (iterp(2)+gridBLK_[*idit]->origin_(2)+0.5)*node_scale_(2)+origin_(2);
                                // if(gridBLK_[*idit]->local_grid_[nodeid] == Expandnode_){//debug
                                //     MKArray.markers[i].colors.push_back(Ecolor);
                                //     MKArray.markers[i].points.push_back(pt);
                                // }
                                // else if(gridBLK_[*idit]->local_grid_[nodeid]->flags_[0] && gridBLK_[*idit]->local_grid_[nodeid]->flags_[3]){//debug
                                //     MKArray.markers[i+1].points.push_back(pt);
                                //     MKArray.markers[i+1].colors.push_back(Ecolor);
                                // }
                                // else if(gridBLK_[*idit]->local_grid_[nodeid]->flags_[0]){//debug
                                //     MKArray.markers[i+1].points.push_back(pt);
                                //     MKArray.markers[i+1].colors.push_back(Xcolor);
                                // }
                                // else 

                                // visualize node tag by color
                                if(!gridBLK_[*idit]->local_grid_[nodeid]->flags_[0]){
                                    switch (gridBLK_[*idit]->local_grid_[nodeid]->tag_)
                                    {
                                    case AERO:
                                        localcolor.a = 0.001;

                                        localcolor.b = 0.9;
                                        break;
                                    case GROUND:
                                        localcolor.a = 1.0;
                                        localcolor.b = 0.3;
                                        localcolor.b = gridBLK_[*idit]->local_grid_[nodeid]->t_score_;
                                        // cout<<"t_score "<<gridBLK_[*idit]->local_grid_[nodeid]->t_score_<<endl;

                                        break;
                                    case TRANS:
                                        localcolor.a = 1.0;
                                        localcolor.b = 0.6;
                                        break;

                                    default:
                                        localcolor.a = 1.0;
                                        localcolor.b = 0.3;
                                        localcolor.g = 0.5;
                                        localcolor.r = 0.8;
                                    }
                                
                                    MKArray.markers[i].colors.push_back(localcolor);
                                    MKArray.markers[i].points.push_back(pt);
                                }
                                // else{
                                //     MKArray.markers[i].colors.push_back(globalcolor);
                                //     MKArray.markers[i].points.push_back(pt);
                                // }
                                i++;
                                if(show_dtg_){
                                    pt.y -= resolution_ * int(gridBLK_[*idit]->local_grid_[nodeid]->ties_.size()); 
                                    pt.z -= resolution_; 
                                    for(auto &tie : gridBLK_[*idit]->local_grid_[nodeid]->ties_){
                                        pt.y += resolution_ * 2;
                                        MKArray.markers[i].colors.push_back(CM_->Id2Color(int(tie.root_id_) % drone_num_, 0.5));
                                        MKArray.markers[i].points.push_back(pt);
                                    }
                                }
                                i--;
                            }
                        }
                    }
                }
            }
            i+=2;
        }        
        //publish FOV, to do
        i = 0;
        for(list<int>::iterator idit = Showblocklist_.begin(); idit != Showblocklist_.end(); idit++){
            if(MKArray.markers[i].points.size() == 0){
                MKArray.markers[i].color.a = 0.2;
                MKArray.markers[i].type = visualization_msgs::msg::Marker::CUBE;
                MKArray.markers[i].action = visualization_msgs::msg::Marker::DELETE;
            }
            if(MKArray.markers[i+1].points.size() == 0){
                MKArray.markers[i+1].color.a = 0.2;
                MKArray.markers[i+1].type = visualization_msgs::msg::Marker::CUBE;
                MKArray.markers[i+1].action = visualization_msgs::msg::Marker::DELETE;
            }
            i+=2;
        }  
        Showblocklist_.clear();
        node_pub_->publish(MKArray);
    }
    // if(debug_){
    //     vector<Eigen::Vector4d> pts;
    //     Debug(pts);
    // }
    mtx_.unlock();
}

void LowResMap::DebugTie(const Eigen::Vector3d &p){
    shared_ptr<LR_node> node = GetNode(p);
    cout<<"debug p:"<<p.transpose()<<endl;
    if(node == NULL){
        RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "NULL TIE");
        return;
    }
    else{
        bool success = false;
        for(auto & tie : node->ties_){
            if(tie.in_dir_ == 128) success = true;
            cout<<"tie id:"<<int(tie.root_id_)<<" in:"<<int(tie.in_dir_)<<" out:"<<int(tie.out_dir_)<<endl;
        }
        // if(!success){
        //     RCLCPP_ERROR(rclcpp::get_logger("lowres_map"), "debug error");
        //     rclcpp::shutdown();
        // }
    }
}

void LowResMap::Debug(vector<Eigen::Vector4d> &pts){
    visualization_msgs::msg::Marker mk;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::msg::Marker::CUBE_LIST;      //nodes
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    mk.header.frame_id = "world";
    mk.header.stamp = node_->now();
    mk.id = 0;
    mk.color.a = 0.7;
    mk.color.g = 1.0;
    for(auto & blk : gridBLK_){
        if(blk != NULL){
            for(int x = 0; x < blk->block_size_(0); x++)
                for(int y = 0; y < blk->block_size_(1); y++)
                    for(int z = 0; z < blk->block_size_(2); z++){
                shared_ptr<LR_node> node = blk->local_grid_[z*blk->block_size_(0)*blk->block_size_(1) + y*blk->block_size_(0) + x];
                if(node != NULL && node->flags_[0]){
                    geometry_msgs::msg::Point pt;
                    pt.x = blk->origin_(0) * node_scale_(0) + node_scale_(0) * x + node_scale_(0) / 2 + origin_(0);
                    pt.y = blk->origin_(1) * node_scale_(1) + node_scale_(1) * y + node_scale_(1) / 2 + origin_(1);
                    pt.z = blk->origin_(2) * node_scale_(2) + node_scale_(2) * z + node_scale_(2) / 2 + origin_(2);
                    mk.points.emplace_back(pt);
                }
            }
        }
    }
    debug_pub_->publish(mk);

}

void LowResMap::Debug(vector<Eigen::Vector3d> &pts){
    visualization_msgs::msg::Marker mk;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;      //nodes
    mk.scale.x = 0.15;
    mk.scale.y = 0.15;
    mk.scale.z = 0.15;
    mk.header.frame_id = "world";
    mk.header.stamp = node_->now();
    mk.id = 0;
    mk.color.a = 0.7;
    mk.color.r = 1.0;
    Eigen::Vector3d pos;
    geometry_msgs::msg::Point pt;

    for(int i = 0; i < pts.size(); i++){
        pt.x = pts[i](0);
        pt.y = pts[i](1);
        pt.z = pts[i](2);
        mk.points.push_back(pt);
    }
    debug_pub_->publish(mk);
}

void LowResMap::Debug2(vector<Eigen::Vector3d> &pts){
    visualization_msgs::msg::Marker mk;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;      //nodes
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.header.frame_id = "world";
    mk.header.stamp = node_->now();
    mk.id = -1;
    mk.color.a = 0.6;
    mk.color.b = 1.0;
    Eigen::Vector3d pos;
    geometry_msgs::msg::Point pt;

    for(int i = 0; i < pts.size(); i++){
        pt.x = pts[i](0) ;
        pt.y = pts[i](1) ;
        pt.z = pts[i](2) ;
        mk.points.push_back(pt);
    }
    debug_pub_->publish(mk);
}


void LowResMap::Debug2(list<Eigen::Vector3d> &pts){
    visualization_msgs::msg::Marker mk;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::msg::Marker::POINTS;      //nodes
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    mk.header.frame_id = "world";
    mk.header.stamp = node_->now();
    mk.id = 1;
    mk.color.a = 0.6;
    mk.color.b = 1.0;
    Eigen::Vector3d pos;
    geometry_msgs::msg::Point pt;

    for(auto &p : pts){
        pt.x = p(0) ;
        pt.y = p(1) ;
        pt.z = p(2) ;
        mk.points.push_back(pt);
    }
    debug_pub_->publish(mk);
}

void LowResMap::Debug(const uint32_t &id){
    visualization_msgs::msg::Marker mk;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::msg::Marker::LINE_LIST;      //nodes
    mk.scale.x = 0.05;
    mk.scale.y = 0.05;
    mk.scale.z = 0.05;
    mk.header.frame_id = "world";
    mk.header.stamp = node_->now();
    mk.id = 0;
    mk.color.a = 0.7;
    mk.color.g = 1.0;

    geometry_msgs::msg::Point pt;
    int have_tie = 0;
    for(auto &bk : gridBLK_){
        if(bk == NULL) continue;
        Eigen::Vector3i it = bk->origin_;
        Eigen::Vector3i pt_s, pt_e;
        double l;
        for(it(0) = bk->origin_(0); it(0) < bk->block_size_(0) + bk->origin_(0); it(0)++)
            for(it(1) = bk->origin_(1); it(1) < bk->block_size_(1) + bk->origin_(1); it(1)++)
                for(it(2) = bk->origin_(2); it(2) < bk->block_size_(2) + bk->origin_(2); it(2)++){
            int l_id = GetNodeId(it, bk);
            if(bk->local_grid_[l_id] == NULL) continue;
            // if(!bk->local_grid_[l_id]->flags_[0]) have_tie++;
            for(auto &tie : bk->local_grid_[l_id]->ties_){
                if(tie.root_id_ == id){
                    pt_s = it;
                    pt_e = it;
                    if(TopoDirIter(tie.in_dir_, pt_e, l)){
                        pt.x = (pt_s(0)+0.5) * node_scale_(0) + origin_(0);
                        pt.y = (pt_s(1)+0.5) * node_scale_(1) + origin_(1);
                        pt.z = (pt_s(2)+0.5) * node_scale_(2) + origin_(2);
                        mk.points.emplace_back(pt);
                        pt.x = (pt_e(0)+0.5) * node_scale_(0) + origin_(0);
                        pt.y = (pt_e(1)+0.5) * node_scale_(1) + origin_(1);
                        pt.z = (pt_e(2)+0.5) * node_scale_(2) + origin_(2);
                        mk.points.emplace_back(pt);
                    }
                    break;
                }
            }
        }
    }
    debug_pub_->publish(mk);
}


void LowResMap::Debug2(list<list<Eigen::Vector3d>> &paths){
    visualization_msgs::msg::Marker mk;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::msg::Marker::LINE_LIST;      //nodes
    mk.scale.x = 0.02;
    mk.scale.y = 0.02;
    mk.scale.z = 0.02;
    mk.header.frame_id = "world";
    mk.header.stamp = node_->now();
    mk.id = -1;
    mk.color.a = 0.3;
    mk.color.b = 1.0;
    Eigen::Vector3d pos;
    geometry_msgs::msg::Point pt;

    for(auto &path : paths){
        vector<Eigen::Vector3d> pth;
        if(path.size() <= 1) continue;
        for(auto &p : path) pth.emplace_back(p);
        for(int i = 1; i < pth.size(); i++){
            pt.x = pth[i-1](0);
            pt.y = pth[i-1](1);
            pt.z = pth[i-1](2);
            mk.points.emplace_back(pt);
            pt.x = pth[i](0);
            pt.y = pth[i](1);
            pt.z = pth[i](2);
            mk.points.emplace_back(pt);
        }
    }
    debug_pub_->publish(mk);
}

void LowResMap::OfflineInitLowResMap() {
    auto gp = [&](const std::string &name, auto def) {
        if (!node_->has_parameter(name)) node_->declare_parameter(name, def);
        return node_->get_parameter(name).get_value<decltype(def)>();
    };

    ground_thr_       = (float)gp("LowResMap.ground_thr",        0.6);
    min_aero_height_  = gp("LowResMap.min_aero_height",          0.5);
    max_aero_height_  = gp("LowResMap.max_aero_height",          5.0);
    topo_spacing_thr_ = gp("LowResMap.topo_spacing_thr", 3.0);
    topo_sample_prob_ = gp("LowResMap.topo_sample_prob", 0.8);
    topo_max_range_   = gp("LowResMap.topo_max_range",  12.0);

    next_topo_id_ = 0;
    topo_nodes_.clear();
    topo_adj_.clear();
    node_topo_id_.clear();

    std::default_random_engine rng(42);
    std::uniform_real_distribution<double> dist01(0.0, 1.0);

    // Coarse grid for O(1) TopoNode spacing check
    int cs = std::max(1, (int)std::ceil(topo_spacing_thr_ / node_scale_.minCoeff()));
    int64_t cny = voxel_num_.y() / cs + 2;
    int64_t cnz = voxel_num_.z() / cs + 2;
    std::unordered_set<int64_t> topo_coarse_ground;
    std::unordered_set<int64_t> topo_coarse_aero;
    auto coarse_key = [&](int ix, int iy, int iz) -> int64_t {
        return (int64_t)(ix/cs) * cny * cnz
             + (int64_t)(iy/cs) * cnz
             + (int64_t)(iz/cs);
    };

    int occ_cnt = 0, free_cnt = 0;
    std::vector<Eigen::Vector3i> aero_anchors;

    for (int ix = 0; ix < voxel_num_.x(); ix++) {
        for (int iy = 0; iy < voxel_num_.y(); iy++) {
            for (int iz = 0; iz < voxel_num_.z(); iz++) {
                Eigen::Vector3i node3i(ix, iy, iz);
                Eigen::Vector3d pos = IdtoPos(node3i);

                Eigen::Vector3i blockid;
                if (!GetBlock3Id(pos, blockid)) continue;
                int blkid = blockid.z()*b_n_.y() + blockid.y()*b_n_.x() + blockid.x();

                if (gridBLK_[blkid] == nullptr) {
                    gridBLK_[blkid] = std::make_shared<LR_block>();
                    gridBLK_[blkid]->origin_ = Eigen::Vector3i(
                        blockid.x()*block_size_.x(),
                        blockid.y()*block_size_.y(),
                        blockid.z()*block_size_.z());
                    Eigen::Vector3i bsz = GetBolckSize(blockid);
                    gridBLK_[blkid]->block_size_ = bsz;
                    gridBLK_[blkid]->local_grid_.resize(bsz.x()*bsz.y()*bsz.z());
                    gridBLK_[blkid]->has_ground_ = false;
                }

                int nodeid = GetNodeId(node3i, gridBLK_[blkid]);
                if (gridBLK_[blkid]->local_grid_[nodeid] == nullptr)
                    gridBLK_[blkid]->local_grid_[nodeid] = std::make_shared<LR_node>();

                auto &lr = gridBLK_[blkid]->local_grid_[nodeid];
                // Offline: only check the center BM voxel to avoid misclassifying
                // nodes adjacent to ground as Xnodes
                VoxelState center_state = map_->GetVoxState(pos);

                if (center_state == VoxelState::occupied) {
                    lr->flags_.reset();
                    lr->flags_.set(0);
                    lr->last_update_ = 0.0;
                    Xlist_.push_back({blkid, nodeid});
                    occ_cnt++;
                } else {
                    lr->flags_.reset();
                    lr->flags_.set(1);

                    float score = GetNodeTraversabilityScoreOffline(node3i);
                    lr->t_score_ = score;
                    if (score >= ground_thr_) {
                        lr->tag_ = GROUND;
                        gridBLK_[blkid]->has_ground_ = true;
                        aero_anchors.push_back(node3i);
                    } else if (CheckNode(node3i) == 0 && score > 0.0f) {
                        lr->tag_ = DANGEROUS;
                        aero_anchors.push_back(node3i);
                    } else {
                        lr->tag_ = UNKNOWN;
                    }
                    free_cnt++;

                    // TopoNode sampling: GROUND only in pass 1
                    if (lr->tag_ == GROUND) {
                        bool far_enough = true;
                        int cx = ix/cs, cy = iy/cs, cz_c = iz/cs;
                        for (int dcx = -1; dcx <= 1 && far_enough; dcx++)
                            for (int dcy = -1; dcy <= 1 && far_enough; dcy++)
                                for (int dcz = -1; dcz <= 1 && far_enough; dcz++)
                                    if (topo_coarse_ground.count(coarse_key(
                                            (cx+dcx)*cs, (cy+dcy)*cs, (cz_c+dcz)*cs)))
                                        far_enough = false;

                        if (far_enough && dist01(rng) < topo_sample_prob_) {
                            uint32_t tid = next_topo_id_++;
                            topo_nodes_[tid] = {tid, pos, lr->tag_};
                            topo_adj_[tid]   = {};
                            node_topo_id_[PostoId(pos)] = tid;
                            topo_coarse_ground.insert(coarse_key(ix, iy, iz));
                        }
                    }
                }
            }
        }
    }

    // Pass 2a: project TRANS upward from anchors for nodes below min_aero_height
    int aero_step_min = (int)std::ceil(min_aero_height_ / node_scale_.z());
    int aero_step_max = (int)std::floor(max_aero_height_ / node_scale_.z());
    int trans_cnt = 0;

    for (const auto &anchor : aero_anchors) {
        for (int diz = 1; diz < aero_step_min; diz++) {
            Eigen::Vector3i a3i(anchor.x(), anchor.y(), anchor.z() + diz);
            if (!InsideMap(a3i)) continue;

            Eigen::Vector3d apos = IdtoPos(a3i);
            Eigen::Vector3i blockid;
            if (!GetBlock3Id(apos, blockid)) continue;
            int blkid = blockid.z()*b_n_.y() + blockid.y()*b_n_.x() + blockid.x();
            if (gridBLK_[blkid] == nullptr) continue;

            int nodeid = GetNodeId(a3i, gridBLK_[blkid]);
            auto &lr = gridBLK_[blkid]->local_grid_[nodeid];
            if (lr == nullptr || lr->flags_[0]) continue;
            if (lr->tag_ != UNKNOWN) continue;

            if (CheckNode(a3i) == 0) {
                lr->tag_ = TRANS;
                trans_cnt++;
            }
        }
    }

    // Pass 2b: project AERO upward from confirmed surface nodes
    int aero_cnt = 0;

    for (const auto &anchor : aero_anchors) {
        for (int diz = aero_step_min; diz <= aero_step_max; diz++) {
            Eigen::Vector3i a3i(anchor.x(), anchor.y(), anchor.z() + diz);
            if (!InsideMap(a3i) ) continue;

            Eigen::Vector3d apos = IdtoPos(a3i);
            Eigen::Vector3i blockid;
            if (!GetBlock3Id(apos, blockid) || apos.z() > max_aero_height_) continue;
            int blkid = blockid.z()*b_n_.y() + blockid.y()*b_n_.x() + blockid.x();
            if (gridBLK_[blkid] == nullptr) continue;

            int nodeid = GetNodeId(a3i, gridBLK_[blkid]);
            auto &lr = gridBLK_[blkid]->local_grid_[nodeid];
            if (lr == nullptr || lr->flags_[0]) continue;  // skip Xnodes
            if (lr->tag_ != UNKNOWN) continue;             // only upgrade UNKNOWN

            if (CheckNode(a3i) == 0) {
                lr->tag_ = AERO;
                aero_cnt++;

                // TopoNode sampling for AERO
                int ax = a3i.x(), ay = a3i.y(), az = a3i.z();
                bool far_enough = true;
                for (int dcx = -1; dcx <= 1 && far_enough; dcx++)
                    for (int dcy = -1; dcy <= 1 && far_enough; dcy++)
                        for (int dcz = -1; dcz <= 1 && far_enough; dcz++)
                            if (topo_coarse_aero.count(coarse_key(
                                    (ax/cs+dcx)*cs, (ay/cs+dcy)*cs, (az/cs+dcz)*cs)))
                                far_enough = false;

                if (far_enough && dist01(rng) < topo_sample_prob_) {
                    uint32_t tid = next_topo_id_++;
                    topo_nodes_[tid] = {tid, apos, AERO};
                    topo_adj_[tid]   = {};
                    node_topo_id_[PostoId(apos)] = tid;
                    topo_coarse_aero.insert(coarse_key(ax, ay, az));
                }
            }
        }
    }

    // Pass 3: horizontal 8-direction expansion of AERO nodes
    // Collect all AERO node indices first, then expand once
    std::vector<Eigen::Vector3i> aero_nodes;
    for (int ix = 0; ix < voxel_num_.x(); ix++)
        for (int iy = 0; iy < voxel_num_.y(); iy++)
            for (int iz = 0; iz < voxel_num_.z(); iz++) {
                Eigen::Vector3i n3i(ix, iy, iz);
                auto lr = GetNode(IdtoPos(n3i));
                if (lr && lr->tag_ == AERO) aero_nodes.push_back(n3i);
            }

    int aero_exp_cnt = 0;
    const int HX[8] = {1,-1, 0, 0, 1, 1,-1,-1};
    const int HY[8] = {0, 0, 1,-1, 1,-1, 1,-1};

    for (const auto &an : aero_nodes) {
        for (int d = 0; d < 8; d++) {
            Eigen::Vector3i np(an.x() + HX[d], an.y() + HY[d], an.z());
            if (!InsideMap(np)) continue;

            Eigen::Vector3d npos = IdtoPos(np);
            Eigen::Vector3i blockid;
            if (!GetBlock3Id(npos, blockid)) continue;
            int blkid = blockid.z()*b_n_.y() + blockid.y()*b_n_.x() + blockid.x();
            if (gridBLK_[blkid] == nullptr) continue;

            int nodeid = GetNodeId(np, gridBLK_[blkid]);
            auto &lr = gridBLK_[blkid]->local_grid_[nodeid];
            if (lr == nullptr || lr->flags_[0]) continue;
            if (lr->tag_ != UNKNOWN) continue;

            if (CheckNode(np) == 0) {
                lr->tag_ = AERO;
                aero_exp_cnt++;
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("lowres_map"),
        "OfflineInitLowResMap: occ=%d free=%d trans=%d aero=%d aero_exp=%d topo_nodes=%zu",
        occ_cnt, free_cnt, trans_cnt, aero_cnt, aero_exp_cnt, topo_nodes_.size());
}

void LowResMap::OfflineBuildTopoModAware() {
    using std::queue;
    if (topo_nodes_.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "OfflineBuildTopoModAware: no topo nodes");
        return;
    }

    // ── helpers ──────────────────────────────────────────────────────────

    // Linear grid index from node 3i (consistent with PostoId(IdtoPos(p)))
    auto gid = [&](const Eigen::Vector3i& p) -> int {
        return p.z()*v_n_(1) + p.y()*voxel_num_(0) + p.x();
    };

    // Backtrack sch_node parent chain → world-coord path (start→end order)
    auto extract_path = [&](const shared_ptr<sch_node>& leaf) {
        std::vector<Eigen::Vector3d> path;
        for (auto cur = leaf; cur; cur = cur->parent_)
            path.push_back(IdtoPos(cur->pos_));
        std::reverse(path.begin(), path.end());
        return path;
    };

    // Add bidirectional edge (skip if already exists)
    auto add_edge = [&](uint32_t a, uint32_t b, float cost,
                        EdgeTag etag, bool gate,
                        const std::vector<Eigen::Vector3d>& path_ab) {
        for (auto& e : topo_adj_[a]) if (e.to_ == b) return;
        std::vector<Eigen::Vector3d> path_ba(path_ab.rbegin(), path_ab.rend());
        topo_adj_[a].push_back({a, b, cost, etag, gate, path_ab});
        topo_adj_[b].push_back({b, a, cost, etag, gate, path_ba});
    };

    // 6-connectivity offsets
    const int DX[6] = {1,-1,0,0,0,0};
    const int DY[6] = {0,0,1,-1,0,0};
    const int DZ[6] = {0,0,0,0,1,-1};

    // Dijkstra through nodes of tag_filter only.
    // Returns: hits = { topo_id → sch_node } for all reached TopoNodes.
    auto tag_dijkstra = [&](const Eigen::Vector3i& start, T_TAG tag_filter,
                             std::unordered_map<uint32_t, shared_ptr<sch_node>>& hits) {
        hits.clear();
        prio_D open;
        std::unordered_map<int, shared_ptr<sch_node>> seen;

        auto s = make_shared<sch_node>();
        s->pos_ = start; s->g_score_ = 0.0;
        s->parent_ = nullptr; s->status_ = not_expand;
        seen[gid(start)] = s;
        open.push(s);

        while (!open.empty()) {
            auto cur = open.top(); open.pop();
            if (cur->status_ == in_close) continue;
            cur->status_ = in_close;

            // Record if this is a TopoNode (after finalising optimal cost)
            auto tit = node_topo_id_.find(PostoId(IdtoPos(cur->pos_)));
            if (tit != node_topo_id_.end())
                hits[tit->second] = cur;

            for (int i = 0; i < 6; i++) {
                Eigen::Vector3i np = cur->pos_ + Eigen::Vector3i(DX[i], DY[i], DZ[i]);
                if (!InsideMap(np)) continue;
                auto lr = GetNode(IdtoPos(np));
                if (!lr || lr->flags_[0]) continue;
                // GROUND dijkstra: GROUND only; AERO dijkstra: AERO + TRANS passable
                bool passable = (lr->tag_ == tag_filter)
                             || (tag_filter == AERO && lr->tag_ == TRANS);
                if (!passable) continue;

                double ng = cur->g_score_ + GetDist(DX[i], DY[i], DZ[i]);
                if (ng > topo_max_range_) continue;

                int ni = gid(np);
                auto it = seen.find(ni);
                if (it != seen.end() && it->second->g_score_ <= ng) continue;

                auto nn = make_shared<sch_node>();
                nn->pos_ = np; nn->g_score_ = ng;
                nn->parent_ = cur; nn->status_ = not_expand;
                seen[ni] = nn;
                open.push(nn);
            }
        }
    };

    // Directional Dijkstra through GROUND/AERO/TRANS nodes.
    // z_dir=+1: upward (GROUND→AERO), z_dir=-1: downward (AERO→GROUND).
    // Cost: vertical step = base; horizontal step = base * H_PEN (strongly discouraged);
    //       wrong-direction z step = base * Z_BACK_PEN.
    // Target check happens when a node is *closed* (optimal cost confirmed).
    // Returns the first closed target-tag TopoNode, which is the one with minimum
    // penalised cost — i.e. the nearest one that required the least horizontal detour.
    auto cross_search = [&](const Eigen::Vector3i& start, T_TAG target_tag,
                             int z_dir,
                             const std::unordered_set<uint32_t>& already_visited,
                             uint32_t& found_id,
                             std::vector<Eigen::Vector3d>& found_path) -> bool {
        constexpr double H_PEN      = 5.0;   // horizontal step multiplier
        constexpr double Z_BACK_PEN = 3.0;   // wrong-direction z multiplier

        prio_D open;
        std::unordered_map<int, shared_ptr<sch_node>> seen;

        auto s = make_shared<sch_node>();
        s->pos_ = start; s->g_score_ = 0.0;
        s->parent_ = nullptr; s->status_ = not_expand;
        seen[gid(start)] = s;
        open.push(s);

        while (!open.empty()) {
            auto cur = open.top(); open.pop();
            if (cur->status_ == in_close) continue;
            cur->status_ = in_close;

            // Check on close: this node's cost is now optimal
            if (cur->pos_ != start) {
                auto tit = node_topo_id_.find(PostoId(IdtoPos(cur->pos_)));
                if (tit != node_topo_id_.end()) {
                    auto tnit = topo_nodes_.find(tit->second);
                    if (tnit != topo_nodes_.end()
                            && tnit->second.tag_ == target_tag
                            && !already_visited.count(tit->second)) {
                        found_id   = tit->second;
                        found_path = extract_path(cur);
                        return true;
                    }
                }
            }

            for (int i = 0; i < 6; i++) {
                Eigen::Vector3i np = cur->pos_ + Eigen::Vector3i(DX[i], DY[i], DZ[i]);
                if (!InsideMap(np)) continue;
                auto lr = GetNode(IdtoPos(np));
                if (!lr || lr->flags_[0]) continue;
                if (lr->tag_ == UNKNOWN) continue;
                // if (lr->tag_ == UNKNOWN || lr->tag_ == DANGEROUS) continue;

                double base = GetDist(DX[i], DY[i], DZ[i]);
                bool is_horizontal = (DZ[i] == 0);
                bool wrong_z       = (DZ[i] * z_dir < 0);
                double ng = cur->g_score_ + base
                            * (is_horizontal ? H_PEN : (wrong_z ? Z_BACK_PEN : 1.0));
                if (ng > topo_max_range_ * 2.0) continue;

                int ni = gid(np);
                auto it = seen.find(ni);
                if (it != seen.end() && it->second->g_score_ <= ng) continue;

                auto nn = make_shared<sch_node>();
                nn->pos_ = np; nn->g_score_ = ng;
                nn->parent_ = cur; nn->status_ = not_expand;
                seen[ni] = nn;
                open.push(nn);
            }
        }
        return false;
    };

    // Convert vector path → run PrunePath → return pruned vector
    auto prune_path = [&](const std::vector<Eigen::Vector3d>& raw)
            -> std::vector<Eigen::Vector3d> {
        if (raw.size() <= 2) return raw;
        std::list<Eigen::Vector3d> in(raw.begin(), raw.end());
        std::list<Eigen::Vector3d> out;
        double len = 0.0;
        if (PrunePath(in, out, len) && out.size() >= 2)
            return std::vector<Eigen::Vector3d>(out.begin(), out.end());
        return raw;
    };

    // ── Main BFS over TopoNodes ───────────────────────────────────────────
    std::unordered_set<uint32_t> vis_ground, vis_aero;
    queue<uint32_t> ground_q, aerial_q;

    // Seed: first GROUND TopoNode
    for (auto& [tid, tn] : topo_nodes_) {
        if (tn.tag_ == GROUND) { ground_q.push(tid); vis_ground.insert(tid); break; }
    }

    int edge_cnt = 0;

    while (!ground_q.empty() || !aerial_q.empty()) {

        // ① GROUND expansion
        if (!ground_q.empty()) {
            uint32_t cid = ground_q.front(); ground_q.pop();
            Eigen::Vector3i c3i; PostoId3(topo_nodes_[cid].pos_, c3i);

            std::unordered_map<uint32_t, shared_ptr<sch_node>> hits;
            tag_dijkstra(c3i, GROUND, hits);

            for (auto& [tid, sn] : hits) {
                if (tid == cid) continue;
                if (!vis_ground.count(tid)) { vis_ground.insert(tid); ground_q.push(tid); }
                auto path = prune_path(extract_path(sn));
                add_edge(cid, tid, (float)sn->g_score_, EDGE_GROUND, false, path);
                edge_cnt++;
            }

            // Cross search upward → first unvisited AERO TopoNode
            uint32_t fid; std::vector<Eigen::Vector3d> cpath;
            if (cross_search(c3i, AERO, +1, vis_aero, fid, cpath)) {
                if (!vis_aero.count(fid)) { vis_aero.insert(fid); aerial_q.push(fid); }
                float cost = (float)(topo_nodes_[fid].pos_ - topo_nodes_[cid].pos_).norm();
                add_edge(cid, fid, cost, EDGE_CROSS, true, prune_path(cpath));
                edge_cnt++;
            }
        }

        // ② AERO expansion (symmetric)
        if (!aerial_q.empty()) {
            uint32_t cid = aerial_q.front(); aerial_q.pop();
            Eigen::Vector3i c3i; PostoId3(topo_nodes_[cid].pos_, c3i);

            std::unordered_map<uint32_t, shared_ptr<sch_node>> hits;
            tag_dijkstra(c3i, AERO, hits);

            for (auto& [tid, sn] : hits) {
                if (tid == cid) continue;
                if (!vis_aero.count(tid)) { vis_aero.insert(tid); aerial_q.push(tid); }
                auto path = prune_path(extract_path(sn));
                add_edge(cid, tid, (float)sn->g_score_, EDGE_AERIAL, false, path);
                edge_cnt++;
            }

            // Cross search downward → first unvisited GROUND TopoNode
            uint32_t fid; std::vector<Eigen::Vector3d> cpath;
            if (cross_search(c3i, GROUND, -1, vis_ground, fid, cpath)) {
                if (!vis_ground.count(fid)) { vis_ground.insert(fid); ground_q.push(fid); }
                float cost = (float)(topo_nodes_[fid].pos_ - topo_nodes_[cid].pos_).norm();
                add_edge(cid, fid, cost, EDGE_CROSS, true, prune_path(cpath));
                edge_cnt++;
            }
        }
    }

    // Mark nodes with no edges as non-reachable
    non_reachable_.clear();
    for (auto& [tid, tn] : topo_nodes_)
        if (topo_adj_[tid].empty()) non_reachable_.insert(tid);

    RCLCPP_INFO(rclcpp::get_logger("lowres_map"),
        "OfflineBuildTopoModAware: topo_nodes=%zu edges=%d non_reachable=%zu",
        topo_nodes_.size(), edge_cnt, non_reachable_.size());
}

void LowResMap::CollectLowResTags(std::vector<Eigen::Vector3d> &ground,
                                   std::vector<Eigen::Vector3d> &aero,
                                   std::vector<Eigen::Vector3d> &trans) {
    ground.clear(); aero.clear(); trans.clear();

    for (int b = 0; b < (int)gridBLK_.size(); b++) {
        if (gridBLK_[b] == nullptr) continue;
        auto &blk = gridBLK_[b];
        int n_total = blk->block_size_.x() * blk->block_size_.y() * blk->block_size_.z();
        for (int n = 0; n < n_total; n++) {
            auto &node = blk->local_grid_[n];
            if (node == nullptr || node == Outnode_ || node == Expandnode_) continue;
            if (node->flags_[0] || !node->flags_[1]) continue;

            int bsxy = blk->block_size_.x() * blk->block_size_.y();
            int iz = n / bsxy;
            int iy = (n % bsxy) / blk->block_size_.x();
            int ix = n % blk->block_size_.x();
            Eigen::Vector3i node3i = blk->origin_ + Eigen::Vector3i(ix, iy, iz);
            Eigen::Vector3d pos    = IdtoPos(node3i);

            switch (node->tag_) {
                case GROUND: ground.push_back(pos); break;
                case AERO:   aero.push_back(pos);   break;
                case TRANS:  trans.push_back(pos);  break;
                default: break;
            }
        }
    }
}

uint32_t LowResMap::InsertQueryPoint(const Eigen::Vector3d& pos, T_TAG tag,
                                      std::string* err_msg) {
    auto fail = [&](const std::string& reason) -> uint32_t {
        if (err_msg) *err_msg = reason;
        RCLCPP_WARN(rclcpp::get_logger("lowres_map"), "InsertQueryPoint: %s", reason.c_str());
        return UINT32_MAX;
    };

    auto tag_name = [](T_TAG t) -> const char* {
        switch (t) { case GROUND: return "GROUND"; case AERO: return "AERO";
                     case TRANS:  return "TRANS";  default:   return "UNKNOWN"; }
    };

    // 1. Check position is inside map
    if (!InsideMap(pos))
        return fail("pos (" + std::to_string(pos.x()) + "," + std::to_string(pos.y()) +
                    "," + std::to_string(pos.z()) + ") outside map");

    // 2. Snap to nearest LR node index
    Eigen::Vector3i p3i;
    PostoId3(pos, p3i);

    // 3. Shared helpers (same as OfflineBuildTopoModAware)
    auto gid = [&](const Eigen::Vector3i& p) -> int {
        return p.z()*v_n_(1) + p.y()*voxel_num_(0) + p.x();
    };
    const int DX[6] = {1,-1,0,0,0,0};
    const int DY[6] = {0,0,1,-1,0,0};
    const int DZ[6] = {0,0,0,0,1,-1};

    auto extract_path = [&](const shared_ptr<sch_node>& leaf) {
        std::vector<Eigen::Vector3d> path;
        for (auto cur = leaf; cur; cur = cur->parent_)
            path.push_back(IdtoPos(cur->pos_));
        std::reverse(path.begin(), path.end());
        return path;
    };

    auto prune_path_fn = [&](const std::vector<Eigen::Vector3d>& raw)
            -> std::vector<Eigen::Vector3d> {
        if (raw.size() <= 2) return raw;
        std::list<Eigen::Vector3d> in(raw.begin(), raw.end());
        std::list<Eigen::Vector3d> out;
        double len = 0.0;
        if (PrunePath(in, out, len) && out.size() >= 2)
            return std::vector<Eigen::Vector3d>(out.begin(), out.end());
        return raw;
    };

    auto add_edge_fn = [&](uint32_t a, uint32_t b, float cost,
                            EdgeTag etag, bool gate,
                            const std::vector<Eigen::Vector3d>& path_ab) {
        for (auto& e : topo_adj_[a]) if (e.to_ == b) return;
        std::vector<Eigen::Vector3d> path_ba(path_ab.rbegin(), path_ab.rend());
        topo_adj_[a].push_back({a, b, cost, etag, gate, path_ab});
        topo_adj_[b].push_back({b, a, cost, etag, gate, path_ba});
    };

    // 4. Try to register a candidate index and connect it to the topo graph.
    //    Returns qid on success, UINT32_MAX if no topo neighbors found (with rollback).
    T_TAG   tag_filter = (tag == GROUND) ? GROUND : AERO;
    EdgeTag etag       = (tag == GROUND) ? EDGE_GROUND : EDGE_AERIAL;

    auto try_connect = [&](const Eigen::Vector3i& cand_idx) -> uint32_t {
        Eigen::Vector3d snapped = IdtoPos(cand_idx);
        int lid = PostoId(snapped);

        // Return existing TopoNode immediately
        auto it = node_topo_id_.find(lid);
        if (it != node_topo_id_.end()) {
            if (err_msg) err_msg->clear();
            RCLCPP_INFO(rclcpp::get_logger("lowres_map"),
                "InsertQueryPoint: reused existing topo id=%u at (%.2f,%.2f,%.2f)",
                it->second, snapped.x(), snapped.y(), snapped.z());
            return it->second;
        }

        // Register new node
        uint32_t qid = next_topo_id_++;
        topo_nodes_[qid] = {qid, snapped, tag};
        topo_adj_[qid]   = {};
        node_topo_id_[lid] = qid;

        // Dijkstra to find same-tag neighbors
        prio_D open;
        std::unordered_map<int, shared_ptr<sch_node>> seen;
        std::unordered_map<uint32_t, shared_ptr<sch_node>> hits;

        auto s = make_shared<sch_node>();
        s->pos_ = cand_idx; s->g_score_ = 0.0;
        s->parent_ = nullptr; s->status_ = not_expand;
        seen[gid(cand_idx)] = s;
        open.push(s);

        while (!open.empty()) {
            auto cur = open.top(); open.pop();
            if (cur->status_ == in_close) continue;
            cur->status_ = in_close;

            auto tit = node_topo_id_.find(PostoId(IdtoPos(cur->pos_)));
            if (tit != node_topo_id_.end() && tit->second != qid)
                hits[tit->second] = cur;

            for (int i = 0; i < 6; i++) {
                Eigen::Vector3i np = cur->pos_ + Eigen::Vector3i(DX[i], DY[i], DZ[i]);
                if (!InsideMap(np)) continue;
                auto lrn = GetNode(IdtoPos(np));
                if (!lrn || lrn->flags_[0]) continue;
                bool passable = (lrn->tag_ == tag_filter)
                             || (tag_filter == AERO && lrn->tag_ == TRANS);
                if (!passable) continue;

                double ng = cur->g_score_ + GetDist(DX[i], DY[i], DZ[i]);
                if (ng > topo_max_range_) continue;

                int ni = gid(np);
                auto it2 = seen.find(ni);
                if (it2 != seen.end() && it2->second->g_score_ <= ng) continue;

                auto nn = make_shared<sch_node>();
                nn->pos_ = np; nn->g_score_ = ng;
                nn->parent_ = cur; nn->status_ = not_expand;
                seen[ni] = nn;
                open.push(nn);
            }
        }

        if (hits.empty()) {
            // Rollback: this candidate is isolated, try the next one
            topo_nodes_.erase(qid);
            topo_adj_.erase(qid);
            node_topo_id_.erase(lid);
            return UINT32_MAX;
        }

        for (auto& [tid, sn] : hits)
            add_edge_fn(qid, tid, (float)sn->g_score_, etag, false,
                        prune_path_fn(extract_path(sn)));

        if (err_msg) err_msg->clear();
        RCLCPP_INFO(rclcpp::get_logger("lowres_map"),
            "InsertQueryPoint: id=%u tag=%s snapped=(%.2f,%.2f,%.2f) "
            "dist_to_query=%.2f neighbors=%zu",
            qid, tag_name(tag), snapped.x(), snapped.y(), snapped.z(),
            (IdtoPos(cand_idx) - pos).norm(), hits.size());
        return qid;
    };

    // 5. Expanding Chebyshev search: r = 0, 1, 2, ...
    //    XY capped at MAX_XY_R; Z uses full map height (no cap on |dz|).
    //    For each shell, first candidate with valid tag that connects → return immediately.
    constexpr int MAX_XY_R = 2;
    const int z_max_off = std::max(p3i.z(), voxel_num_.z() - 1 - p3i.z());
    const int max_r     = std::max(MAX_XY_R, z_max_off);

    int tried = 0;
    for (int r = 0; r <= max_r; r++) {
        for (int dz = -r; dz <= r; dz++) {
            for (int dy = -r; dy <= r; dy++) {
                for (int dx = -r; dx <= r; dx++) {
                    // Only process the shell surface (Chebyshev distance == r)
                    if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) continue;
                    // XY hard cap
                    if (std::abs(dx) > MAX_XY_R || std::abs(dy) > MAX_XY_R) continue;

                    Eigen::Vector3i cand(p3i.x()+dx, p3i.y()+dy, p3i.z()+dz);
                    if (!InsideMap(cand)) continue;
                    auto lrc = GetNode(IdtoPos(cand));
                    if (!lrc || lrc->flags_[0] || lrc->tag_ != tag) continue;

                    ++tried;
                    uint32_t qid = try_connect(cand);
                    if (qid != UINT32_MAX) return qid;
                    // else: node found but isolated → keep searching
                }
            }
        }
    }

    return fail(std::string("no connectable ") + tag_name(tag) +
                " node found after trying " + std::to_string(tried) +
                " candidates around (" +
                std::to_string(pos.x()) + "," + std::to_string(pos.y()) +
                "," + std::to_string(pos.z()) + ") XY±" +
                std::to_string(MAX_XY_R) + "/full-Z");
}

void LowResMap::RemoveQueryPoint(uint32_t node_id) {
    auto nit = topo_nodes_.find(node_id);
    if (nit == topo_nodes_.end()) return;

    // Remove back-edges from all neighbors
    auto ait = topo_adj_.find(node_id);
    if (ait != topo_adj_.end()) {
        for (auto& e : ait->second) {
            auto nb = topo_adj_.find(e.to_);
            if (nb == topo_adj_.end()) continue;
            auto& nv = nb->second;
            nv.erase(std::remove_if(nv.begin(), nv.end(),
                [&](const TopoEdge& te){ return te.to_ == node_id; }),
                nv.end());
        }
        topo_adj_.erase(ait);
    }

    int lid = PostoId(nit->second.pos_);
    node_topo_id_.erase(lid);
    topo_nodes_.erase(nit);
    non_reachable_.erase(node_id);
}


}
