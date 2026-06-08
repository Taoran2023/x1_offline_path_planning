#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <block_map/block_map.h>
#include <lowres_map/lowres_map.h>
#include <path_planning/viz.h>
#include <path_planning/topo_planner.h>
#include <block_map/mapping_struct.h>

using namespace std::chrono_literals;

class TestMapNode : public rclcpp::Node {
public:
    explicit TestMapNode(const std::string &pcd_path)
        : Node("test_map"), pcd_path_(pcd_path) {}

    void initialize() {
        // ── Visualization debug ──────────────────────────────────────
        debug_path_ = declare_parameter<bool>("viz.debug_path", false);

        // ── Planning targets ─────────────────────────────────────────
        auto tv1 = declare_parameter<std::vector<double>>("target1", {0.0, 0.0, 0.0});
        auto tv2 = declare_parameter<std::vector<double>>("target2", {5.0, 5.0, 0.0});
        auto tv3 = declare_parameter<std::vector<double>>("target3", {10.0, 0.0, 2.0});
        target_pos_[0] = Eigen::Vector3d(tv1[0], tv1[1], tv1[2]);
        target_pos_[1] = Eigen::Vector3d(tv2[0], tv2[1], tv2[2]);
        target_pos_[2] = Eigen::Vector3d(tv3[0], tv3[1], tv3[2]);

        // ── PCD preprocessing ────────────────────────────────────────
        pcd_yaw_deg_ = declare_parameter<double>("pcd.yaw_deg", 0.0);

        // ── Step-by-step debug switches (all enabled by default) ─────
        // Dependency chain: blockmap → voxels/traversability/lowres → targets/lrtags/topograph
        dbg_raw_cloud_    = declare_parameter<bool>("dbg.raw_cloud",    true);
        dbg_blockmap_     = declare_parameter<bool>("dbg.blockmap",     true);
        dbg_patch_trav_   = declare_parameter<bool>("dbg.patch_trav",   true);
        patch_trav_radius_= declare_parameter<int> ("patch_trav.fill_radius", 1);
        dbg_voxels_       = declare_parameter<bool>("dbg.voxels",       true);
        dbg_traversability_ = declare_parameter<bool>("dbg.traversability", true);
        dbg_lowres_       = declare_parameter<bool>("dbg.lowres",       true);
        dbg_targets_      = declare_parameter<bool>("dbg.targets",      true);
        dbg_lrtags_       = declare_parameter<bool>("dbg.lrtags",       true);
        dbg_topograph_    = declare_parameter<bool>("dbg.topograph",    true);
        dbg_plantest_     = declare_parameter<bool>("dbg.plantest",     true);

        // ── Path stitching prune window ──────────────────────────────
        stitch_prune_len_ = declare_parameter<double>("plan.stitch_prune_length", 10.0);

        // ── Agent cost parameters ────────────────────────────────────
        agent_m4g1_cost_g_  = declare_parameter<double>("agent.m4g1.cost_g",       1.5);
        agent_g1_cost_g_    = declare_parameter<double>("agent.g1.cost_g",         1.0);
        agent_m4_cost_g_    = declare_parameter<double>("agent.m4.cost_g",         1.2);
        agent_m4_cost_a_    = declare_parameter<double>("agent.m4.cost_a",         0.8);
        agent_m4_transform_ = declare_parameter<double>("agent.m4.transform_cost", 5.0);

        bmap_ = std::make_shared<BlockMap>();
        bmap_->init(shared_from_this());

        lmap_ = std::make_shared<lowres::LowResMap>();
        lmap_->SetMap(bmap_.get());
        lmap_->init(shared_from_this());

        viz_ = std::make_unique<Viz>(shared_from_this());

        load_timer_ = create_wall_timer(500ms, [this]() {
            load_timer_->cancel();
            LoadAndInsert();
            pub_timer_ = create_wall_timer(5s, [this]() { Republish(); });
        });
    }

private:
    // ── Step label (unified log format) ──────────────────────────────
    void StepSkip(const char* name) {
        RCLCPP_INFO(get_logger(), "[SKIP] %s", name);
    }

    // ── Phase 2b/2c: build three search trees + solve separation point ──
    void RunPlanTest() {
        uint32_t start_id   = target_ids_[0];  // target1: shared start
        uint32_t goal_g1_id = target_ids_[1];  // target2: G1 goal
        uint32_t goal_m4_id = target_ids_[2];  // target3: M4 goal

        if (start_id==UINT32_MAX || goal_g1_id==UINT32_MAX || goal_m4_id==UINT32_MAX) {
            RCLCPP_WARN(get_logger(), "[plantest] skipped: a target failed to insert");
            return;
        }

        using namespace path_planning;
        Agent agent_m4g1{"m4g1", GROUND_ONLY, (float)agent_m4g1_cost_g_, 0.f, 0.f};
        Agent agent_g1  {"g1",   GROUND_ONLY, (float)agent_g1_cost_g_,   0.f, 0.f};
        Agent agent_m4  {"m4",   DUAL_MODE,   (float)agent_m4_cost_g_,
                                              (float)agent_m4_cost_a_,
                                              (float)agent_m4_transform_};
        RCLCPP_INFO(get_logger(),
            "[plantest] agents: m4g1(cg=%.2f) g1(cg=%.2f) m4(cg=%.2f ca=%.2f tx=%.2f)",
            agent_m4g1.cost_g, agent_g1.cost_g,
            agent_m4.cost_g, agent_m4.cost_a, agent_m4.transform_cost);

        TreeSearch m4g1_tree, g1_tree, m4_tree;
        m4g1_tree.init(lmap_.get(), agent_m4g1, start_id);
        g1_tree  .init(lmap_.get(), agent_g1,   goal_g1_id);
        m4_tree  .init(lmap_.get(), agent_m4,   goal_m4_id);

        m4g1_tree.build_tree();
        g1_tree  .build_tree();
        m4_tree  .build_tree();

        RCLCPP_INFO(get_logger(),
            "[plantest] settled: m4g1=%zu  g1=%zu  m4=%zu",
            m4g1_tree.settled_count(), g1_tree.settled_count(), m4_tree.settled_count());

        // Cost to each goal
        RCLCPP_INFO(get_logger(),
            "[plantest] cost start→g1_goal(via g1_tree)=%.2f  start→m4_goal(via m4_tree)=%.2f",
            g1_tree.cost_to(start_id), m4_tree.cost_to(start_id));

        // Separation-point solve
        auto overlap = findOverlap(m4g1_tree, g1_tree, m4_tree);
        RCLCPP_INFO(get_logger(), "[plantest] overlap nodes: %zu", overlap.size());

        if (!overlap.empty()) {
            uint32_t sep = findBestPoint(overlap);
            const auto& sp = lmap_->topo_nodes_.at(sep).pos_;
            RCLCPP_INFO(get_logger(),
                "[plantest] best sep: id=%u pos=(%.2f,%.2f,%.2f) combined_cost=%.2f",
                sep, sp.x(), sp.y(), sp.z(), overlap[0].second);

            // Log path node counts
            auto shared_nodes = m4g1_tree.recover_path(sep);
            auto g1_nodes     = g1_tree.recover_path(sep);
            auto m4_nodes     = m4_tree.recover_path(sep);
            RCLCPP_INFO(get_logger(),
                "[plantest] path nodes: shared=%zu  g1_tail=%zu  m4_tail=%zu",
                shared_nodes.size(), g1_nodes.size(), m4_nodes.size());

            // Reverse g1/m4 from goal→sep to sep→goal for easier viz reading
            std::reverse(g1_nodes.begin(), g1_nodes.end());
            std::reverse(m4_nodes.begin(), m4_nodes.end());

            path_shared_ = StitchAndPrune(shared_nodes, lmap_.get(), stitch_prune_len_);
            path_g1_     = StitchAndPrune(g1_nodes,     lmap_.get(), stitch_prune_len_);
            path_m4_     = StitchAndPrune(m4_nodes,     lmap_.get(), stitch_prune_len_);
            path_data_ready_ = true;
            RCLCPP_INFO(get_logger(),
                "[plantest] path: shared=%zu segs  g1=%zu segs  m4=%zu segs",
                path_shared_.size(), path_g1_.size(), path_m4_.size());
        } else {
            RCLCPP_WARN(get_logger(), "[plantest] no overlap nodes — the three trees do not intersect; check map connectivity");
        }

        // Extract visualization data
        tree_m4g1_viz_ = ExtractTreeViz(m4g1_tree, lmap_.get());
        tree_g1_viz_   = ExtractTreeViz(g1_tree,   lmap_.get());
        tree_m4_viz_   = ExtractTreeViz(m4_tree,   lmap_.get());
        tree_data_ready_ = true;
        RCLCPP_INFO(get_logger(),
            "[plantest] tree viz: m4g1=%zu/%zu  g1=%zu/%zu  m4=%zu/%zu  (nodes/edges)",
            tree_m4g1_viz_.nodes.size(), tree_m4g1_viz_.edges.size(),
            tree_g1_viz_.nodes.size(),   tree_g1_viz_.edges.size(),
            tree_m4_viz_.nodes.size(),   tree_m4_viz_.edges.size());
    }

    void LoadAndInsert() {
        // ── Step 1+2: load PCD + downsample (always runs; prerequisite for all later steps) ──
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud) == -1) {
            RCLCPP_ERROR(get_logger(), "Failed to load PCD: %s", pcd_path_.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "[1] Loaded %zu points from %s",
                    cloud->size(), pcd_path_.c_str());

        if (pcd_yaw_deg_ != 0.0) {
            double yaw_rad = pcd_yaw_deg_ * M_PI / 180.0;
            float cy = static_cast<float>(std::cos(yaw_rad));
            float sy = static_cast<float>(std::sin(yaw_rad));
            for (auto& pt : cloud->points) {
                float nx = cy * pt.x - sy * pt.y;
                float ny = sy * pt.x + cy * pt.y;
                pt.x = nx;  pt.y = ny;
            }
            RCLCPP_INFO(get_logger(), "[1] PCD rotated %.1f deg around Z", pcd_yaw_deg_);
        }

        if (dbg_raw_cloud_) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(cloud);
            vg.setLeafSize(0.4f, 0.4f, 0.4f);
            vg.filter(*cloud_ds);
            pcl::toROSMsg(*cloud_ds, raw_cloud_);
            raw_cloud_.header.frame_id = "map";
            raw_cloud_.header.stamp = now();
            RCLCPP_INFO(get_logger(), "[2] Downsampled %zu → %zu pts for /viz/raw_cloud",
                        cloud->size(), cloud_ds->size());
        } else { StepSkip("raw_cloud (step 2)"); }

        // ── Step 3: BlockMap insertion + fill ────────────────────────
        if (dbg_blockmap_) {
            bmap_->OfflineInsertPcl(cloud);
            bmap_->OfflineFillUnknownFree();
            RCLCPP_INFO(get_logger(), "[3] BlockMap: %zu voxel centers",
                        bmap_->cur_pcl_.size());
        } else { StepSkip("blockmap (step 3) — voxels/lowres depend on this step"); }

        // ── Step 4: patch traversability in sparse regions ───────────
        if (dbg_blockmap_ && dbg_patch_trav_) {
            bmap_->PatchTraversabilityMap(patch_trav_radius_);
        } else { StepSkip("patch_trav (step 4)"); }

        // ── Step 5: collect occupied / free voxels ───────────────────
        if (dbg_voxels_) {
            bmap_->CollectVoxels(occ_pts_, occ_colors_, free_pts_);
            RCLCPP_INFO(get_logger(), "[5] Occupied: %zu  Free: %zu",
                        occ_pts_.size(), free_pts_.size());
        } else { StepSkip("voxels (step 5) → /viz/occupied_voxels /viz/free_voxels"); }

        // ── Step 6: collect traversability ────────────────────────────
        if (dbg_traversability_) {
            bmap_->CollectTraversability(trav_pts_, trav_scores_);
            RCLCPP_INFO(get_logger(), "[6] Traversability surface voxels: %zu",
                        trav_pts_.size());
        } else { StepSkip("traversability (step 6) → /viz/traversability"); }

        // ── Step 7: LowResMap init + topo graph construction ─────────
        if (dbg_lowres_) {
            lmap_->OfflineInitLowResMap();
            lmap_->OfflineBuildTopoModAware();
            RCLCPP_INFO(get_logger(), "[7] LowResMap ready: topo_nodes=%zu",
                        lmap_->topo_nodes_.size());
        } else { StepSkip("lowres (step 7) — targets/lrtags/topograph depend on this step"); }

        // ── Phase 2a: InsertQueryPoint for target1/2/3 ────────────────
        query_pts_.clear();
        target_ids_[0] = target_ids_[1] = target_ids_[2] = UINT32_MAX;
        if (dbg_targets_) {
            if (!dbg_lowres_)
                RCLCPP_WARN(get_logger(), "[targets] lowres is disabled — InsertQueryPoint may fail");
            const char* tnames[3] = {"target1", "target2", "target3"};
            for (int i = 0; i < 3; i++) {
                std::string err;
                uint32_t qid = lmap_->InsertQueryPoint(target_pos_[i], lowres::GROUND, &err);
                target_ids_[i] = qid;
                if (qid == UINT32_MAX) {
                    RCLCPP_WARN(get_logger(), "[targets] %s FAILED: %s", tnames[i], err.c_str());
                } else {
                    const auto& np = lmap_->topo_nodes_.at(qid).pos_;
                    query_pts_.push_back(np);
                    RCLCPP_INFO(get_logger(),
                        "[targets] %s: id=%u snapped=(%.2f,%.2f,%.2f) edges=%zu",
                        tnames[i], qid, np.x(), np.y(), np.z(),
                        lmap_->topo_adj_.at(qid).size());
                }
            }
        } else { StepSkip("targets → /viz/query_points"); }

        // ── Collect LowRes tags + TopoNodes ──────────────────────────
        lr_ground_.clear(); lr_aero_.clear(); lr_trans_.clear();
        if (dbg_lrtags_) {
            lmap_->CollectLowResTags(lr_ground_, lr_aero_, lr_trans_);
            RCLCPP_INFO(get_logger(), "[lrtags] ground=%zu aero=%zu trans=%zu",
                        lr_ground_.size(), lr_aero_.size(), lr_trans_.size());
        } else { StepSkip("lrtags → /viz/lr_tags"); }

        // ── Collect topo graph (nodes + edges) ───────────────────────
        topo_ground_.clear(); topo_aero_.clear(); topo_edges_.clear();
        if (dbg_topograph_) {
            for (const auto &[id, tn] : lmap_->topo_nodes_) {
                if (tn.tag_ == lowres::GROUND)      topo_ground_.push_back(tn.pos_);
                else if (tn.tag_ == lowres::AERO)   topo_aero_.push_back(tn.pos_);
            }
            for (const auto &[from_id, edge_list] : lmap_->topo_adj_) {
                for (const auto &e : edge_list) {
                    if (e.from_ < e.to_) {
                        Viz::VizEdge ve;
                        ve.path    = e.path_;
                        ve.tag     = static_cast<int>(e.edge_tag_);
                        ve.is_gate = e.is_gate_;
                        topo_edges_.push_back(std::move(ve));
                    }
                }
            }
            RCLCPP_INFO(get_logger(), "[topograph] nodes ground=%zu aero=%zu  edges=%zu",
                        topo_ground_.size(), topo_aero_.size(), topo_edges_.size());
        } else { StepSkip("topograph → /viz/topo_graph"); }

        // ── Phase 2b/2c: TreeSearch build + separation-point solve ───
        if (dbg_plantest_ && dbg_targets_ && query_pts_.size() == 3) {
            RunPlanTest();
        } else if (dbg_plantest_) {
            RCLCPP_WARN(get_logger(), "[plantest] requires dbg.targets=true and all three targets inserted successfully");
        }

        Republish();
        RCLCPP_INFO(get_logger(), "Map ready, republishing every 5s");
    }

    void Republish() {
        if (dbg_raw_cloud_)
            viz_->PublishCloud(raw_cloud_);
        if (dbg_voxels_) {
            viz_->PublishOccupied(occ_pts_, occ_colors_, bmap_->resolution_);
            viz_->PublishFree(free_pts_, bmap_->resolution_);
        }
        if (dbg_traversability_)
            viz_->PublishTraversability(trav_pts_, trav_scores_, bmap_->resolution_);
        if (dbg_lrtags_)
            viz_->PublishLowResTags(lr_ground_, lr_aero_, lr_trans_,
                                    lmap_->node_scale_.maxCoeff());
        if (dbg_topograph_)
            viz_->PublishTopoGraph(topo_ground_, topo_aero_, topo_edges_,
                                   lmap_->node_scale_.maxCoeff() * 0.3, debug_path_);
        if (dbg_targets_)
            viz_->PublishQueryPoints(query_pts_, lmap_->node_scale_.maxCoeff() * 0.45);
        if (dbg_plantest_ && tree_data_ready_) {
            double nr = lmap_->node_scale_.maxCoeff() * 0.2;
            auto conv_tree = [](const std::vector<path_planning::TreeVizData::Edge>& in) {
                std::vector<Viz::TreeEdge> out; out.reserve(in.size());
                for (const auto& e : in) out.push_back({e.from, e.to, e.tag});
                return out;
            };
            viz_->PublishSearchTree(Viz::TREE_M4G1, tree_m4g1_viz_.nodes, conv_tree(tree_m4g1_viz_.edges), nr);
            viz_->PublishSearchTree(Viz::TREE_G1,   tree_g1_viz_.nodes,   conv_tree(tree_g1_viz_.edges),   nr);
            viz_->PublishSearchTree(Viz::TREE_M4,   tree_m4_viz_.nodes,   conv_tree(tree_m4_viz_.edges),   nr);
        }
        if (dbg_plantest_ && path_data_ready_) {
            double nr = lmap_->node_scale_.maxCoeff() * 0.3;
            viz_->PublishPath(Viz::PATH_SHARED, path_shared_, nr);
            viz_->PublishPath(Viz::PATH_G1,     path_g1_,     nr);
            viz_->PublishPath(Viz::PATH_M4,     path_m4_,     nr);
        }
    }

    // ── Member variables ─────────────────────────────────────────────
    std::string pcd_path_;
    std::shared_ptr<BlockMap> bmap_;
    std::shared_ptr<lowres::LowResMap> lmap_;
    std::unique_ptr<Viz> viz_;
    rclcpp::TimerBase::SharedPtr load_timer_, pub_timer_;

    // Precomputed data (re-published directly by Republish)
    sensor_msgs::msg::PointCloud2 raw_cloud_;
    std::vector<Eigen::Vector3d>  occ_pts_, free_pts_, trav_pts_;
    std::vector<std_msgs::msg::ColorRGBA> occ_colors_;
    std::vector<float>            trav_scores_;
    std::vector<Eigen::Vector3d>  lr_ground_, lr_aero_, lr_trans_;
    std::vector<Eigen::Vector3d>  topo_ground_, topo_aero_;
    std::vector<Viz::VizEdge>     topo_edges_;
    std::vector<Eigen::Vector3d>  query_pts_;
    Eigen::Vector3d               target_pos_[3];
    uint32_t                      target_ids_[3] = {UINT32_MAX, UINT32_MAX, UINT32_MAX};

    // Visualization data for the three search trees
    path_planning::TreeVizData    tree_m4g1_viz_, tree_g1_viz_, tree_m4_viz_;
    bool                          tree_data_ready_ = false;

    // Recovered paths (StitchAndPrune output, used directly for viz and trajectory optimization)
    path_planning::AgentPath      path_shared_, path_g1_, path_m4_;
    bool                          path_data_ready_ = false;

    // PCD preprocessing
    double pcd_yaw_deg_     = 0.0;

    // Debug switches
    bool debug_path_        = false;
    bool dbg_raw_cloud_     = true;
    bool dbg_blockmap_      = true;
    bool dbg_patch_trav_    = true;
    int  patch_trav_radius_ = 1;
    bool dbg_voxels_        = true;
    bool dbg_traversability_= true;
    bool dbg_lowres_        = true;
    bool dbg_targets_       = true;
    bool dbg_lrtags_        = true;
    bool dbg_topograph_     = true;
    bool dbg_plantest_      = true;

    // Path stitching prune window
    double stitch_prune_len_   = 10.0;

    // Agent cost parameters
    double agent_m4g1_cost_g_  = 1.5;
    double agent_g1_cost_g_    = 1.0;
    double agent_m4_cost_g_    = 1.2;
    double agent_m4_cost_a_    = 0.8;
    double agent_m4_transform_ = 5.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("test_map"), "Usage: test_map <path/to/map.pcd>");
        return 1;
    }

    auto node = std::make_shared<TestMapNode>(argv[1]);
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
