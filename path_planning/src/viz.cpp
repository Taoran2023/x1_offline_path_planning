#include <path_planning/viz.h>

using namespace std::chrono_literals;

static geometry_msgs::msg::Point toGP(const Eigen::Vector3d &p) {
    geometry_msgs::msg::Point gp;
    gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
    return gp;
}

Viz::Viz(rclcpp::Node::SharedPtr node) : node_(node) {
    cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/viz/raw_cloud", rclcpp::QoS(1).transient_local());
    occ_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/viz/occupied_voxels", rclcpp::QoS(1).transient_local());
    free_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/viz/free_voxels", rclcpp::QoS(1).transient_local());
    trav_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/viz/traversability", rclcpp::QoS(1).transient_local());
    lr_tags_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/viz/lr_tags", rclcpp::QoS(1).transient_local());
    topo_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/viz/topo_nodes", rclcpp::QoS(1).transient_local());
    topo_graph_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/viz/topo_graph", rclcpp::QoS(1).transient_local());
    query_pts_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/viz/query_points", rclcpp::QoS(1).transient_local());

    const char* tree_topics[3] = {"/viz/tree_m4g1", "/viz/tree_g1", "/viz/tree_m4"};
    for (int i = 0; i < 3; i++) {
        tree_pubs_[i] = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            tree_topics[i], rclcpp::QoS(1).transient_local());
    }

    const char* path_topics[3] = {"/viz/path_shared", "/viz/path_g1", "/viz/path_m4"};
    for (int i = 0; i < 3; i++) {
        path_pubs_[i] = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            path_topics[i], rclcpp::QoS(1).transient_local());
    }
}

void Viz::PublishCloud(const sensor_msgs::msg::PointCloud2 &cloud) {
    cloud_pub_->publish(cloud);
}

void Viz::PublishOccupied(const std::vector<Eigen::Vector3d> &pts,
                          const std::vector<std_msgs::msg::ColorRGBA> &colors,
                          double vox_size) {
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = node_->now();
    del.ns = "occ";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    if (pts.empty()) { occ_pub_->publish(ma); return; }

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = node_->now();
    m.ns = "occ";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = vox_size;
    m.scale.y = vox_size;
    m.scale.z = vox_size;
    m.color.a = 1.0f;  // fallback; per-point colors will override
    m.pose.orientation.w = 1.0;

    m.points.reserve(pts.size());
    m.colors.reserve(colors.size());
    for (size_t i = 0; i < pts.size(); i++) {
        geometry_msgs::msg::Point gp;
        gp.x = pts[i].x(); gp.y = pts[i].y(); gp.z = pts[i].z();
        m.points.push_back(gp);
        m.colors.push_back(i < colors.size() ? colors[i] : del.color);
    }

    ma.markers.push_back(m);
    occ_pub_->publish(ma);
}

void Viz::PublishFree(const std::vector<Eigen::Vector3d> &pts, double vox_size) {
    free_pub_->publish(MakeVoxelMarkers(pts, vox_size * 0.5, 0.2f, 0.8f, 0.2f, 0.3f, "free"));
}

void Viz::PublishTraversability(const std::vector<Eigen::Vector3d> &pts,
                                const std::vector<float> &scores,
                                double vox_size) {
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = node_->now();
    del.ns = "trav";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    if (pts.empty()) { trav_pub_->publish(ma); return; }

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = node_->now();
    m.ns = "trav";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = vox_size;
    m.scale.y = vox_size;
    m.scale.z = vox_size;
    m.color.a = 0.7f;
    m.pose.orientation.w = 1.0;

    m.points.reserve(pts.size());
    m.colors.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); i++) {
        geometry_msgs::msg::Point gp;
        gp.x = pts[i].x(); gp.y = pts[i].y(); gp.z = pts[i].z();
        m.points.push_back(gp);

        float s = std::max(0.f, std::min(1.f, scores[i]));
        std_msgs::msg::ColorRGBA c;
        c.r = 1.0f - s;
        c.g = s;
        c.b = 0.0f;
        c.a = 0.85f;
        m.colors.push_back(c);
    }

    ma.markers.push_back(m);
    trav_pub_->publish(ma);
}

void Viz::PublishLowResTags(const std::vector<Eigen::Vector3d> &ground,
                            const std::vector<Eigen::Vector3d> &aero,
                            const std::vector<Eigen::Vector3d> &trans,
                            double vox_size) {
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = node_->now();
    del.ns = "lr_tags";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    // ground=green, aero=blue, trans=yellow; each as a separate CUBE_LIST
    struct Layer { const std::vector<Eigen::Vector3d> *pts; float r,g,b,a; int id; };
    Layer layers[] = {
        {&ground, 0.9f, 0.9f, 0.1f, 1.0f, 0},
        {&aero,   0.1f, 0.4f, 0.9f, 1.0f, 1},
        {&trans,  0.9f, 0.1f, 0.1f, 1.0f, 2},
    };

    for (auto &lyr : layers) {
        if (lyr.pts->empty()) continue;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = "lr_tags";
        m.id = lyr.id;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = vox_size * 0.1;
        m.scale.y = vox_size * 0.1;
        m.scale.z = vox_size * 0.1;
        m.color.r = lyr.r; m.color.g = lyr.g; m.color.b = lyr.b; m.color.a = lyr.a;
        m.pose.orientation.w = 1.0;
        m.points.reserve(lyr.pts->size());
        for (const auto &p : *lyr.pts) {
            geometry_msgs::msg::Point gp;
            gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
            m.points.push_back(gp);
        }
        ma.markers.push_back(m);
    }
    lr_tags_pub_->publish(ma);
}

visualization_msgs::msg::MarkerArray Viz::MakeVoxelMarkers(
    const std::vector<Eigen::Vector3d> &pts,
    double vox_size,
    float r, float g, float b, float a,
    const std::string &ns)
{
    visualization_msgs::msg::MarkerArray ma;

    // Delete-all marker to clear stale markers from previous publishes
    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = node_->now();
    del.ns = ns;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    if (pts.empty()) return ma;

    // Use a single CUBE_LIST marker for efficiency
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = node_->now();
    m.ns = ns;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = vox_size;
    m.scale.y = vox_size;
    m.scale.z = vox_size;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    m.pose.orientation.w = 1.0;

    m.points.reserve(pts.size());
    for (const auto &p : pts) {
        geometry_msgs::msg::Point gp;
        gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
        m.points.push_back(gp);
    }

    ma.markers.push_back(m);
    return ma;
}

void Viz::PublishTopoGraph(const std::vector<Eigen::Vector3d> &ground,
                           const std::vector<Eigen::Vector3d> &aero,
                           const std::vector<VizEdge>         &edges,
                           double node_radius,
                           bool   debug_path)
{
    visualization_msgs::msg::MarkerArray ma;

    // --- delete-all ---
    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = node_->now();
    del.ns = "topo_graph";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    // --- nodes (same style as PublishTopoNodes) ---
    struct NodeLayer { const std::vector<Eigen::Vector3d> *pts; float r, g, b; int id; };
    NodeLayer node_layers[] = {
        {&ground, 1.0f, 0.5f, 0.0f, 0},  // orange
        {&aero,   0.0f, 0.9f, 0.9f, 1},  // cyan
    };
    for (auto &lyr : node_layers) {
        if (lyr.pts->empty()) continue;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = "topo_graph";
        m.id = lyr.id;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = node_radius * 2.0;
        m.scale.y = node_radius * 2.0;
        m.scale.z = node_radius * 2.0;
        m.color.r = lyr.r; m.color.g = lyr.g; m.color.b = lyr.b; m.color.a = 1.0f;
        m.pose.orientation.w = 1.0;
        m.points.reserve(lyr.pts->size());
        for (const auto &p : *lyr.pts) m.points.push_back(toGP(p));
        ma.markers.push_back(m);
    }

    // --- edges: 3 LINE_LIST markers (GROUND=orange, AERIAL=cyan, CROSS=magenta) ---
    struct EdgeLayer { float r, g, b; int id; };
    EdgeLayer edge_layers[] = {
        {1.0f, 0.5f, 0.0f, 2},  // GROUND
        {0.0f, 0.9f, 0.9f, 3},  // AERIAL
        {0.9f, 0.0f, 0.9f, 4},  // CROSS
    };
    visualization_msgs::msg::Marker edge_markers[3];
    for (int i = 0; i < 3; i++) {
        auto &m = edge_markers[i];
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = "topo_graph";
        m.id = edge_layers[i].id;
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.05;
        m.color.r = edge_layers[i].r;
        m.color.g = edge_layers[i].g;
        m.color.b = edge_layers[i].b;
        m.color.a = 0.8f;
        m.pose.orientation.w = 1.0;
    }

    for (const auto &e : edges) {
        int idx = std::min(e.tag, 2);
        auto &m = edge_markers[idx];

        if (debug_path && e.path.size() >= 2) {
            // LINE_LIST draws segments: push pairs of consecutive waypoints
            for (size_t i = 0; i + 1 < e.path.size(); i++) {
                m.points.push_back(toGP(e.path[i]));
                m.points.push_back(toGP(e.path[i + 1]));
            }
        } else if (e.path.size() >= 2) {
            // Direct line: only start and end
            m.points.push_back(toGP(e.path.front()));
            m.points.push_back(toGP(e.path.back()));
        }
    }

    for (auto &m : edge_markers) {
        if (!m.points.empty()) ma.markers.push_back(m);
    }

    topo_graph_pub_->publish(ma);
}

void Viz::PublishTopoNodes(const std::vector<Eigen::Vector3d> &ground,
                           const std::vector<Eigen::Vector3d> &aero,
                           double radius) {
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = node_->now();
    del.ns = "topo";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    struct Layer { const std::vector<Eigen::Vector3d> *pts; float r, g, b; int id; };
    Layer layers[] = {
        {&ground, 1.0f, 0.5f, 0.0f, 0},  // orange
        {&aero,   0.0f, 0.9f, 0.9f, 1},  // cyan
    };

    for (auto &lyr : layers) {
        if (lyr.pts->empty()) continue;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = "topo";
        m.id = lyr.id;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = radius * 2.0;
        m.scale.y = radius * 2.0;
        m.scale.z = radius * 2.0;
        m.color.r = lyr.r; m.color.g = lyr.g; m.color.b = lyr.b; m.color.a = 1.0f;
        m.pose.orientation.w = 1.0;
        m.points.reserve(lyr.pts->size());
        for (const auto &p : *lyr.pts) {
            geometry_msgs::msg::Point gp;
            gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
            m.points.push_back(gp);
        }
        ma.markers.push_back(m);
    }
    topo_pub_->publish(ma);
}

void Viz::PublishSearchTree(SearchTreeId id,
                            const std::vector<Eigen::Vector3d> &nodes,
                            const std::vector<TreeEdge> &edges,
                            double node_radius)
{
    // Base color per tree
    static const float kColors[3][3] = {
        {0.9f, 0.9f, 0.9f},  // TREE_M4G1: white
        {0.2f, 0.9f, 0.2f},  // TREE_G1:   green
        {0.3f, 0.5f, 1.0f},  // TREE_M4:   blue (ground layer)
    };
    static const char* kNs[3] = {"tree_m4g1", "tree_g1", "tree_m4"};

    float r = kColors[id][0], g = kColors[id][1], b = kColors[id][2];
    const char* ns = kNs[id];

    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = node_->now();
    del.ns = ns;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    // settled nodes as SPHERE_LIST
    if (!nodes.empty()) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = ns; m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = node_radius * 2.0;
        m.scale.y = node_radius * 2.0;
        m.scale.z = node_radius * 2.0;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.7f;
        m.pose.orientation.w = 1.0;
        for (const auto &p : nodes) m.points.push_back(toGP(p));
        ma.markers.push_back(m);
    }

    if (!edges.empty()) {
        // GROUND edges: base color, width 0.15
        // AERIAL edges: cyan (0.0, 0.9, 1.0), width 0.20 (only appears for TREE_M4)
        // CROSS edges:  magenta, width 0.18
        struct EdgeLayer { float r,g,b,a; float width; int marker_id; };
        EdgeLayer elayers[3] = {
            {r,    g,    b,    0.55f, 0.15f, 1},  // EDGE_GROUND: tree's own color
            {0.0f, 0.9f, 1.0f, 0.8f, 0.20f, 2},  // EDGE_AERIAL: cyan, thicker
            {0.9f, 0.0f, 0.9f, 0.8f, 0.18f, 3},  // EDGE_CROSS:  magenta
        };

        visualization_msgs::msg::Marker ems[3];
        for (int i = 0; i < 3; i++) {
            auto &m = ems[i];
            m.header.frame_id = "map";
            m.header.stamp = node_->now();
            m.ns = ns; m.id = elayers[i].marker_id;
            m.type = visualization_msgs::msg::Marker::LINE_LIST;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = elayers[i].width;
            m.color.r = elayers[i].r; m.color.g = elayers[i].g;
            m.color.b = elayers[i].b; m.color.a = elayers[i].a;
            m.pose.orientation.w = 1.0;
        }

        for (const auto &e : edges) {
            int idx = std::min(e.tag, 2);
            ems[idx].points.push_back(toGP(e.from));
            ems[idx].points.push_back(toGP(e.to));
        }

        for (auto &m : ems) {
            if (!m.points.empty()) ma.markers.push_back(m);
        }
    }

    tree_pubs_[id]->publish(ma);
}

void Viz::PublishPath(PathId id,
                      const path_planning::AgentPath &path,
                      double node_radius)
{
    using namespace path_planning;

    static const float kColors[3][3] = {
        {0.9f, 0.9f, 0.9f},  // PATH_SHARED: white
        {0.2f, 0.9f, 0.2f},  // PATH_G1:     green
        {0.3f, 0.5f, 1.0f},  // PATH_M4:     blue
    };
    static const char* kNs[3] = {"path_shared", "path_g1", "path_m4"};

    float r = kColors[id][0], g = kColors[id][1], b = kColors[id][2];
    const char* ns = kNs[id];

    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = node_->now();
    del.ns = ns;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    if (path.empty()) { path_pubs_[id]->publish(ma); return; }

    // boundary nodes (start + each segment endpoint) as SPHERE_LIST
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = ns; m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = node_radius * 2.0;
        m.scale.y = node_radius * 2.0;
        m.scale.z = node_radius * 2.0;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
        m.pose.orientation.w = 1.0;

        if (!path.front().second.empty())
            m.points.push_back(toGP(path.front().second.front()));
        for (const auto &seg : path)
            if (!seg.second.empty())
                m.points.push_back(toGP(seg.second.back()));

        if (!m.points.empty()) ma.markers.push_back(m);
    }

    // segments as LINE_LIST, colored by TrajSegTag (same scheme as search tree)
    struct SegLayer { float r,g,b,a; float width; int marker_id; };
    SegLayer slayers[3] = {
        {r,    g,    b,    0.9f, 0.25f, 1},  // SEG_GROUND: own color
        {0.0f, 0.9f, 1.0f, 1.0f, 0.30f, 2},  // SEG_AERIAL: cyan
        {0.9f, 0.0f, 0.9f, 1.0f, 0.28f, 3},  // SEG_TRANS:  magenta
    };

    visualization_msgs::msg::Marker ems[3];
    for (int i = 0; i < 3; i++) {
        auto &m = ems[i];
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = ns; m.id = slayers[i].marker_id;
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = slayers[i].width;
        m.color.r = slayers[i].r; m.color.g = slayers[i].g;
        m.color.b = slayers[i].b; m.color.a = slayers[i].a;
        m.pose.orientation.w = 1.0;
    }

    for (const auto &seg : path) {
        int idx = static_cast<int>(seg.first);
        if (idx < 0 || idx > 2) idx = 0;
        auto &m = ems[idx];
        const auto &pts = seg.second;
        for (size_t k = 0; k + 1 < pts.size(); k++) {
            m.points.push_back(toGP(pts[k]));
            m.points.push_back(toGP(pts[k + 1]));
        }
    }

    for (auto &m : ems)
        if (!m.points.empty()) ma.markers.push_back(m);

    path_pubs_[id]->publish(ma);
}

void Viz::PublishQueryPoints(const std::vector<Eigen::Vector3d> &pts, double radius)
{
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = node_->now();
    del.ns = "query_points";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    if (pts.empty()) { query_pts_pub_->publish(ma); return; }

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = node_->now();
    m.ns = "query_points";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = radius * 2.0;
    m.scale.y = radius * 2.0;
    m.scale.z = radius * 2.0;
    m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 1.0f;
    m.pose.orientation.w = 1.0;
    m.points.reserve(pts.size());
    for (const auto &p : pts) m.points.push_back(toGP(p));
    ma.markers.push_back(m);

    query_pts_pub_->publish(ma);
}
