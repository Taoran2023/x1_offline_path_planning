#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <path_planning/traj_types.h>
#include <Eigen/Core>
#include <vector>
#include <string>

class Viz {
public:
    explicit Viz(rclcpp::Node::SharedPtr node);

    // Publish raw PCL as PointCloud2 on /viz/raw_cloud
    void PublishCloud(const sensor_msgs::msg::PointCloud2 &cloud);

    // Publish occupied voxels as cube markers on /viz/occupied_voxels, colored by height
    void PublishOccupied(const std::vector<Eigen::Vector3d> &pts,
                         const std::vector<std_msgs::msg::ColorRGBA> &colors,
                         double vox_size);

    // Publish free voxels as smaller cube markers on /viz/free_voxels
    void PublishFree(const std::vector<Eigen::Vector3d> &pts, double vox_size);

    // Publish surface voxels colored by traversability score (0=red, 1=green)
    void PublishTraversability(const std::vector<Eigen::Vector3d> &pts,
                               const std::vector<float> &scores,
                               double vox_size);

    // Publish LowRes tag layers: GROUND=green, AERO=blue, TRANS=yellow
    void PublishLowResTags(const std::vector<Eigen::Vector3d> &ground,
                           const std::vector<Eigen::Vector3d> &aero,
                           const std::vector<Eigen::Vector3d> &trans,
                           double vox_size);

    // Publish TopoNodes as spheres: GROUND=orange, AERO=cyan
    void PublishTopoNodes(const std::vector<Eigen::Vector3d> &ground,
                          const std::vector<Eigen::Vector3d> &aero,
                          double radius);

    // Edge descriptor (tag: 0=GROUND, 1=AERIAL, 2=CROSS)
    struct VizEdge {
        std::vector<Eigen::Vector3d> path;  // includes start and end nodes
        int  tag;
        bool is_gate;
    };

    // Publish topo graph: nodes as spheres + edges as lines on /viz/topo_graph
    // debug_path=true: draw stored path_ waypoints; false: direct start→end line
    void PublishTopoGraph(const std::vector<Eigen::Vector3d> &ground,
                          const std::vector<Eigen::Vector3d> &aero,
                          const std::vector<VizEdge>         &edges,
                          double node_radius,
                          bool   debug_path);

    // Publish query points (start/goal) as red spheres on /viz/query_points
    // radius: slightly larger than topo node radius (caller passes node_radius * 1.5)
    void PublishQueryPoints(const std::vector<Eigen::Vector3d> &pts,
                            double radius);

    // Tree id for PublishSearchTree
    enum SearchTreeId { TREE_M4G1 = 0, TREE_G1 = 1, TREE_M4 = 2 };

    // Publish one Dijkstra search tree as nodes (spheres) + parent→child edges (lines).
    // Topics: /viz/tree_m4g1, /viz/tree_g1, /viz/tree_m4
    // Colors: TREE_M4G1=white, TREE_G1=green, TREE_M4=blue
    struct TreeEdge {
        Eigen::Vector3d from, to;
        int tag;  // 0=GROUND, 1=AERIAL, 2=CROSS
    };

    void PublishSearchTree(SearchTreeId id,
                           const std::vector<Eigen::Vector3d> &nodes,
                           const std::vector<TreeEdge> &edges,
                           double node_radius = 0.2);

    // Path id for PublishPath
    enum PathId { PATH_SHARED = 0, PATH_G1 = 1, PATH_M4 = 2 };

    // Publish recovered AgentPath as boundary nodes (spheres) + segments (LINE_LIST).
    // Topics: /viz/path_shared, /viz/path_g1, /viz/path_m4
    // Colors match corresponding trees; segments colored by TrajSegTag.
    void PublishPath(PathId id,
                     const path_planning::AgentPath &path,
                     double node_radius = 0.3);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occ_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr free_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trav_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lr_tags_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topo_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topo_graph_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr query_pts_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pubs_[3];
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pubs_[3];

    visualization_msgs::msg::MarkerArray MakeVoxelMarkers(
        const std::vector<Eigen::Vector3d> &pts,
        double vox_size,
        float r, float g, float b, float a,
        const std::string &ns);
};
