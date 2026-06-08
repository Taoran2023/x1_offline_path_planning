#pragma once
#include <lowres_map/lowres_map.h>
#include <path_planning/traj_types.h>
#include <Eigen/Core>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <limits>
#include <cstdint>

namespace path_planning {

// ── Agent ────────────────────────────────────────────────────────────────────

enum AgentMode { GROUND_ONLY, DUAL_MODE };

struct Agent {
    std::string name;
    AgentMode   mode           = GROUND_ONLY;
    float       cost_g         = 1.0f;  // ground edge cost multiplier
    float       cost_a         = 0.8f;  // aerial edge cost multiplier
    float       transform_cost = 0.0f;  // fixed cost added on EDGE_CROSS
};

inline Agent make_agent_m4g1() { return {"m4g1", GROUND_ONLY, 1.5f, 0.f,  0.f }; }
inline Agent make_agent_g1()   { return {"g1",   GROUND_ONLY, 1.0f, 0.f,  0.f }; }
inline Agent make_agent_m4()   { return {"m4",   DUAL_MODE,   1.2f, 0.8f, 5.0f}; }

// ── TreeSearch ───────────────────────────────────────────────────────────────

class TreeSearch {
public:
    // Bind the topo graph and set the root node.
    void init(lowres::LowResMap* lmap, Agent agent, uint32_t root_id);

    // Dijkstra from root_id, expanding until the priority queue is empty or
    // all nodes with cost ≤ energy_budget have been settled.
    void build_tree(float energy_budget = std::numeric_limits<float>::infinity());

    // Backtrack parent_ chain and return the node-id sequence [root … target].
    // Returns empty vector if target_id is not reachable.
    std::vector<uint32_t> recover_path(uint32_t target_id) const;

    bool  is_reachable(uint32_t node_id) const;
    float cost_to(uint32_t node_id) const;

    const std::unordered_set<uint32_t>& settled() const { return settled_; }
    size_t settled_count() const { return settled_.size(); }

    // Returns parent node id, or UINT32_MAX for root / unreachable node.
    uint32_t parent_of(uint32_t node_id) const;

private:
    // Returns false when this agent cannot traverse the edge at all.
    bool  edge_passable(const lowres::TopoEdge& e) const;

    // Edge traversal cost: accounts for agent mode, t_score penalty, transform_cost.
    float edge_cost(const lowres::TopoEdge& e) const;

    // Mean inverse-traversability penalty along edge.path_  (1.0 = flat ground).
    float avg_trav_penalty(const lowres::TopoEdge& e) const;

    lowres::LowResMap*                     lmap_    = nullptr;
    Agent                                  agent_;
    uint32_t                               root_id_ = 0;

    std::unordered_map<uint32_t, float>    cost_;     // min cost to each settled node
    std::unordered_map<uint32_t, uint32_t> parent_;   // shortest-path tree parent
    std::unordered_set<uint32_t>           settled_;
};

// ── Overlap / best-point ─────────────────────────────────────────────────────

// Nodes settled by all three trees, sorted ascending by combined cost.
std::vector<std::pair<uint32_t, float>>
findOverlap(const TreeSearch& m4g1_tree,
            const TreeSearch& g1_tree,
            const TreeSearch& m4_tree);

// Node id with minimum combined cost (returns UINT32_MAX if overlap is empty).
uint32_t findBestPoint(const std::vector<std::pair<uint32_t, float>>& overlap);

// ── Tree visualization data ──────────────────────────────────────────────────

struct TreeVizData {
    std::vector<Eigen::Vector3d> nodes;
    struct Edge {
        Eigen::Vector3d from, to;
        int tag;  // lowres::EDGE_GROUND=0, EDGE_AERIAL=1, EDGE_CROSS=2
    };
    std::vector<Edge> edges;
};

// Extract settled node positions and parent→child edges for visualization.
TreeVizData ExtractTreeViz(const TreeSearch& tree, lowres::LowResMap* lmap);

// Stitch node-id sequence (from recover_path) into typed waypoint segments,
// merging consecutive same-tag edges and applying PrunePath per group.
AgentPath StitchAndPrune(const std::vector<uint32_t>& node_ids, lowres::LowResMap* lmap,
                         double stitch_prune_len = 10.0);

} // namespace path_planning
