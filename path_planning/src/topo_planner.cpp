#include <path_planning/topo_planner.h>
#include <queue>
#include <algorithm>
#include <cassert>
#include <cmath>

namespace path_planning {

// ── TreeSearch::init ──────────────────────────────────────────────────────────

void TreeSearch::init(lowres::LowResMap* lmap, Agent agent, uint32_t root_id) {
    lmap_    = lmap;
    agent_   = agent;
    root_id_ = root_id;
    cost_.clear();
    parent_.clear();
    settled_.clear();
}

// ── TreeSearch::build_tree ────────────────────────────────────────────────────

void TreeSearch::build_tree(float energy_budget) {
    assert(lmap_ && "TreeSearch::build_tree called before init()");

    // min-heap: (cost, node_id)
    using Entry = std::pair<float, uint32_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

    cost_[root_id_] = 0.0f;
    pq.push({0.0f, root_id_});

    while (!pq.empty()) {
        auto [cur_cost, uid] = pq.top();
        pq.pop();

        if (settled_.count(uid)) continue;
        if (cur_cost > energy_budget) break;

        settled_.insert(uid);

        auto it = lmap_->topo_adj_.find(uid);
        if (it == lmap_->topo_adj_.end()) continue;

        for (const auto& e : it->second) {
            if (!edge_passable(e)) continue;

            float new_cost = cur_cost + edge_cost(e);
            auto cost_it = cost_.find(e.to_);
            if (cost_it == cost_.end() || new_cost < cost_it->second) {
                cost_[e.to_]   = new_cost;
                parent_[e.to_] = uid;
                pq.push({new_cost, e.to_});
            }
        }
    }
}

// ── TreeSearch::recover_path ──────────────────────────────────────────────────

std::vector<uint32_t> TreeSearch::recover_path(uint32_t target_id) const {
    if (!is_reachable(target_id)) return {};

    std::vector<uint32_t> path;
    uint32_t cur = target_id;
    while (cur != root_id_) {
        path.push_back(cur);
        auto it = parent_.find(cur);
        if (it == parent_.end()) return {};  // should not happen for reachable nodes
        cur = it->second;
    }
    path.push_back(root_id_);
    std::reverse(path.begin(), path.end());
    return path;
}

bool TreeSearch::is_reachable(uint32_t node_id) const {
    return settled_.count(node_id) > 0;
}

float TreeSearch::cost_to(uint32_t node_id) const {
    auto it = cost_.find(node_id);
    return (it != cost_.end()) ? it->second : std::numeric_limits<float>::infinity();
}

// ── EdgePassable ──────────────────────────────────────────────────────────────

bool TreeSearch::edge_passable(const lowres::TopoEdge& e) const {
    if (agent_.mode == GROUND_ONLY) {
        // Ground-only agents cannot traverse aerial or cross (takeoff/landing) edges.
        return e.edge_tag_ == lowres::EDGE_GROUND;
    }
    // DUAL_MODE: all edge types are valid.
    return true;
}

// ── EdgeCost ─────────────────────────────────────────────────────────────────

float TreeSearch::edge_cost(const lowres::TopoEdge& e) const {
    switch (e.edge_tag_) {
    case lowres::EDGE_GROUND:
        return agent_.cost_g * e.cost_ * avg_trav_penalty(e);

    case lowres::EDGE_AERIAL:
        return agent_.cost_a * e.cost_;

    case lowres::EDGE_CROSS:
        // Cross edges are mode-transition; apply aerial cost + fixed transform penalty.
        return agent_.cost_a * e.cost_ + agent_.transform_cost;

    default:
        return e.cost_;
    }
}

// ── AvgTravPenalty ────────────────────────────────────────────────────────────

float TreeSearch::avg_trav_penalty(const lowres::TopoEdge& e) const {
    if (e.path_.empty()) return 1.0f;

    float sum = 0.0f;
    int   cnt = 0;
    for (const auto& pt : e.path_) {
        auto node = lmap_->GetNode(pt);
        if (node && node->t_score_ > 0.0f) {
            sum += node->t_score_;
            cnt++;
        }
    }
    if (cnt == 0) return 1.0f;
    float avg_score = sum / cnt;
    // Invert: flat ground (score=1) → penalty 1.0; rough (score=0.1) → penalty 10.0
    return 1.0f / std::max(0.05f, avg_score);
}

// ── findOverlap ───────────────────────────────────────────────────────────────

std::vector<std::pair<uint32_t, float>>
findOverlap(const TreeSearch& m4g1_tree,
            const TreeSearch& g1_tree,
            const TreeSearch& m4_tree) {
    std::vector<std::pair<uint32_t, float>> result;

    for (uint32_t id : m4g1_tree.settled()) {
        if (!g1_tree.is_reachable(id)) continue;
        if (!m4_tree.is_reachable(id)) continue;

        float combined = m4g1_tree.cost_to(id)
                       + g1_tree.cost_to(id)
                       + m4_tree.cost_to(id);
        result.push_back({id, combined});
    }

    std::sort(result.begin(), result.end(),
              [](const auto& a, const auto& b){ return a.second < b.second; });
    return result;
}

// ── findBestPoint ─────────────────────────────────────────────────────────────

uint32_t findBestPoint(const std::vector<std::pair<uint32_t, float>>& overlap) {
    if (overlap.empty()) return UINT32_MAX;
    return overlap.front().first;
}

// ── parent_of ─────────────────────────────────────────────────────────────────

uint32_t TreeSearch::parent_of(uint32_t node_id) const {
    auto it = parent_.find(node_id);
    return (it != parent_.end()) ? it->second : UINT32_MAX;
}

// ── ExtractTreeViz ────────────────────────────────────────────────────────────

TreeVizData ExtractTreeViz(const TreeSearch& tree, lowres::LowResMap* lmap) {
    TreeVizData result;
    result.nodes.reserve(tree.settled_count());
    result.edges.reserve(tree.settled_count());

    for (uint32_t id : tree.settled()) {
        auto nit = lmap->topo_nodes_.find(id);
        if (nit == lmap->topo_nodes_.end()) continue;
        result.nodes.push_back(nit->second.pos_);

        uint32_t pid = tree.parent_of(id);
        if (pid == UINT32_MAX) continue;
        auto pit = lmap->topo_nodes_.find(pid);
        if (pit == lmap->topo_nodes_.end()) continue;

        // Look up the edge_tag of the corresponding edge in topo_adj_
        int tag = 0;  // default EDGE_GROUND
        auto adj_it = lmap->topo_adj_.find(pid);
        if (adj_it != lmap->topo_adj_.end()) {
            for (const auto& e : adj_it->second) {
                if (e.to_ == id) { tag = static_cast<int>(e.edge_tag_); break; }
            }
        }
        result.edges.push_back({pit->second.pos_, nit->second.pos_, tag});
    }
    return result;
}

// ── StitchAndPrune ────────────────────────────────────────────────────────────

AgentPath StitchAndPrune(const std::vector<uint32_t>& node_ids, lowres::LowResMap* lmap,
                         double stitch_prune_len) {
    AgentPath result;
    if (node_ids.size() < 2) return result;

    // Step 1: collect raw (tag, path_pts) per TopoEdge
    struct RawSeg { TrajSegTag tag; std::vector<Eigen::Vector3d> pts; };
    std::vector<RawSeg> raw;
    raw.reserve(node_ids.size() - 1);

    for (size_t i = 0; i + 1 < node_ids.size(); ++i) {
        uint32_t from = node_ids[i], to = node_ids[i + 1];
        auto adj_it = lmap->topo_adj_.find(from);
        if (adj_it == lmap->topo_adj_.end()) continue;

        for (const auto& e : adj_it->second) {
            if (e.to_ != to) continue;
            TrajSegTag tag;
            switch (e.edge_tag_) {
                case lowres::EDGE_AERIAL: tag = SEG_AERIAL; break;
                case lowres::EDGE_CROSS:  tag = SEG_TRANS;  break;
                default:                  tag = SEG_GROUND; break;
            }
            raw.push_back({tag, e.path_});
            break;
        }
    }

    if (raw.empty()) return result;

    // Snap a world position to the nearest LowRes node center
    const Eigen::Vector3d& lr_scale = lmap->node_scale_;
    const Eigen::Vector3d& lr_orig  = lmap->origin_;
    auto snap_to_node = [&](const Eigen::Vector3d& p) -> Eigen::Vector3d {
        Eigen::Vector3i idx(
            (int)std::floor((p(0) - lr_orig(0)) / lr_scale(0)),
            (int)std::floor((p(1) - lr_orig(1)) / lr_scale(1)),
            (int)std::floor((p(2) - lr_orig(2)) / lr_scale(2)));
        return (idx.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)).cwiseProduct(lr_scale) + lr_orig;
    };

    // Interpolation step: one LowRes voxel width in xy
    const double interp_step = std::min(lr_scale(0), lr_scale(1));

    // Step 2: group consecutive same-tag segments, interpolate then double-pass prune
    size_t i = 0;
    while (i < raw.size()) {
        TrajSegTag cur_tag = raw[i].tag;

        // find end of this tag-group
        size_t j = i + 1;
        while (j < raw.size() && raw[j].tag == cur_tag) ++j;

        // concatenate waypoints; segments share boundary points → skip duplicate start
        std::list<Eigen::Vector3d> pts_list;
        for (size_t k = i; k < j; ++k) {
            const auto& seg_pts = raw[k].pts;
            if (seg_pts.empty()) continue;
            size_t start = (pts_list.empty()) ? 0 : 1;
            for (size_t p = start; p < seg_pts.size(); ++p)
                pts_list.push_back(seg_pts[p]);
        }

        if (pts_list.size() >= 2) {
            // Interpolate at LowRes grid resolution, snapping each inserted point to a node center
            std::list<Eigen::Vector3d> dense;
            for (auto it = pts_list.begin(); std::next(it) != pts_list.end(); ++it) {
                Eigen::Vector3d a = snap_to_node(*it);
                Eigen::Vector3d b = snap_to_node(*std::next(it));
                Eigen::Vector3d diff = b - a;
                double d = diff.norm();
                int n = std::max(1, (int)std::ceil(d / interp_step));
                for (int s = 0; s < n; ++s)
                    dense.push_back(a + diff * (s / (double)n));
            }
            dense.push_back(snap_to_node(pts_list.back()));

            std::list<Eigen::Vector3d> pruned;
            double len = 0.0;
            bool ok = dense.size() >= 2 &&
                      lmap->PrunePath(dense, pruned, len, stitch_prune_len) && pruned.size() >= 2;

            const std::list<Eigen::Vector3d>& out = ok ? pruned : dense;
            result.push_back({cur_tag,
                std::vector<Eigen::Vector3d>(out.begin(), out.end())});
        }

        i = j;
    }

    return result;
}

} // namespace path_planning
