#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace alc_planner
{

struct Pose6f
{
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();

    Eigen::Vector3f forward() const {
        return orientation * Eigen::Vector3f::UnitX();
    }
};

struct Keyframe
{
    int node_id = -1;
    Pose6f pose;
    std::vector<int32_t> word_ids;
};

struct KeyframeSaliency
{
    float saliency_local = 0.0f;
    float saliency_global = 0.0f;
    float plc_intrinsic = 0.0f;
    bool is_lighthouse = false;
};

struct GraphEdge
{
    int to = -1;
    float dist = 0.0f;
    float variance = 0.0f;
};

struct GraphState
{
    std::vector<Keyframe> keyframes;
    std::vector<std::vector<GraphEdge>> adj;
    std::unordered_map<int, int> node_to_ix;
    std::vector<int> ix_to_node;
    int robot_ix = -1;
    std::uint64_t version = 0;
};

struct SaliencyState
{
    std::vector<KeyframeSaliency> keyframes;
    float plc_calibration = 1.0f;
};

struct ALCCandidate
{
    int tau_ix = -1;
    Pose6f rep_pose;
    std::vector<int> keyframe_ixs;
    float euclidean_dist = 0.0f;
    float graph_dist = 0.0f;
    float graph_dist_var = 0.0f;
    float map_dist = 0.0f;
    float P_lc = 0.0f;
    float delta_U = 0.0f;
    float delta_U_ub = 0.0f;
    float P_lc_ub = 0.0f;
    float reward = 0.0f;
    float reward_ub = 0.0f;
    bool is_lighthouse = false;
};

struct Params
{
    float cv_L = 3.0f;
    float cv_G = 3.0f;
    float cs_lighthouse = 0.6f;
    float d_min_lighthouse = 5.0f;
    float cl = 10.0f;
    float ct = 0.1f;
    float cE = 15.0f;
    float cG = 3.0f;
    float cs = 0.3f;
    float eps_dbscan = 1.5f;
    int min_pts = 2;
    float theta_max = 0.5f;
    float lambda_decay = 0.01f;
    float alpha_cov = 0.3f;
    bool use_variance_uncertainty = false;
};

}  // namespace alc_planner
