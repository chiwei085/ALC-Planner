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
    float saliency_local = 0.0f;
    float saliency_global = 0.0f;
    float plc_intrinsic = 0.0f;
    bool is_lighthouse = false;
    std::vector<int32_t> word_ids;
};

struct GraphEdge
{
    int to = -1;
    float dist = 0.0f;
};

struct GraphState
{
    std::unordered_map<int, Keyframe> keyframes;
    std::unordered_map<int, std::vector<GraphEdge>> adj;
    int robot_node_id = -1;
};

struct ALCCandidate
{
    int tau_id = -1;
    Pose6f rep_pose;
    std::vector<int> keyframe_ids;
    float euclidean_dist = 0.0f;
    float graph_dist = 0.0f;
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
};

}  // namespace alc_planner
