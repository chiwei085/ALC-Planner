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
};

struct GraphEdge
{
    int to = -1;
    bool closure = false;
};

struct GraphState
{
    std::vector<Keyframe> keyframes;
    std::vector<std::vector<GraphEdge>> adj;
    std::unordered_map<int, int> node_to_ix;
    std::vector<int> ix_to_node;
    int robot_ix = -1;
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
    float P_lc = 0.0f;
    // Entropy of the robot pose relative to the keyframe, now against after
    // closing there and driving straight back: a log ratio of covariances, so
    // it has no unit. Driving away needs no revisit: its error is the direct
    // drive's own.
    float gain = 0.0f;
    float log_det = 0.0f;
    float position_std = 0.0f;
    float euclidean_dist = 0.0f;
    float map_dist = 0.0f;
    float cost = 0.0f;
    float score = 0.0f;
};

struct Params
{
    float cv_L = 3.0f;
    float cv_G = 3.0f;
    float cs = 0.3f;

    // Drift and closure noise of the R2 with RTAB-Map, fitted on 108 closure
    // chains up to 15.6 m. They cannot tell yaw drift per metre from 0 to 1e-3;
    // zero is not physical, so a value inside that range is assumed.
    float drift_trans_var_per_m = 5.4e-4f;
    float drift_yaw_var_per_rad = 2.9e-3f;
    float drift_yaw_var_per_m = 3.0e-4f;
    float closure_trans_var = 4.0e-3f;
    float closure_yaw_var = 7.3e-3f;

    // Free room beside the car: goal clearance 0.22 m minus half width 0.115.
    // A position error above it can put a mapped passage into a wall.
    float max_position_std_m = 0.105f;
    float turning_radius_m = 0.5f;
    // Nav2 goal checker tolerances.
    float arrival_xy_m = 0.35f;
    float arrival_yaw_rad = 0.5f;
};

}  // namespace alc_planner
