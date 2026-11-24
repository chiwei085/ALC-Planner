#include "alc_planner/revisit_selector.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

namespace alc_planner
{

namespace
{

using Triplets = std::vector<Eigen::Triplet<double>>;

// Keeps zero-length edges (a robot turning on the spot) invertible.
constexpr double kMinVariance = 1.0e-6;

double yawOf(const Pose6f& pose) {
    const Eigen::Vector3f forward = pose.forward();
    return std::atan2(forward.y(), forward.x());
}

double wrap(const double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

struct Planar
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

Planar planar(const Pose6f& pose) {
    return {pose.position.x(), pose.position.y(), yawOf(pose)};
}

// Pose of b in the frame of a.
Planar relative(const Planar& a, const Planar& b) {
    const double c = std::cos(a.yaw);
    const double s = std::sin(a.yaw);
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return {c * dx + s * dy, -s * dx + c * dy, wrap(b.yaw - a.yaw)};
}

Eigen::Matrix3d adjoint(const Planar& t) {
    const double c = std::cos(t.yaw);
    const double s = std::sin(t.yaw);
    Eigen::Matrix3d ad;
    ad << c, -s, t.y, s, c, -t.x, 0.0, 0.0, 1.0;
    return ad;
}

void addBlock(Triplets& triplets, const int row, const int col,
              const Eigen::Matrix3d& block) {
    if (row < 0 || col < 0) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            triplets.emplace_back(3 * row + i, 3 * col + j, block(i, j));
        }
    }
}

std::vector<int> connectedTo(const GraphState& graph, const int start) {
    std::vector<int> order{start};
    std::vector<char> seen(graph.keyframes.size(), 0);
    seen[static_cast<std::size_t>(start)] = 1;
    for (std::size_t i = 0; i < order.size(); ++i) {
        for (const auto& edge : graph.adj[static_cast<std::size_t>(order[i])]) {
            auto& mark = seen[static_cast<std::size_t>(edge.to)];
            if (!mark) {
                mark = 1;
                order.push_back(edge.to);
            }
        }
    }
    return order;
}

double turnAngle(const Planar& from, const Planar& to) {
    const double bearing = std::atan2(to.y - from.y, to.x - from.x);
    return std::abs(wrap(bearing - from.yaw)) +
           std::abs(wrap(to.yaw - bearing));
}

// Robot covariance relative to the keyframe after a closure there and a direct
// drive of `distance` back, in the frame of that drive.
Eigen::Matrix3d closeAndReturn(const double distance, const Planar& keyframe,
                               const Planar& robot, const Params& params) {
    const double bearing =
        std::atan2(robot.y - keyframe.y, robot.x - keyframe.x);
    // Yaw error made before the drive swings it sideways over its length.
    const double before =
        params.closure_yaw_var +
        params.drift_yaw_var_per_rad * std::abs(wrap(bearing - keyframe.yaw));
    const double after =
        params.drift_yaw_var_per_rad * std::abs(wrap(robot.yaw - bearing));
    const double per_m = params.drift_yaw_var_per_m;
    const double d = distance;
    const double trans =
        params.closure_trans_var + params.drift_trans_var_per_m * d;
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov(0, 0) = trans;
    cov(1, 1) = trans + d * d * before + d * d * d * per_m / 3.0;
    cov(2, 2) = before + after + d * per_m;
    cov(1, 2) = cov(2, 1) = d * before + d * d * per_m / 2.0;
    return cov;
}

float gainOver(const float log_det, const double distance,
               const Planar& keyframe, const Planar& robot,
               const Params& params) {
    return static_cast<float>(
        0.5 *
        (log_det -
         std::log(
             closeAndReturn(distance, keyframe, robot, params).determinant())));
}

double drivingCost(const double distance, const Planar& robot,
                   const Planar& target, const double turning_radius) {
    // The car turns on an arc: towards the target, then to its view.
    return distance + turning_radius * turnAngle(robot, target);
}

}  // namespace

RevisitSelector::RevisitSelector(Params params) : params_(params) {}

RevisitDecision RevisitSelector::select(
    const GraphState& graph, const SaliencyState& saliency,
    const nav_msgs::msg::OccupancyGrid& map,
    const std::vector<Pose6f>& excluded) const {
    RevisitDecision decision;
    const int count = static_cast<int>(graph.keyframes.size());
    if (graph.robot_ix < 0 || graph.robot_ix >= count ||
        graph.adj.size() != graph.keyframes.size()) {
        return decision;
    }
    const auto at = [&graph](const int ix) -> const Pose6f& {
        return graph.keyframes[static_cast<std::size_t>(ix)].pose;
    };

    // Information matrix of the pose graph with the robot held fixed.
    const auto component = connectedTo(graph, graph.robot_ix);
    std::vector<int> column(static_cast<std::size_t>(count), -1);
    int unknowns = 0;
    for (const int ix : component) {
        if (ix != graph.robot_ix) {
            column[static_cast<std::size_t>(ix)] = unknowns++;
        }
    }
    if (unknowns == 0) {
        return decision;
    }
    Triplets triplets;
    std::set<std::pair<int, int>> used;
    for (const int from : component) {
        for (const auto& edge : graph.adj[static_cast<std::size_t>(from)]) {
            if (edge.to <= from || !used.emplace(from, edge.to).second) {
                continue;
            }
            const Planar step = relative(planar(at(from)), planar(at(edge.to)));
            double trans = params_.closure_trans_var;
            double yaw = params_.closure_yaw_var;
            if (!edge.closure) {
                trans =
                    params_.drift_trans_var_per_m * std::hypot(step.x, step.y) +
                    kMinVariance;
                yaw = params_.drift_yaw_var_per_rad * std::abs(step.yaw) +
                      params_.drift_yaw_var_per_m * std::hypot(step.x, step.y) +
                      kMinVariance;
            }
            const Eigen::Matrix3d info =
                Eigen::Vector3d(1.0 / trans, 1.0 / trans, 1.0 / yaw)
                    .asDiagonal();
            const Eigen::Matrix3d j = -adjoint(relative(step, Planar{}));
            const int a = column[static_cast<std::size_t>(from)];
            const int b = column[static_cast<std::size_t>(edge.to)];
            addBlock(triplets, a, a, j.transpose() * info * j);
            addBlock(triplets, a, b, j.transpose() * info);
            addBlock(triplets, b, a, info * j);
            addBlock(triplets, b, b, info);
        }
    }
    Eigen::SparseMatrix<double> information(3 * unknowns, 3 * unknowns);
    information.setFromTriplets(triplets.begin(), triplets.end());
    const Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(
        information);
    if (solver.info() != Eigen::Success) {
        return decision;
    }

    const Planar robot = planar(at(graph.robot_ix));
    const auto samePlace = [this](const Planar& a, const Planar& b) {
        return std::hypot(a.x - b.x, a.y - b.y) < params_.arrival_xy_m &&
               std::abs(wrap(a.yaw - b.yaw)) < params_.arrival_yaw_rad;
    };

    std::vector<ALCCandidate> candidates;
    for (const int ix : component) {
        if (ix == graph.robot_ix) {
            continue;
        }
        const int col = column[static_cast<std::size_t>(ix)];
        Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(3 * unknowns, 3);
        rhs.block<3, 3>(3 * col, 0).setIdentity();
        const Eigen::MatrixXd solution = solver.solve(rhs);
        // Keyframe uncertainty with the robot fixed, turned into robot
        // uncertainty with the keyframe fixed.
        const Planar target = planar(at(ix));
        const Eigen::Matrix3d ad = adjoint(relative(robot, target));
        const Eigen::Matrix3d cov =
            ad * solution.block<3, 3>(3 * col, 0) * ad.transpose();
        const double largest =
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
                cov.topLeftCorner<2, 2>(), Eigen::EigenvaluesOnly)
                .eigenvalues()
                .maxCoeff();
        const auto std_m =
            static_cast<float>(std::sqrt(std::max(0.0, largest)));
        decision.max_position_std = std::max(decision.max_position_std, std_m);

        if (std_m <= params_.max_position_std_m ||
            ix >= static_cast<int>(saliency.keyframes.size()) ||
            samePlace(robot, target) ||
            std::any_of(excluded.begin(), excluded.end(),
                        [&](const Pose6f& pose) {
                            return samePlace(planar(pose), target);
                        })) {
            continue;
        }
        const auto& node = saliency.keyframes[static_cast<std::size_t>(ix)];
        if (node.saliency_local < params_.cs) {
            continue;
        }
        ALCCandidate candidate;
        candidate.tau_ix = ix;
        candidate.rep_pose = at(ix);
        candidate.P_lc = std::clamp(
            node.plc_intrinsic * saliency.plc_calibration, 0.0f, 1.0f);
        candidate.position_std = std_m;
        candidate.euclidean_dist = static_cast<float>(
            std::hypot(target.x - robot.x, target.y - robot.y));
        candidate.log_det = static_cast<float>(std::log(cov.determinant()));
        candidate.gain = gainOver(candidate.log_det, candidate.euclidean_dist,
                                  target, robot, params_);
        if (candidate.gain <= 0.0f) {
            continue;
        }
        // Upper bound: a path is never shorter than the straight line.
        candidate.score =
            static_cast<float>(candidate.P_lc * candidate.gain /
                               drivingCost(candidate.euclidean_dist, robot,
                                           target, params_.turning_radius_m));
        if (candidate.score > 0.0f) {
            candidates.push_back(candidate);
        }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const ALCCandidate& a, const ALCCandidate& b) {
                  return a.score > b.score;
              });
    for (auto& candidate : candidates) {
        if (decision.best && candidate.score <= decision.best->score) {
            break;
        }
        candidate.map_dist = path_planner_.computeDist(
            at(graph.robot_ix).position, candidate.rep_pose.position, map);
        if (!std::isfinite(candidate.map_dist)) {
            continue;
        }
        candidate.cost = static_cast<float>(
            drivingCost(candidate.map_dist, robot, planar(candidate.rep_pose),
                        params_.turning_radius_m));
        candidate.gain = gainOver(candidate.log_det, candidate.map_dist,
                                  planar(candidate.rep_pose), robot, params_);
        candidate.score = candidate.P_lc * candidate.gain / candidate.cost;
        if (candidate.gain <= 0.0f) {
            continue;
        }
        if (!decision.best || candidate.score > decision.best->score) {
            decision.best = candidate;
        }
    }
    return decision;
}

}  // namespace alc_planner
