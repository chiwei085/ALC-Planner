#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap_msgs/msg/info.hpp>
#include <rtabmap_msgs/msg/link.hpp>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <rtabmap_msgs/msg/node.hpp>
#include <sqlite3.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace alc_planner
{
namespace
{

class Statement
{
public:
    Statement(sqlite3* db, const std::string& sql) : db_(db) {
        if (sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt_, nullptr) !=
            SQLITE_OK) {
            throw std::runtime_error("sqlite3_prepare_v2 failed for query: " +
                                     sql);
        }
    }

    ~Statement() {
        if (stmt_ != nullptr) {
            sqlite3_finalize(stmt_);
        }
    }

    sqlite3_stmt* get() const { return stmt_; }

private:
    sqlite3* db_ = nullptr;
    sqlite3_stmt* stmt_ = nullptr;
};

geometry_msgs::msg::Pose parsePoseBlob(sqlite3_stmt* stmt,
                                       const int column_index) {
    const void* blob = sqlite3_column_blob(stmt, column_index);
    const int blob_bytes = sqlite3_column_bytes(stmt, column_index);
    if (blob == nullptr ||
        (blob_bytes != static_cast<int>(12 * sizeof(float)) &&
         blob_bytes != static_cast<int>(16 * sizeof(float)))) {
        throw std::runtime_error("Unexpected pose blob size");
    }

    const float* matrix_data = static_cast<const float*>(blob);
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    const int rows = blob_bytes == static_cast<int>(12 * sizeof(float)) ? 3 : 4;
    constexpr int cols = 4;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            matrix(row, col) = matrix_data[row * 4 + col];
        }
    }

    Eigen::Quaternionf orientation(matrix.block<3, 3>(0, 0));
    orientation.normalize();

    geometry_msgs::msg::Pose pose;
    pose.position.x = matrix(0, 3);
    pose.position.y = matrix(1, 3);
    pose.position.z = matrix(2, 3);
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    return pose;
}

std::array<double, 36> parseInformationBlob(sqlite3_stmt* stmt,
                                            const int column_index) {
    const void* blob = sqlite3_column_blob(stmt, column_index);
    const int blob_bytes = sqlite3_column_bytes(stmt, column_index);
    if (blob == nullptr ||
        blob_bytes != static_cast<int>(36 * sizeof(double))) {
        throw std::runtime_error("Unexpected information blob size");
    }

    std::array<double, 36> information{};
    std::memcpy(information.data(), blob, sizeof(double) * information.size());
    return information;
}

rtabmap_msgs::msg::MapData loadMapDataFromDb(const std::string& db_path,
                                             std::size_t& max_word_count,
                                             int& first_node_id,
                                             float& map_resolution) {
    sqlite3* db = nullptr;
    if (sqlite3_open_v2(db_path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) !=
        SQLITE_OK) {
        const std::string error =
            db != nullptr ? sqlite3_errmsg(db) : "unknown";
        if (db != nullptr) {
            sqlite3_close(db);
        }
        throw std::runtime_error("sqlite3_open_v2 failed: " + error);
    }

    struct DbCloser
    {
        sqlite3* db = nullptr;
        ~DbCloser() {
            if (db != nullptr) {
                sqlite3_close(db);
            }
        }
    } db_closer{db};

    rtabmap_msgs::msg::MapData map_data;
    map_data.header.frame_id = "map";
    map_data.graph.header.frame_id = "map";

    std::unordered_map<int, std::size_t> node_index_by_id;

    {
        Statement stmt(db, "SELECT id, pose FROM Node ORDER BY id");
        while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
            rtabmap_msgs::msg::Node node_msg;
            node_msg.id = sqlite3_column_int(stmt.get(), 0);
            node_msg.pose = parsePoseBlob(stmt.get(), 1);

            if (first_node_id < 0) {
                first_node_id = node_msg.id;
            }

            node_index_by_id.emplace(node_msg.id, map_data.nodes.size());
            map_data.graph.poses_id.push_back(node_msg.id);
            map_data.graph.poses.push_back(node_msg.pose);
            map_data.nodes.push_back(std::move(node_msg));
        }
    }

    {
        Statement stmt(
            db,
            "SELECT node_id, word_id FROM Feature ORDER BY node_id, word_id");
        int current_node_id = std::numeric_limits<int>::min();
        std::unordered_set<int32_t> unique_words;

        while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
            const int node_id = sqlite3_column_int(stmt.get(), 0);
            const int32_t word_id =
                static_cast<int32_t>(sqlite3_column_int(stmt.get(), 1));
            if (current_node_id != node_id) {
                current_node_id = node_id;
                unique_words.clear();
            }

            if (!unique_words.insert(word_id).second) {
                continue;
            }

            const auto it = node_index_by_id.find(node_id);
            if (it == node_index_by_id.end()) {
                continue;
            }
            map_data.nodes[it->second].word_id_keys.push_back(word_id);
        }
    }

    {
        Statement stmt(
            db,
            "SELECT from_id, to_id, type, information_matrix FROM Link "
            "ORDER BY from_id, to_id");
        while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
            rtabmap_msgs::msg::Link link_msg;
            link_msg.from_id = sqlite3_column_int(stmt.get(), 0);
            link_msg.to_id = sqlite3_column_int(stmt.get(), 1);
            link_msg.type = sqlite3_column_int(stmt.get(), 2);
            link_msg.transform.rotation.w = 1.0;
            link_msg.information = parseInformationBlob(stmt.get(), 3);
            map_data.graph.links.push_back(std::move(link_msg));
        }
    }

    {
        Statement stmt(
            db, "SELECT cell_size FROM Data WHERE cell_size > 0 LIMIT 1");
        if (sqlite3_step(stmt.get()) == SQLITE_ROW) {
            map_resolution =
                static_cast<float>(sqlite3_column_double(stmt.get(), 0));
        }
    }

    max_word_count = 0;
    for (const auto& node_msg : map_data.nodes) {
        max_word_count = std::max(max_word_count, node_msg.word_id_keys.size());
    }

    return map_data;
}

nav_msgs::msg::OccupancyGrid makePlaceholderMap(const float resolution) {
    nav_msgs::msg::OccupancyGrid map_msg;
    map_msg.header.frame_id = "map";
    map_msg.info.resolution = resolution > 0.0f ? resolution : 0.05f;
    map_msg.info.width = 1;
    map_msg.info.height = 1;
    map_msg.info.origin.orientation.w = 1.0;
    map_msg.data = {-1};
    return map_msg;
}

class DbPublisherNode : public rclcpp::Node
{
public:
    DbPublisherNode() : rclcpp::Node("db_publisher_node") {
        declare_parameter<std::string>("db_path", "");

        const auto latched_qos =
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        pub_map_data_ = create_publisher<rtabmap_msgs::msg::MapData>(
            "/rtabmap/mapData", latched_qos);
        pub_info_ = create_publisher<rtabmap_msgs::msg::Info>("/rtabmap/info",
                                                              latched_qos);
        pub_map_ =
            create_publisher<nav_msgs::msg::OccupancyGrid>("/map", latched_qos);

        timer_ =
            create_wall_timer(std::chrono::milliseconds(250),
                              std::bind(&DbPublisherNode::publishOnce, this));
    }

private:
    void publishOnce() {
        const bool all_connected =
            pub_map_data_->get_subscription_count() > 0 &&
            pub_info_->get_subscription_count() > 0 &&
            pub_map_->get_subscription_count() > 0;
        if (!all_connected && wait_cycles_ < 20) {
            ++wait_cycles_;
            return;
        }
        timer_->cancel();

        const std::string db_path = get_parameter("db_path").as_string();
        if (db_path.empty()) {
            RCLCPP_ERROR(get_logger(), "Parameter 'db_path' is required");
            rclcpp::shutdown();
            return;
        }

        try {
            std::size_t max_word_count = 0;
            int first_node_id = -1;
            float map_resolution = 0.05f;
            auto map_data = loadMapDataFromDb(db_path, max_word_count,
                                              first_node_id, map_resolution);
            auto map_msg = makePlaceholderMap(map_resolution);

            rtabmap_msgs::msg::Info info_msg;
            info_msg.header.frame_id = "map";
            info_msg.ref_id = first_node_id;
            info_msg.loop_closure_id = 0;
            info_msg.stats_keys.push_back("Keypoint/Words");
            info_msg.stats_values.push_back(static_cast<float>(max_word_count));

            RCLCPP_INFO(get_logger(),
                        "Matched subscribers before publish: mapData=%zu "
                        "info=%zu map=%zu",
                        pub_map_data_->get_subscription_count(),
                        pub_info_->get_subscription_count(),
                        pub_map_->get_subscription_count());

            pub_map_->publish(map_msg);
            pub_info_->publish(info_msg);
            pub_map_data_->publish(map_data);

            RCLCPP_INFO(get_logger(),
                        "Published DB snapshot: %zu nodes, %zu links, "
                        "ref_id=%d, max_words=%zu",
                        map_data.nodes.size(), map_data.graph.links.size(),
                        first_node_id, max_word_count);

            shutdown_timer_ = create_wall_timer(std::chrono::milliseconds(1000),
                                                []() { rclcpp::shutdown(); });
        }
        catch (const std::exception& ex) {
            RCLCPP_ERROR(get_logger(), "DB publish failed: %s", ex.what());
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<rtabmap_msgs::msg::MapData>::SharedPtr pub_map_data_;
    rclcpp::Publisher<rtabmap_msgs::msg::Info>::SharedPtr pub_info_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
    int wait_cycles_ = 0;
};

}  // namespace
}  // namespace alc_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<alc_planner::DbPublisherNode>());
    return 0;
}
