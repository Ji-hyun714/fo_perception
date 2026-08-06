#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <unordered_set>
#include <vector>
#include <iostream>

// ros2 run clustering sub_interval --ros-args -p input_topic:=/lidar/points_fr
// ros2 run clustering sub_interval --ros-args -p input_topic:=/ground
// ros2 run clustering sub_interval --ros-args -p input_topic:=/TFnonground/merged_points


// ------------------- Track 구조체 -------------------
struct Track {
    int id;
    Eigen::Vector2f pos;
    Eigen::Vector2f vel;
    rclcpp::Time last_time;
    float lx, ly, lz;
    int missed;
};

// ------------------- Node -------------------
class LidarClusteringNode : public rclcpp::Node {
public:
    LidarClusteringNode()
    : Node("lidar_tracking_node"),
      next_id_(0),
      first_msg_(true)
    {
        // rclcpp::QoS qos(rclcpp::KeepLast(10));
        // qos.best_effort();
        // qos.durability(rclcpp::DurabilityPolicy::Volatile);

        this->declare_parameter<std::string>("input_topic", "/lidar/points_fl");
        std::string topic = this->get_parameter("input_topic").as_string();

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            topic,  // "/lidar/points_fl",
            10,
            std::bind(&LidarClusteringNode::cloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "✅ Hz Node started: %s", topic.c_str());
    }

private:

    // ------------------- ID allocator (0~255 고정) -------------------
    int allocateTrackId()
    {
        std::unordered_set<int> used_ids;
        for (const auto& t : tracks_)
            used_ids.insert(t.id);

        for (int k = 0; k < 256; k++) {
            int candidate = (next_id_ + k) % 256;
            if (used_ids.count(candidate) == 0) {
                next_id_ = (candidate + 1) % 256;
                return candidate;
            }
        }
        return -1;  // 전부 사용중
    }

    // ------------------- Callback -------------------
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        (void)msg;  // unused warning 제거

        rclcpp::Time now = this->get_clock()->now();

        // ------------------- 수신 간격 측정 -------------------
        if (first_msg_) {
            last_msg_time_ = now;
            first_msg_ = false;
            return;
        }

        double interval_ms =
            (now - last_msg_time_).seconds() * 1000.0;

        std::cout << "Subscription interval: "
                  << interval_ms << " ms" << std::endl;

        last_msg_time_ = now;
    }

    // ------------------- ROS 객체 -------------------
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    // ------------------- Tracking -------------------
    std::vector<Track> tracks_;
    int next_id_;

    // ------------------- Time 측정용 -------------------
    rclcpp::Time last_msg_time_;
    bool first_msg_;
};

// ------------------- main -------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarClusteringNode>());
    rclcpp::shutdown();
    return 0;
}
