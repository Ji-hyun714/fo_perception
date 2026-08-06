#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using std::placeholders::_1;
using sensor_msgs::msg::PointCloud2;

class PointCloudMerger : public rclcpp::Node
{
public:
  PointCloudMerger()
  : Node("lidar_merge_node")
  {
    // Subscribers
    sub_lidar1_ = this->create_subscription<PointCloud2>(
      "/lidar_11_201/TFpoints_new", 10, std::bind(&PointCloudMerger::callbackLidar1, this, _1));
    sub_lidar2_ = this->create_subscription<PointCloud2>(
      "/lidar_12_201/TFpoints_new", 10, std::bind(&PointCloudMerger::callbackLidar2, this, _1));
    sub_lidar3_ = this->create_subscription<PointCloud2>(
      "/lidar_13_201/TFpoints_new", 10, std::bind(&PointCloudMerger::callbackLidar3, this, _1));

    // Publisher
    pub_merged_ = this->create_publisher<PointCloud2>("/merge_201/points_new", 10);
    // pub_merged_ = this->create_publisher<PointCloud2>("/merge_201/points_new", 10);

    // 50ms Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PointCloudMerger::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "✅ PointCloudMerger started. (50ms timer)");
  }

private:
  // ===== LiDAR Callbacks =====
  void callbackLidar1(const PointCloud2::SharedPtr msg)
  {
    buffer_lidar1_ = *msg;
    time_lidar1_ = this->get_clock()->now();
  }

  void callbackLidar2(const PointCloud2::SharedPtr msg)
  {
    buffer_lidar2_ = *msg;
    time_lidar2_ = this->get_clock()->now();
  }

  void callbackLidar3(const PointCloud2::SharedPtr msg)
  {
    buffer_lidar3_ = *msg;
    time_lidar3_ = this->get_clock()->now();
  }

  // Timer (50ms)
  void timerCallback()
  {
    auto now = this->get_clock()->now();
    // double timeout_ms = 150.0;  // ✅ 150 ms 이내 데이터만 유효
    double timeout_ms = 250.0;

    bool valid1 = (now - time_lidar1_).seconds() * 1000.0 < timeout_ms;
    bool valid2 = (now - time_lidar2_).seconds() * 1000.0 < timeout_ms;
    bool valid3 = (now - time_lidar3_).seconds() * 1000.0 < timeout_ms;

    std::vector<PointCloud2> valid_pcs;
    std::vector<rclcpp::Time> valid_stamps;

    if (valid1 && !buffer_lidar1_.data.empty()) {
      valid_pcs.push_back(buffer_lidar1_);
      valid_stamps.push_back(rclcpp::Time(buffer_lidar1_.header.stamp));
    } else buffer_lidar1_.data.clear();

    if (valid2 && !buffer_lidar2_.data.empty()) {
      valid_pcs.push_back(buffer_lidar2_);
      valid_stamps.push_back(rclcpp::Time(buffer_lidar2_.header.stamp));
    } else buffer_lidar2_.data.clear();

    if (valid3 && !buffer_lidar3_.data.empty()) {
      valid_pcs.push_back(buffer_lidar3_);
      valid_stamps.push_back(rclcpp::Time(buffer_lidar3_.header.stamp));
    } else buffer_lidar3_.data.clear();

    // 병합 수행
    if (!valid_pcs.empty())
    {
      PointCloud2 merged = valid_pcs[0];
      for (size_t i = 1; i < valid_pcs.size(); ++i)
        merged = mergePointClouds(merged, valid_pcs[i]);

      // ✅ 가장 "먼저" 들어온 timestamp 사용
      rclcpp::Time earliest_stamp = getEarliestStamp(valid_stamps);

      builtin_interfaces::msg::Time stamp_msg;
      stamp_msg.sec = earliest_stamp.seconds();
      stamp_msg.nanosec = earliest_stamp.nanoseconds() % 1000000000;

      merged.header.stamp = stamp_msg;
      merged.header.frame_id = "vehicle";

      pub_merged_->publish(merged);
    }
    else
    {
      // 3개 라이다 모두 단절
      // RCLCPP_WARN(this->get_logger(),
      //   "⚠ No valid LiDAR data within %.0f ms → skip publish", timeout_ms);
    }

    // Timeout 로그
    // if (!valid1) RCLCPP_WARN(this->get_logger(), "🚫 LiDAR 11_201 timeout (>150 ms)");
    // if (!valid2) RCLCPP_WARN(this->get_logger(), "🚫 LiDAR 12_201 timeout (>150 ms)");
    // if (!valid3) RCLCPP_WARN(this->get_logger(), "🚫 LiDAR 13_201 timeout (>150 ms)");
  }

  // PointCloud 병합
  PointCloud2 mergePointClouds(const PointCloud2 &pc1, const PointCloud2 &pc2)
  {
    PointCloud2 merged = pc1;
    size_t total_points = pc1.width * pc1.height + pc2.width * pc2.height;

    merged.width = total_points;
    merged.row_step = merged.point_step * merged.width;
    merged.data.reserve(pc1.data.size() + pc2.data.size());
    merged.data.insert(merged.data.end(), pc2.data.begin(), pc2.data.end());
    return merged;
  }

  rclcpp::Time getEarliestStamp(const std::vector<rclcpp::Time> &stamps)
  {
    if (stamps.empty()) return this->get_clock()->now();
    rclcpp::Time earliest = stamps[0];
    for (const auto &t : stamps)
      if (t < earliest)
        earliest = t;
    return earliest;
  }

  // 멤버 변수
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lidar1_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lidar2_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lidar3_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_merged_;
  rclcpp::TimerBase::SharedPtr timer_;

  PointCloud2 buffer_lidar1_;
  PointCloud2 buffer_lidar2_;
  PointCloud2 buffer_lidar3_;
  rclcpp::Time time_lidar1_{0, 0, RCL_ROS_TIME};
  rclcpp::Time time_lidar2_{0, 0, RCL_ROS_TIME};
  rclcpp::Time time_lidar3_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMerger>());
  rclcpp::shutdown();
  return 0;
}