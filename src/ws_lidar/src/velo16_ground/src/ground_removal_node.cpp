#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"


struct PointXYZIR
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{0.0f};
  uint16_t ring{0};
};


class RingGroundRemovalNode : public rclcpp::Node
{
public:
  RingGroundRemovalNode()
  : Node("ring_ground_removal_node")
  {
    this->declare_parameter<std::string>("input_topic", "/lidar_11_201/points");
    this->declare_parameter<std::string>("ground_topic", "/ring_ground/lidar_11_201");
    this->declare_parameter<std::string>("nonground_topic", "/ring_nonground/lidar_11_201");

    this->declare_parameter<double>("min_range", 0.0);
    this->declare_parameter<double>("max_range", 80.0);

    this->declare_parameter<double>("ground_z_min", -5.0);
    this->declare_parameter<double>("ground_z_max", -0.2);

    this->declare_parameter<double>("max_neighbor_z_diff", 0.25);
    this->declare_parameter<double>("max_slope_deg", 12.0);
    this->declare_parameter<double>("max_neighbor_xy_dist", 1.0);

    this->declare_parameter<bool>("use_near_low_z_ground", true);
    this->declare_parameter<double>("near_low_z_range", 4.5);
    this->declare_parameter<double>("near_low_z_threshold", -0.4);

    this->declare_parameter<bool>("allow_no_ring", false);

    input_topic_ = this->get_parameter("input_topic").as_string();
    ground_topic_ = this->get_parameter("ground_topic").as_string();
    nonground_topic_ = this->get_parameter("nonground_topic").as_string();

    min_range_ = this->get_parameter("min_range").as_double();
    max_range_ = this->get_parameter("max_range").as_double();

    ground_z_min_ = this->get_parameter("ground_z_min").as_double();
    ground_z_max_ = this->get_parameter("ground_z_max").as_double();

    max_neighbor_z_diff_ = this->get_parameter("max_neighbor_z_diff").as_double();
    max_slope_deg_ = this->get_parameter("max_slope_deg").as_double();
    max_neighbor_xy_dist_ = this->get_parameter("max_neighbor_xy_dist").as_double();

    max_slope_rad_ = max_slope_deg_ * M_PI / 180.0;

    use_near_low_z_ground_ = this->get_parameter("use_near_low_z_ground").as_bool();
    near_low_z_range_ = this->get_parameter("near_low_z_range").as_double();
    near_low_z_threshold_ = this->get_parameter("near_low_z_threshold").as_double();

    allow_no_ring_ = this->get_parameter("allow_no_ring").as_bool();

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.durability_volatile();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      qos,
      std::bind(&RingGroundRemovalNode::cloudCallback, this, std::placeholders::_1)
    );

    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      ground_topic_,
      qos
    );

    nonground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      nonground_topic_,
      qos
    );

    // RCLCPP_INFO(this->get_logger(), "Ring Ground Removal Node started");
    // RCLCPP_INFO(this->get_logger(), "Node name : %s", this->get_name());
    RCLCPP_INFO(
      this->get_logger(),
      "%s -> %s",
      input_topic_.c_str(),
      nonground_topic_.c_str()
    );
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const bool has_intensity = hasField(*msg, "intensity");
    const bool has_ring = hasField(*msg, "ring");

    if (!has_ring) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "PointCloud2에 ring 필드가 없습니다."
      );

      if (!allow_no_ring_) {
        publishEmpty(msg->header);
        return;
      }
    }

    std::vector<PointXYZIR> points;
    points.reserve(msg->width * msg->height);

    if (has_intensity && has_ring) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
      sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg, "ring");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_ring) {
        if (!isFiniteXYZ(*iter_x, *iter_y, *iter_z)) {
          continue;
        }

        PointXYZIR p;
        p.x = *iter_x;
        p.y = *iter_y;
        p.z = *iter_z;
        p.intensity = std::isfinite(*iter_intensity) ? *iter_intensity : 0.0f;
        p.ring = *iter_ring;

        points.push_back(p);
      }
    } else if (has_intensity && !has_ring) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
        if (!isFiniteXYZ(*iter_x, *iter_y, *iter_z)) {
          continue;
        }

        PointXYZIR p;
        p.x = *iter_x;
        p.y = *iter_y;
        p.z = *iter_z;
        p.intensity = std::isfinite(*iter_intensity) ? *iter_intensity : 0.0f;
        p.ring = 0;

        points.push_back(p);
      }
    } else if (!has_intensity && has_ring) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg, "ring");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_ring) {
        if (!isFiniteXYZ(*iter_x, *iter_y, *iter_z)) {
          continue;
        }

        PointXYZIR p;
        p.x = *iter_x;
        p.y = *iter_y;
        p.z = *iter_z;
        p.intensity = 0.0f;
        p.ring = *iter_ring;

        points.push_back(p);
      }
    } else {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!isFiniteXYZ(*iter_x, *iter_y, *iter_z)) {
          continue;
        }

        PointXYZIR p;
        p.x = *iter_x;
        p.y = *iter_y;
        p.z = *iter_z;
        p.intensity = 0.0f;
        p.ring = 0;

        points.push_back(p);
      }
    }

    std::vector<PointXYZIR> valid_points;
    valid_points.reserve(points.size());

    for (const auto & p : points) {
      const double range = std::sqrt(
        static_cast<double>(p.x) * static_cast<double>(p.x) +
        static_cast<double>(p.y) * static_cast<double>(p.y)
      );

      if (
        std::isfinite(p.x) &&
        std::isfinite(p.y) &&
        std::isfinite(p.z) &&
        range >= min_range_ &&
        range <= max_range_
      ) {
        valid_points.push_back(p);
      }
    }

    if (valid_points.empty()) {
      publishEmpty(msg->header);
      return;
    }

    std::vector<uint8_t> ground_mask = classifyGroundByRing(valid_points);

    std::vector<PointXYZIR> ground_points;
    std::vector<PointXYZIR> nonground_points;

    ground_points.reserve(valid_points.size());
    nonground_points.reserve(valid_points.size());

    for (std::size_t i = 0; i < valid_points.size(); ++i) {
      if (ground_mask[i]) {
        ground_points.push_back(valid_points[i]);
      } else {
        nonground_points.push_back(valid_points[i]);
      }
    }

    auto ground_msg = createCloud(msg->header, ground_points);
    auto nonground_msg = createCloud(msg->header, nonground_points);

    ground_pub_->publish(ground_msg);
    nonground_pub_->publish(nonground_msg);
  }

  std::vector<uint8_t> classifyGroundByRing(const std::vector<PointXYZIR> & points)
  {
    std::vector<uint8_t> ground_mask(points.size(), 0);

    std::unordered_map<uint16_t, std::vector<std::size_t>> ring_map;
    ring_map.reserve(32);

    for (std::size_t i = 0; i < points.size(); ++i) {
      ring_map[points[i].ring].push_back(i);
    }

    for (auto & kv : ring_map) {
      auto & ring_indices = kv.second;

      if (ring_indices.size() < 3) {
        continue;
      }

      std::sort(
        ring_indices.begin(),
        ring_indices.end(),
        [&points](std::size_t a, std::size_t b) {
          const double azimuth_a = std::atan2(points[a].y, points[a].x);
          const double azimuth_b = std::atan2(points[b].y, points[b].x);
          return azimuth_a < azimuth_b;
        }
      );

      const std::size_t n = ring_indices.size();

      for (std::size_t i = 0; i < n; ++i) {
        const std::size_t curr_idx = ring_indices[i];
        const auto & curr = points[curr_idx];

        const double curr_x = curr.x;
        const double curr_y = curr.y;
        const double curr_z = curr.z;

        const double curr_range = std::sqrt(curr_x * curr_x + curr_y * curr_y);

        if (use_near_low_z_ground_) {
          if (
            curr_range <= near_low_z_range_ &&
            curr_z < near_low_z_threshold_
          ) {
            ground_mask[curr_idx] = 1;
            continue;
          }
        }

        const bool z_candidate =
          curr_z >= ground_z_min_ &&
          curr_z <= ground_z_max_;

        if (!z_candidate) {
          ground_mask[curr_idx] = 0;
          continue;
        }

        int neighbor_ground_count = 0;
        int neighbor_count = 0;

        for (const int offset : {-1, 1}) {
          const int j_signed = static_cast<int>(i) + offset;

          if (j_signed < 0 || j_signed >= static_cast<int>(n)) {
            continue;
          }

          const std::size_t neighbor_idx = ring_indices[static_cast<std::size_t>(j_signed)];
          const auto & neighbor = points[neighbor_idx];

          const double dx = curr_x - static_cast<double>(neighbor.x);
          const double dy = curr_y - static_cast<double>(neighbor.y);
          const double dz = curr_z - static_cast<double>(neighbor.z);

          const double xy_dist = std::sqrt(dx * dx + dy * dy);

          if (xy_dist < 1e-6) {
            continue;
          }

          if (xy_dist > max_neighbor_xy_dist_) {
            continue;
          }

          const double slope = std::abs(std::atan2(dz, xy_dist));
          const double z_diff = std::abs(dz);

          neighbor_count++;

          if (
            z_diff <= max_neighbor_z_diff_ &&
            slope <= max_slope_rad_
          ) {
            neighbor_ground_count++;
          }
        }

        if (neighbor_count > 0 && neighbor_ground_count > 0) {
          ground_mask[curr_idx] = 1;
        } else {
          ground_mask[curr_idx] = 0;
        }
      }
    }

    return ground_mask;
  }

  sensor_msgs::msg::PointCloud2 createCloud(
    const std_msgs::msg::Header & header,
    const std::vector<PointXYZIR> & points)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header = header;
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(points.size());
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);

    modifier.setPointCloud2Fields(
      5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16
    );

    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(cloud_msg, "ring");

    for (const auto & p : points) {
      *iter_x = p.x;
      *iter_y = p.y;
      *iter_z = p.z;
      *iter_intensity = p.intensity;
      *iter_ring = p.ring;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
      ++iter_ring;
    }

    return cloud_msg;
  }

  void publishEmpty(const std_msgs::msg::Header & header)
  {
    std::vector<PointXYZIR> empty;

    auto ground_msg = createCloud(header, empty);
    auto nonground_msg = createCloud(header, empty);

    ground_pub_->publish(ground_msg);
    nonground_pub_->publish(nonground_msg);
  }

  static bool hasField(
    const sensor_msgs::msg::PointCloud2 & msg,
    const std::string & name)
  {
    for (const auto & field : msg.fields) {
      if (field.name == name) {
        return true;
      }
    }

    return false;
  }

  static bool isFiniteXYZ(float x, float y, float z)
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }

private:
  std::string input_topic_;
  std::string ground_topic_;
  std::string nonground_topic_;

  double min_range_{0.0};
  double max_range_{80.0};

  double ground_z_min_{-5.0};
  double ground_z_max_{-0.2};

  double max_neighbor_z_diff_{0.25};
  double max_slope_deg_{12.0};
  double max_slope_rad_{12.0 * M_PI / 180.0};
  double max_neighbor_xy_dist_{1.0};

  bool use_near_low_z_ground_{true};
  double near_low_z_range_{4.5};
  double near_low_z_threshold_{-0.4};

  bool allow_no_ring_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_pub_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RingGroundRemovalNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}