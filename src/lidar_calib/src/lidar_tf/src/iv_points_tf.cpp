#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>

class IvPointsChangeTF : public rclcpp::Node
{
public:
  IvPointsChangeTF()
  : Node("iv_points_right_to_left_change")
  {
    /* ===============================
     * Subscriber QoS
     *  - LiDAR driver 계열은 대부분 BEST_EFFORT
     * =============================== */
    rclcpp::QoS sub_qos = rclcpp::SensorDataQoS();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/iv_points_right",
      sub_qos,
      std::bind(&IvPointsChangeTF::cloudCallback, this, std::placeholders::_1)
    );

    /* ===============================
     * Publisher QoS (중요)
     *  - RViz, fusion, bag record 대응
     *  - RELIABLE 필수
     * =============================== */
    rclcpp::QoS pub_qos(rclcpp::KeepLast(10));
    pub_qos.reliable();

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/iv_points_right_applyTF",
      pub_qos
    );

    initTransform();

    RCLCPP_INFO(this->get_logger(),
      // "lidar_tf change node started (BEST_EFFORT sub / RELIABLE pub)");
      "lidar_tf change node started");
  }

private:
  /* ===============================
   * Calibration Transform
   * seyond_right -> seyond_left
   * =============================== */
  // void initTransform()
  // {
  //   T_.setIdentity();

  //   T_(0,0) =  0.99208173;  T_(0,1) = -0.12492030;  T_(0,2) =  0.01299051;  T_(0,3) =  0.03755384;
  //   T_(1,0) =  0.12460735;  T_(1,1) =  0.99194833;  T_(1,2) =  0.02261700;  T_(1,3) = -0.33950112;
  //   T_(2,0) = -0.01571124;  T_(2,1) = -0.02081920;  T_(2,2) =  0.99965980;  T_(2,3) =  0.08904954;
  // }

  void initTransform()
  {
    const std::string file_path =
      "/home/wise/lidar_calib/src/Multi_LiCa/output/results.txt";

    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_FATAL(this->get_logger(),
        "Failed to open calibration file: %s", file_path.c_str());
      rclcpp::shutdown();
      return;
    }

    std::string line;
    bool found_matrix = false;
    Eigen::Matrix4f T;
    T.setZero();

    while (std::getline(file, line)) {
      if (line.find("calibrated transformation matrix") != std::string::npos) {
        // 다음 4줄이 행렬
        for (int i = 0; i < 4; ++i) {
          std::getline(file, line);

          // [, ], 제거
          line.erase(
            std::remove_if(line.begin(), line.end(),
              [](char c) { return c == '[' || c == ']'; }),
            line.end()
          );

          std::stringstream ss(line);
          for (int j = 0; j < 4; ++j) {
            ss >> T(i, j);
          }
        }
        found_matrix = true;
        break;
      }
    }

    file.close();

    if (!found_matrix) {
      RCLCPP_FATAL(this->get_logger(),
        "Transformation matrix not found in results.txt");
      rclcpp::shutdown();
      return;
    }

    T_ = T;

    RCLCPP_INFO(this->get_logger(),
      "Loaded calibration matrix from file:\n"
      "[%f %f %f %f]\n"
      "[%f %f %f %f]\n"
      "[%f %f %f %f]\n"
      "[%f %f %f %f]",
      T_(0,0), T_(0,1), T_(0,2), T_(0,3),
      T_(1,0), T_(1,1), T_(1,2), T_(1,3),
      T_(2,0), T_(2,1), T_(2,2), T_(2,3),
      T_(3,0), T_(3,1), T_(3,2), T_(3,3)
    );
  }

  /* ===============================
   * PointCloud Callback
   * =============================== */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(
      new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *cloud_in);
    if (cloud_in->empty())
      return;

    // Apply calibration transform
    pcl::transformPointCloud(*cloud_in, *cloud_out, T_);

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*cloud_out, out_msg);

    out_msg.header.stamp = msg->header.stamp;
    out_msg.header.frame_id = "seyond_left";

    pub_->publish(out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  Eigen::Matrix4f T_;
};

/* ===============================
 * main
 * =============================== */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IvPointsChangeTF>());
  rclcpp::shutdown();
  return 0;
}
