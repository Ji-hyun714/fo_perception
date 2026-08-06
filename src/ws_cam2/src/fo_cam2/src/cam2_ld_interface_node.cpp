// #include "camera_stream/common.hpp"

/* ROS2 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "fo_msgs/msg/cam2_ld.hpp"
#include "cv_bridge/cv_bridge.h"

/* Utils */
// #include "opencv2/opencv.hpp"
#include "fo_cam2/fo_struct.hpp"
#include "fo_cam2/lane_detection.hpp"
#include <yaml-cpp/yaml.h>

/* CPP */
#include <cstdio>
#include <chrono>
#include <thread>
#include <memory>



cv::Mat parseMatrixFromYaml(const YAML::Node& parent_node, const std::string& key)
{
  YAML::Node matrix_node = parent_node[key];
  if (!matrix_node) {
      throw std::runtime_error("YAML key not found: " + key);
  }

  int rows = matrix_node["rows"].as<int>();
  int cols = matrix_node["cols"].as<int>();
  const YAML::Node& data_node = matrix_node["data"];

  if (!data_node || !data_node.IsSequence()) {
      throw std::runtime_error("'data' field is missing or not a sequence for key: " + key);
  }

  if (data_node.size() != static_cast<size_t>(rows * cols)) {
      throw std::runtime_error("Data size does not match rows*cols for key: " + key);
  }

  cv::Mat mat(rows, cols, CV_64F); 
  int data_idx = 0;
  for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
          mat.at<double>(r, c) = data_node[data_idx++].as<double>();
      }
  }

  return mat;
}

CamCalib getCalibPramasFromYaml(std::string& yaml_path)
{
  CamCalib calib;
  YAML::Node root = YAML::LoadFile(yaml_path);
  const auto rc = root["calib"];
  if (!rc) throw std::runtime_error("missing 'stream' section");

  cv::Mat K, D, R, t;
  K = parseMatrixFromYaml(rc, "camera_matrix");
  D = parseMatrixFromYaml(rc, "distortion_coefficients");
  R = parseMatrixFromYaml(rc, "rotation_matrix");
  t = parseMatrixFromYaml(rc, "translation_vector");

  cv::Mat K_rect;
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
          K, D, cv::Size(W_ORG,H_ORG), cv::Mat::eye(3,3,CV_64F), K_rect, 0.0
  ); // 여기부턴 K_rect가 K를 대신함

  // calib.Make(K, D, R, t);
  calib = CamCalib::Make(K_rect, D, R, t);

  std::cout << "--- Parsed Calibration Data ---" << std::endl;
  std::cout << "Rectified Camera Matrix (K_rect):\n" << calib.K << std::endl;
  std::cout << "Distortion (D):\n" << calib.D << std::endl;
  std::cout << "Rotation (R):\n" << calib.R_w2c << std::endl;
  std::cout << "Translation (t):\n" << calib.t_w2c << std::endl;
  std::cout << "---------------------------------" << std::endl;

  // std::cout << "--- Parsed Calibration Data ---" << std::endl;
  // std::cout << "Rectified Camera Matrix (K_rect):\n" << K << std::endl;
  // std::cout << "Distortion (D):\n" << D << std::endl;
  // std::cout << "Rotation (R):\n" << R << std::endl;
  // std::cout << "Translation (t):\n" << t << std::endl;
  // std::cout << "---------------------------------" << std::endl;
  
  return calib;
}

class Cam2LDInterface : public rclcpp::Node
{
public:
  // LaneDetector detector;
  bool debug_mode;

  Cam2LDInterface()
      : Node("Cam2LDInterface")
  {
    RCLCPP_INFO(this->get_logger(), "Cam2LDInterface init");

    this->declare_parameter<std::string>("input_topic", "/image_rect");
    this->declare_parameter<std::string>("output_topic", "/cam2_ld");
    this->declare_parameter<bool>("debug_mode", true);

    std::string yp = "/home/wise/fo_perception/ws_cam2/src/camera_stream/yaml/cam_parameters_220.yaml";
    std::string yaml_path = this->declare_parameter<std::string>("yaml_path",yp);
    calib = getCalibPramasFromYaml(yaml_path);
    
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    debug_mode = this->get_parameter("debug_mode").as_bool();
    
    ld_ = std::make_unique<LaneDetector>(calib);
    RCLCPP_INFO(this->get_logger(), "LaneDetector object initialized.");
  
    pub_ = this->create_publisher<fo_msgs::msg::Cam2LD>(output_topic, 10);
    // pub_dbgimg = this->create_publisher<sensor_msgs::msg::Image>("/dbg_img_ld", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_topic,
        10,
        std::bind(&Cam2LDInterface::callbackLaneDection, this, std::placeholders::_1));
  }
  
  ~Cam2LDInterface()
  {
  }
  
private:
  rclcpp::Publisher<fo_msgs::msg::Cam2LD>::SharedPtr pub_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_dbgimg;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  cv::Mat frame, dbg_img;
  sensor_msgs::msg::Image::SharedPtr dbg_img_msg;
  
  uint8_t cnt=0;

  CamCalib calib;
  std::unique_ptr<LaneDetector> ld_;
  
  void callbackLaneDection(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "callbackCam2LD starts");

    try
    {
      frame = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // std::pair<cv::Vec4d, cv::Vec4d> coeffs = detector.outCam2LD(frame);
    std::pair<cv::Vec4d, cv::Vec4d> coeffs = ld_->outCam2LD(frame);
    if (debug_mode) {
      std::cout << "L: " << coeffs.first << std::endl;
      std::cout << "R: " << coeffs.second << std::endl << std::endl;

      dbg_img = ld_->create_dbg_img();
      // dbg_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dbg_img).toImageMsg();
      // pub_dbgimg->publish(*dbg_img_msg);
    }

    /* Build cam2 LD msg */
    auto msg_pub = std::make_unique<fo_msgs::msg::Cam2LD>();
    msg_pub->la.lane_mark_type     = 0xff; 
    msg_pub->la.lane_mark_quality  = 0xff; 
    msg_pub->la.lane_mark_position = coeffs.first[2];
    msg_pub->la.lane_mark_model_a  = coeffs.first[0];
    msg_pub->la.lane_mark_width    = 0xff;

    msg_pub->ra.lane_mark_type     = 0xff; 
    msg_pub->ra.lane_mark_quality  = 0xff; 
    msg_pub->ra.lane_mark_position = coeffs.second[2];
    msg_pub->ra.lane_mark_model_a  = coeffs.second[0];
    msg_pub->ra.lane_mark_width    = 0xff;

    msg_pub->lb.lane_mark_heading_angle                 = coeffs.first[1];
    msg_pub->lb.lane_mark_model_view_range              = 0xff;
    msg_pub->lb.lane_mark_model_view_range_availability = false;
    msg_pub->lb.lane_mark_model_da                      = coeffs.first[3];

    msg_pub->rb.lane_mark_heading_angle                 = coeffs.second[1];
    msg_pub->rb.lane_mark_model_view_range              = 0xff;
    msg_pub->rb.lane_mark_model_view_range_availability = false;
    msg_pub->rb.lane_mark_model_da                      = coeffs.second[3];

    msg_pub->add.rolling_counter              = cnt;
    msg_pub->add.rh_guardrail                 = false;
    msg_pub->add.lh_guardrail                 = false;
    msg_pub->add.right_lane_color_information = 0xff;
    msg_pub->add.left_lane_color_information  = 0xff;

    pub_->publish(std::move(msg_pub));
    if (debug_mode) {
      RCLCPP_INFO(this->get_logger(), "Cam2_LD message published.");
    }

    cnt = (cnt + 1) & 255;
  }
  
};

int main(int argc, char **argv)
{
  // (void) argc;
  // (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Cam2LDInterface>());
  rclcpp::shutdown();

  return 0;
}
