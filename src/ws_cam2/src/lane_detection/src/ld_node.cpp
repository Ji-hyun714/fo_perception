#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <memory>
#include <string>

#include <utils/CalibData.hpp>
#include <utils/common.hpp>

#include "lane_detection/LaneDetection.hpp"
#include "fo_msgs/msg/cam2_ld.hpp"

using std::placeholders::_1;


class LaneDetectionNode : public rclcpp::Node {

public:
  LaneDetectionNode()
    : Node("LaneDetectionNode") 
  {

    this->declare_parameter<std::string>("input_topic", "/camera/image_rect");
    // this->declare_parameter<std::string>("output_topic", "/camera/ld_result");
    this->declare_parameter<std::string>("output_topic", "/camera/lane_result");
    this->declare_parameter<bool>("debug", true);
    this->declare_parameter<std::string>("yaml_path", "");
    this->declare_parameter<std::string>("ld_cfg", "");

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    debug_mode_ = this->get_parameter("debug").as_bool();
    
    std::string yp = "/home/wise/adsp_perception_camera/src/utils/yaml/ioniq_econ.yaml";
    std::string yaml_path = this->get_parameter("yaml_path").as_string();
    if (yaml_path == "") yaml_path = yp;
    
    cam_conf_ = getCameraConfigFromYaml(yaml_path);
    printCameraConfig(cam_conf_);
    if (cam_conf_.model == 0) { // pinhole 
      calib_ = CalibData::MakePinhole(cam_conf_.image_size, cam_conf_.K, cam_conf_.D, cam_conf_.R, cam_conf_.t);
    } else { // fisheye
      calib_ = CalibData::MakeFisheye(cam_conf_.image_size, cam_conf_.K, cam_conf_.D, cam_conf_.R, cam_conf_.t);
    }

    // lld = LaneLineDetector(calib_);
    // lld_ = std::make_unique<LaneLineDetector>(calib_); // 생성자에서 데이터 모두 준비된 뒤 객체 생성하도록
    std::string ldcfg = "/home/wise/adsp_perception_camera/src/lane_detection/yaml/kcity.yaml";
    std::string ld_cfg = this->get_parameter("ld_cfg").as_string();
    if (ld_cfg == "") ld_cfg = ldcfg;

    lld_ = std::make_unique<LaneLineDetector>(calib_, ld_cfg);

    int qos_depth = 10;
    sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_topic, qos_depth, std::bind(&LaneDetectionNode::callbackLaneDetection, this, _1));
    
    pub_ld_ = this->create_publisher<fo_msgs::msg::Cam2LD>(output_topic, qos_depth);
    if (debug_mode_) {
      pub_dbgimg_ = this->create_publisher<sensor_msgs::msg::Image>("camera/ld_dbgimg", qos_depth);
    }
    
    RCLCPP_INFO(this->get_logger(), "LaneDetectionNode Started. Listening on: %s", input_topic.c_str());
  }
  ~LaneDetectionNode() {}
  
  
private:
  bool debug_mode_;

  CameraConfig cam_conf_;
  CalibData calib_;

  // LaneLineDetector lld_;
  std::unique_ptr<LaneLineDetector> lld_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Publisher<fo_msgs::msg::Cam2LD>::SharedPtr pub_ld_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_dbgimg_;

  uint8_t rolling_counter_ = 0;

private:
  void callbackLaneDetection(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;
    try {
      // frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
      frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    
    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty frame");
        return;
    }

    LaneResult res = lld_->Detect(frame);
    int key = lld_->showDebugWindow(); 
    if (key != -1) {
      cv::destroyAllWindows();
      return;
    }

    fo_msgs::msg::Cam2LD msg_ld;
    msg_ld.header_sen.stamp = msg->header.stamp;
    msg_ld.header_sen.frame_id = msg->header.frame_id;

    msg_ld.header_ld.stamp = this->now();
    msg_ld.header_ld.frame_id = "lane_detection";

    fillMsgLD(res, msg_ld);

    pub_ld_->publish(msg_ld);

    if (debug_mode_ && pub_dbgimg_) {
      // cv::Mat dbg_img = lld_->getDebugImage();
      cv::Mat dbg_img = lld_->buf_dbgimg;

      sensor_msgs::msg::Image::SharedPtr msg_dbg = 
          cv_bridge::CvImage(msg->header, "bgr8", dbg_img).toImageMsg();
      pub_dbgimg_->publish(*msg_dbg);
    } 

    rolling_counter_ = (rolling_counter_ + 1) % 255;
  }

  void fillMsgLD(const LaneResult res, fo_msgs::msg::Cam2LD &msg_pub)
  {
    // auto msg_pub = std::make_unique<fo_msgs::msg::Cam2LD>();

    auto stateToLaneMarkQuality = [](LaneState state) -> uint8_t {
      switch (state) {
        case LaneState::GOOD:  return 3;
        case LaneState::WEAK:  return 2;
        case LaneState::BAD:   return 1;
        case LaneState::NODET:
        default:               return 0;
      }
    };

    const uint8_t quality_L = stateToLaneMarkQuality(res.state_L);
    const uint8_t quality_R = stateToLaneMarkQuality(res.state_R);
    const bool use_left_coeffs  = (quality_L >= 2);
    const bool use_right_coeffs = (quality_R >= 2);

    const float left_model_a = use_left_coeffs ? static_cast<float>(res.coeffs_L[0]) : 0.0F;
    const float left_heading = use_left_coeffs ? static_cast<float>(res.coeffs_L[1]) : 0.0F;
    const float left_pos     = use_left_coeffs ? static_cast<float>(res.coeffs_L[2]) : 0.0F;
    const float left_model_da = use_left_coeffs ? static_cast<float>(res.coeffs_L[3]) : 0.0F;
    const float left_view_range_m = std::max(0.0F, static_cast<float>(res.view_range_m_L));
    const float left_lane_start_m = use_left_coeffs ? std::max(0.0F, static_cast<float>(res.lane_start_m_L)) : 0.0F;
    const float left_lane_end_m = use_left_coeffs ? std::max(0.0F, static_cast<float>(res.lane_end_m_L)) : 0.0F;
    const uint8_t left_view_validity = (res.availability_L > 0) ? 1U : 0U;

    const float right_model_a = use_right_coeffs ? static_cast<float>(res.coeffs_R[0]) : 0.0F;
    const float right_heading = use_right_coeffs ? static_cast<float>(res.coeffs_R[1]) : 0.0F;
    const float right_pos     = use_right_coeffs ? static_cast<float>(res.coeffs_R[2]) : 0.0F;
    const float right_model_da = use_right_coeffs ? static_cast<float>(res.coeffs_R[3]) : 0.0F;
    const float right_view_range_m = std::max(0.0F, static_cast<float>(res.view_range_m_R));
    const float right_lane_start_m = use_right_coeffs ? std::max(0.0F, static_cast<float>(res.lane_start_m_R)) : 0.0F;
    const float right_lane_end_m = use_right_coeffs ? std::max(0.0F, static_cast<float>(res.lane_end_m_R)) : 0.0F;
    const uint8_t right_view_validity = (res.availability_R > 0) ? 1U : 0U;

    msg_pub.la.lane_mark_type     = 0xff; 
    msg_pub.la.lane_mark_quality  = quality_L; 
    msg_pub.la.lane_mark_position = left_pos;
    msg_pub.la.lane_mark_model_a  = left_model_a;
    msg_pub.la.lane_mark_width    = 0xff;

    msg_pub.ra.lane_mark_type     = 0xff; 
    msg_pub.ra.lane_mark_quality  = quality_R; 
    msg_pub.ra.lane_mark_position = right_pos;
    msg_pub.ra.lane_mark_model_a  = right_model_a;
    msg_pub.ra.lane_mark_width    = 0xff;

    msg_pub.lb.lane_mark_heading_angle                 = left_heading;
    msg_pub.lb.lane_mark_model_view_range              = left_view_range_m;
    msg_pub.lb.lane_mark_model_view_range_availability = left_view_validity;
    msg_pub.lb.lane_mark_model_da                      = left_model_da;
    msg_pub.lb.lane_start                              = left_lane_start_m;
    msg_pub.lb.lane_end                                = left_lane_end_m;

    msg_pub.rb.lane_mark_heading_angle                 = right_heading;
    msg_pub.rb.lane_mark_model_view_range              = right_view_range_m;
    msg_pub.rb.lane_mark_model_view_range_availability = right_view_validity;
    msg_pub.rb.lane_mark_model_da                      = right_model_da;
    msg_pub.rb.lane_start                              = right_lane_start_m;
    msg_pub.rb.lane_end                                = right_lane_end_m;

    msg_pub.add.rolling_counter              = rolling_counter_;
    msg_pub.add.rh_guardrail                 = false;
    msg_pub.add.lh_guardrail                 = false;
    msg_pub.add.right_lane_color_information = 0xff;
    msg_pub.add.left_lane_color_information  = 0xff;

    // return msg_pub;
  }

};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LaneDetectionNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
