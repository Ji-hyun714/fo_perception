/* CPP */
#include <cstdio>
#include <chrono>
#include <string>
#include <functional> // std::bind, std::placeholders
#include <memory>     // std::make_shared
#include <unordered_map>
#include <cmath>

/* ROS2 */
#include "rclcpp/rclcpp.hpp"
#include "fo_msgs/msg/cam2_tdtlod.hpp"
#include "fo_msgs/msg/cam2_data_for_sf2.hpp"
#include "yolo_msgs/msg/detection_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

/* Utils */
#include "opencv2/opencv.hpp"
// #include "fo_cam2/fo_struct.hpp"
// #include "fo_cam2/lane_detection.hpp"
#include "fo_cam2/camera_geometry.hpp"
// #include "fo_cam2/cal.hpp"
#include <yaml-cpp/yaml.h>

#define CAM_TRACK_NUM 16

bool MODEL_yolov11m = true;
// bool MODEL_yolov11m = false; // g70 모델일 때
/* 클래스 ID 매핑
  0: person        -> 0x4
  1: bicycle       -> 0x3
  2: car           -> 0x1
  3: motorcycle    -> 0x3
  5: bus           -> 0x2
  7: truck         -> 0x2
  9: traffic light -> X
*/



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

  calib = CamCalib::Make(K_rect, D, R, t);

  std::cout << "--- Parsed Calibration Data ---" << std::endl;
  std::cout << "Rectified Camera Matrix (K_rect):\n" << calib.K << std::endl;
  std::cout << "Distortion (D):\n" << calib.D << std::endl;
  std::cout << "Rotation (R):\n" << calib.R_w2c << std::endl;
  std::cout << "Translation (t):\n" << calib.t_w2c << std::endl;
  std::cout << "---------------------------------" << std::endl;
  
  return calib;
}

class Cam2TDTLODInterface : public rclcpp::Node
{
public:
  // LaneDetector detector;
  bool debug_mode;

  Cam2TDTLODInterface()
      : Node("Cam2TDTLODInterface")
  {
    RCLCPP_INFO(this->get_logger(), "Cam2TDTLODInterface init");

    this->declare_parameter<std::string>("input_topic", "/yolo/tracking");
    this->declare_parameter<std::string>("input_topic_dbgimg", "/yolo/dbg_image");
    this->declare_parameter<std::string>("output_topic", "/cam2_tdtlod");
    this->declare_parameter<std::string>("output_topic_sf", "/cam2/sf_objs");
    this->declare_parameter<bool>("debug_mode", false);

    std::string yp = "/home/wise/fo_perception/ws_cam2/src/camera_stream/yaml/cam_parameters_220.yaml";
    std::string yaml_path = this->declare_parameter<std::string>("yaml_path",yp);
    calib = getCalibPramasFromYaml(yaml_path);
    
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string input_topic_dbgimg = this->get_parameter("input_topic_dbgimg").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    std::string output_topic_sf = this->get_parameter("output_topic_sf").as_string();
    debug_mode = this->get_parameter("debug_mode").as_bool();

    pub_ = this->create_publisher<fo_msgs::msg::Cam2TDTLOD>(output_topic, 10);
    pub_sf = this->create_publisher<fo_msgs::msg::Cam2DataForSF2>(output_topic_sf, 10);

    if (debug_mode)
    {
      RCLCPP_INFO(this->get_logger(), "Debug mode ENABLED. Synchronizing image and detections.");

      // 1. message_filters::Subscriber 객체 생성
      detection_sub_filter_ = std::make_shared<message_filters::Subscriber<yolo_msgs::msg::DetectionArray>>(this, input_topic);
      image_sub_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, input_topic_dbgimg);

      // 2. ApproximateTime 동기화기 생성
      const int queue_size = 10;
      synchronizer_ = std::make_shared<Synchronizer>(
        ApproximateSyncPolicy(queue_size), 
        *detection_sub_filter_, *image_sub_filter_);
      synchronizer_->setAgePenalty(0.1); // 오래된 메시지에 대한 페널티 (선택 사항)
      
      // 3. 동기화된 메시지를 받을 "새로운" 콜백 함수 등록
      synchronizer_->registerCallback(&Cam2TDTLODInterface::callbackDebugImage, this);

    } else {
      RCLCPP_INFO(this->get_logger(), "Debug mode DISABLED. Subscribing to detections only.");

      sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
        input_topic,
        10,
        std::bind(&Cam2TDTLODInterface::callbackPostProcess, this, std::placeholders::_1));
    }
    
    /* for class mapping */
    if (MODEL_yolov11m) {

      CLASS_MAP = {
        // {0, 0},  // person        → TD
        // {1, 0},  // bicycle       → TD
        // {2, 0},  // car           → TD
        // {3, 0},  // motorcycle    → TD
        // {5, 0},  // bus           → TD
        // {7, 0},  // truck         → TD
        // {9, 1}   // traffic light → TL

        0,   // 0 person        → TD 
        0,   // 1 bicycle       → TD
        0,   // 2 car           → TD
        0,   // 3 motorcycle    → TD
        0xF, // 4
        0,   // 5 bus           → TD
        0xF, // 6 
        0,   // 7 truck         → TD
        0xF, // 8 
        1,   // 9 traffic light → TL 
        0xF, //  
        0xF, //  
        0xF, //  
        0xF, //  
        0xF  //  
        };
      
      // ✅ 2차 매핑: TD 세부 타입
      CAM2_TD_LABEL_MAP = {
        {0, 0x4},  // person      → Type 4
        {1, 0x3},  // bicycle     → Type 3
        {2, 0x1},  // car         → Type 1
        {3, 0x3},  // motorcycle  → Type 3
        {5, 0x2},  // bus         → Type 2
        {7, 0x2}   // truck       → Type 2
        };

      VALID_CLASSES = {0, 1, 2, 3, 5, 7, 9}; // 성능 최적화용
      CAM2_OD_LABEL_MAP.clear();             // OD는 일단 사용 안함 -> 보류

    } else {

      CLASS_MAP = { /* TD: 0, TL: 1, OD:2 */
        2, // 0  : cone           
        2, // 1  : stick          
        2, // 2  : stopline       
        2, // 3  : crossline      
        1, // 4  : red_light      
        1, // 5  : yellow_light   
        1, // 6  : green_light    
        1, // 7  : str_and_left   
        1, // 8  : left_turn      
        2, // 9  : drum           
        0, // 10 : bus            
        0, // 11 : bicycle        
        0, // 12 : truck          
        0, // 13 : car            
        0, // 14 : person       
      };

      CAM2_TD_LABEL_MAP = {
        { 10, 2 },      //bus
        { 11, 3 },      //bicycle
        { 12, 2 },      //truck
        { 13, 1 },      //car
        { 14, 4 }       //person
      };

      CAM2_OD_LABEL_MAP = {
        { 0, 0x1 },      //cone
        { 1, 0x2 },      //stick
        { 2, 0x3 },      //stopline
        { 3, 0x4 },      //crossline
        { 9, 0x5 }       //drum
      };
    }


    idx_td.reserve(CAM_TRACK_NUM); // CAM_TRACK_NUM : 16
    idx_tl.reserve(CAM_TRACK_NUM);
    idx_od.reserve(CAM_TRACK_NUM);

  }
  
  ~Cam2TDTLODInterface()
  {
  }
  
private:
  
  rclcpp::Publisher<fo_msgs::msg::Cam2TDTLOD>::SharedPtr pub_;
  rclcpp::Publisher<fo_msgs::msg::Cam2DataForSF2>::SharedPtr pub_sf;
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr sub_;
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_dbgimg;
  
  /* debug mode */
  std::shared_ptr<message_filters::Subscriber<yolo_msgs::msg::DetectionArray>> detection_sub_filter_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_filter_;

  using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<yolo_msgs::msg::DetectionArray, sensor_msgs::msg::Image>;
  using Synchronizer = message_filters::Synchronizer<ApproximateSyncPolicy>;
  std::shared_ptr<Synchronizer> synchronizer_;

  std::array<uint8_t, 15> CLASS_MAP;
  // std::unordered_map<uint8_t, uint8_t> CLASS_MAP;
  std::unordered_set<uint8_t> VALID_CLASSES;

  std::unordered_map<uint8_t, uint8_t> CAM2_TD_LABEL_MAP;
  std::unordered_map<uint8_t, uint8_t> CAM2_OD_LABEL_MAP;
  
  // cv_bridge::CvBridge bridge;
  cv::Mat frame, dbg_img;

  uint8_t rolling_counter=0;
  
  std::vector<int> idx_td, idx_tl, idx_od;
  float THRES_VALID_TL = 0.3;
  float THRES_VALID_TD = 0.25;

  float img_width = 1280.0;
  float img_height = 720.0; // TODO: yaml에서 파싱하도록

  CamCalib calib;


  void callbackPostProcess(const yolo_msgs::msg::DetectionArray::SharedPtr msg) // no debug mode
  {
    auto msg_pub = std::make_unique<fo_msgs::msg::Cam2TDTLOD>();
    build_msg_from_detection(msg, *msg_pub);
    
    pub_->publish(std::move(msg_pub));

    rolling_counter++; // 0~255 순환
  }
  
  void build_msg_from_detection(const yolo_msgs::msg::DetectionArray::SharedPtr& msg_in, 
                                      fo_msgs::msg::Cam2TDTLOD& msg_out)
  {
    // msg_out.header = msg_in->header;
    
    idx_td.clear();
    idx_tl.clear();
    idx_od.clear();

    if (!msg_in->detections.empty()) {

      /* 클래스 분류 - coco */
      for (size_t idx = 0; idx < msg_in->detections.size(); ++idx) {

        const auto& det = msg_in->detections[idx];

        // if (det.class_id < CLASS_MAP.size()) {
        // if (VALID_CLASSES.find(det.class_id) == VALID_CLASSES.end()) { // 클래스 필터링
          
        if (MODEL_yolov11m) {
          if (VALID_CLASSES.find(det.class_id) == VALID_CLASSES.end()) {
            continue;  // 찾지 못했으면 skip
          }
        }

        uint8_t mapped = CLASS_MAP[det.class_id];
        switch (mapped) {
          case 0: 
            idx_td.push_back(idx); break;
          case 1: 
            idx_tl.push_back(idx); break;
          case 2:
            idx_od.push_back(idx); break;
          case 0xF:
            continue;
          default: 
            break;
        }
        // } 

      }

      if (debug_mode) {
        // RCLCPP_INFO(this->get_logger(), "Detections(TD/TL/OD): %zu(%zu/%zu/%zu)", 
        //             msg_in->detections.size(), idx_td.size(), idx_tl.size(), idx_od.size()
        //             );    
        printf("Detections(TD/TL/OD): %zu(%zu/%zu/%zu)", 
                    msg_in->detections.size(), idx_td.size(), idx_tl.size(), idx_od.size());
      }
      
      auto msg2sf = std::make_unique<fo_msgs::msg::Cam2DataForSF2>();
      // msg2sf->header.stamp = this->get_clock()->now();

      msg2sf->header = msg2sf->header;

      /* TD, TL, OD 개별 후처리 */
      processTD(msg_in, msg_out, *msg2sf);
      processTL(msg_in, msg_out);      
      processOD(msg_in, msg_out, *msg2sf); // TD->OD 

      pub_sf->publish(std::move(msg2sf));
    
    } else { // msg 비어있으면 

      auto msg2sf = std::make_unique<fo_msgs::msg::Cam2DataForSF2>();

      msg2sf->header = msg2sf->header;
      pub_sf->publish(std::move(msg2sf));

    }
  }

  void processTD(const yolo_msgs::msg::DetectionArray::SharedPtr& msg_in, 
                       fo_msgs::msg::Cam2TDTLOD& msg_out,
                       fo_msgs::msg::Cam2DataForSF2& msg2sf
                      )
  {
    if (debug_mode) {
      // RCLCPP_INFO(this->get_logger(), "Processing %zu TD objects...", idx_td.size());
    }

    // fo_msgs::msg::Cam2DataForSF2& msg2sf;

    for (uint8_t i: idx_td)
    {
      const auto& det = msg_in->detections[i];

      /* TODO: 슬롯 관리 -> 슬롯 인덱스 추출 */
      // uint8_t si = assign_td_slot(det.id);
      // if (si == -1) { // 슬롯 지금 없으면 패스?
      //     continue; 
      // }
      // auto& td_a = msg_out.td.tda[si];
      // auto& td_b = msg_out.td.tdb[si];
      auto& td_a = msg_out.td.tda[i];
      auto& td_b = msg_out.td.tdb[i];

      /* coord conversion */
      cv::Point2d foot{det.bbox.center.position.x, det.bbox.center.position.y + (det.bbox.size.y / 2.0)};
      cv::Point2d p_gnd = pixToGndUndist(foot, calib); // origin point?
      
      /* 본네트 필터링 */
      if (foot.y > img_height/2 && det.bbox.size.x > 800.0) {
        continue;
      } 
      
      // double range, angle;
      auto [range, angle] = getRangeAngle(p_gnd);

      if (debug_mode) {
        cv::circle(dbg_img, foot, 3, cv::Scalar(0,0,255), -1);

        std::string text_to_show = "gnd(m) X:" + std::to_string(p_gnd.x) 
                                        + " Y:" + std::to_string(p_gnd.y);
        cv::Point2d text_position = foot + cv::Point2d(0, 20);

        cv::putText(dbg_img, text_to_show, text_position, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 2);

        RCLCPP_INFO(this->get_logger(), "() ground coord X: %2dm Y: %2dm", p_gnd.x, p_gnd.y);
      }
      
      /* msg2sf */
      // auto& obj = msg2sf.objs_td[i];
      fo_msgs::msg::Obj2SF obj;

      obj.cam2_track_id = std::stoi(det.id)%255;
      // obj.object_type = det.class_id; 
      obj.object_type = CAM2_TD_LABEL_MAP.at(det.class_id); 
      obj.confidence = det.score;
      obj.pos_x = p_gnd.x;
      obj.pos_y = p_gnd.y;

      msg2sf.objs_td.push_back(obj);
      
      /* fill payload */
      td_a.rolling_counter = rolling_counter;
      td_a.object_age = 0xff;
      td_a.angle_rate = 0xff; 
      if (angle>0) { // counterclockwise?
        td_a.angle_left = std::abs(angle);
        td_a.angle_right = 0;
      } else {
        td_a.angle_left = 0;
        td_a.angle_right = std::abs(angle);
      }
      // td_a.angle_left = 0xff;
      // td_a.angle_right = 0xff;
      td_a.motion_status = 0xff;
      td_a.object_lane = 0xff;
      td_a.cam2_obstacle_brake_lights = false;

      td_b.rolling_counter = rolling_counter;
      td_b.range = range;
      td_b.object_vaildity = (det.score>THRES_VALID_TD? 1:0);
      td_b.range_rate = 0xff;
      td_b.cam2_obstacle_physical_width = 0xff;
      td_b.cam2_track_id = std::stoi(det.id)%255;
      td_b.object_type = CAM2_TD_LABEL_MAP.at(det.class_id);
    }
  }

  std::pair<double, double> getRangeAngle(cv::Point2d p_gnd)
  {
    double range = 0.0;
    double angle = 0.0;

    double x = p_gnd.x;
    double y = p_gnd.y;

    range = std::sqrt(x * x + y * y);
    angle = std::atan2(y, x); // TODO: varify it!

    return {range, angle}; 
  }

  void processTL(const yolo_msgs::msg::DetectionArray::SharedPtr& msg_in, 
                       fo_msgs::msg::Cam2TDTLOD& msg_out)
  {

    if (debug_mode) {
      // RCLCPP_INFO(this->get_logger(), "Processing %zu TL objects...", idx_tl.size());
    }

    if (idx_tl.empty()) {
      return;
    }
    
    if (MODEL_yolov11m) {
      // ✅ YOLOv11m: 가장 confident한 신호등 선택
      uint8_t best_idx = idx_tl[0];
      float best_score = msg_in->detections[best_idx].score;
      
      for (uint8_t i : idx_tl) {
        if (msg_in->detections[i].score > best_score) {
          best_idx = i;
          best_score = msg_in->detections[i].score;
        }
      }
      
      const auto& det = msg_in->detections[best_idx];
      
      // ⚠️ COCO traffic light는 색상 정보 없음!
      msg_out.tl.traffic_light_data = 0;  // Unknown (또는 별도 분류기 필요)
      msg_out.tl.traffic_light_accuracy = det.score;
      msg_out.tl.traffic_light_valid_flag = (det.score > THRES_VALID_TL);
      
      if (debug_mode) {
        RCLCPP_WARN_ONCE(this->get_logger(), 
          "YOLOv11m: Traffic light detected but no color info available");
      }
    } else {
    
      uint8_t best_idx = idx_tl[0];
      float best_y = msg_in->detections[best_idx].bbox.center.position.y;
      
      for (uint8_t i : idx_tl) {
        float curr_y = msg_in->detections[i].bbox.center.position.y;
        if (curr_y < best_y) {  // 더 위쪽 (y 작을수록 위)
          best_idx = i;
          best_y = curr_y;
        }
      }

      for (uint8_t i: idx_tl)
      {
        // const auto& det = msg_in->detections[i];

        // /* TODO: y좌표 필터링, 비교 로직 추가 */

        // msg_out.tl.traffic_light_data = det.class_id-3;
        // msg_out.tl.traffic_light_accuracy = det.score;
        // msg_out.tl.traffic_light_valid_flag = (det.score>THRES_VALID_TL? true:false);

        const auto& det = msg_in->detections[best_idx];
        
        msg_out.tl.traffic_light_data = det.class_id - 3;
        msg_out.tl.traffic_light_accuracy = det.score;
        msg_out.tl.traffic_light_valid_flag = (det.score > THRES_VALID_TL);
      }
    }

  }

  void processOD(const yolo_msgs::msg::DetectionArray::SharedPtr& msg_in, 
                       fo_msgs::msg::Cam2TDTLOD& msg_out,
                       fo_msgs::msg::Cam2DataForSF2& msg2sf
                      )
  {
    if (debug_mode) {
      // RCLCPP_INFO(this->get_logger(), "Processing %zu OD objects...", idx_od.size());
    }

    for (uint8_t i: idx_od)
    {
      const auto& det = msg_in->detections[i];

      /* coord conversion */
      cv::Point2d foot{det.bbox.center.position.x, det.bbox.center.position.y + (det.bbox.size.y / 2.0)};
      cv::Point2d p_gnd = pixToGndUndist(foot, calib); // TODO: Apply the offset from the agreed-upon origin point
      
      // auto& obj = msg2sf.objs_od[i];
      fo_msgs::msg::Obj2SF obj;

      obj.cam2_track_id = std::stoi(det.id)%255;
      obj.object_type = det.class_id;
      obj.confidence = det.score;
      obj.pos_x = p_gnd.x;
      obj.pos_y = p_gnd.y;

      msg2sf.objs_td.push_back(obj);

      RCLCPP_INFO(this->get_logger(), "od class_id: %d ", det.class_id);
      
      auto& od = msg_out.od.sod[i];
      od.rolling_count_1 = rolling_counter;
      od.camera2_static_obj_type = CAM2_OD_LABEL_MAP.at(det.class_id);
      od.static_object_status = 0x00;

      /*  */
      if (det.class_id==3) { // crossline
        // od.static_object_pos_y = 0x00;
        // od.static_object_pos_x = 0x00;
        od.static_object_pos2_y = p_gnd.y;
        od.static_object_pos2_x = p_gnd.x;
      } else {
        od.static_object_pos_y = p_gnd.y;
        od.static_object_pos_x = p_gnd.x;
        // od.static_object_pos2_y = 0x00;
        // od.static_object_pos2_x = 0x00;
      }
    }
  }

  int assign_td_slot(int track_id)
  {

    return 0;
  }

  void callbackDebugImage(const yolo_msgs::msg::DetectionArray::SharedPtr det_msg,
                          const sensor_msgs::msg::Image::SharedPtr img_msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Syncronized node starts.");

    if (img_msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received an empty image message. Skipping frame.");
        return;
    }

    // RCLCPP_DEBUG(get_logger(),
    //   "recv enc=%s w=%u h=%u step=%u size=%zu",
    //   img_msg->encoding.c_str(), img_msg->width, img_msg->height,
    //   img_msg->step, img_msg->data.size());

    /* dbg_img from msg */
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg); // 복사 없이

      if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
        dbg_img = cv_ptr->image; // shallow copy
      } else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(cv_ptr->image, dbg_img, cv::COLOR_RGB2BGR);
      } else if (img_msg->encoding == sensor_msgs::image_encodings::MONO8) {
        cv::cvtColor(cv_ptr->image, dbg_img, cv::COLOR_GRAY2BGR);
      } else if (img_msg->encoding == "NV12" || img_msg->encoding == "nv12") {
        cv::cvtColor(cv_ptr->image, dbg_img, cv::COLOR_YUV2BGR_NV12);
      } else {
        dbg_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
      }
    }
    catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }
    catch (const cv::Exception& e) {            
      RCLCPP_ERROR(get_logger(), "OpenCV error: %s", e.what());
      return;
    }


    /* draw ground distance guide */
    // std::vector<double> distances = {5.0, 10.0, 15.0, 20.0, 25.0};
    // double half_width = 3.0;
    // cv::Scalar line_color = cv::Scalar(0, 0, 255);
    // int line_thickness = 2;

    // for (const double& dist : distances) {
    //     // 1. 지면 좌표계에서 라인의 왼쪽 끝점과 오른쪽 끝점 정의 (x, y) -> (가로, 세로)
    //     cv::Point2d left_point_gnd(  dist, half_width);
    //     cv::Point2d right_point_gnd( dist, -half_width);

    //     // 2. 지면 좌표를 픽셀 좌표로 변환
    //     cv::Point2d left_point_pixel = gndToPix(left_point_gnd, calib);
    //     cv::Point2d right_point_pixel = gndToPix(right_point_gnd, calib);
        
    //     // 3. dbg_img에 라인 그리기
    //     cv::line(dbg_img, left_point_pixel, right_point_pixel, line_color, line_thickness);
        
    //     // 4. (선택 사항) 거리 텍스트 표시
    //     std::string dist_text = std::to_string(static_cast<int>(dist)) + "m";
    //     // 텍스트를 라인의 오른쪽에 약간의 여백을 두고 표시
    //     cv::putText(dbg_img, dist_text, right_point_pixel + cv::Point2d(10, 5), cv::FONT_HERSHEY_SIMPLEX, 0.7, line_color, 2);
    // }
    /*                           */
    std::vector<cv::Point2d> gnd_pts = {
      {5.0, 6.0}, {5.0, 4.0}, {5.0, 2.0}, {5.0, 0.0}, {5.0, -2.0}, {5.0, -4.0}, {5.0, -6.0},
      {10.0, 6.0}, {10.0, 4.0}, {10.0, 2.0}, {10.0, 0.0}, {10.0, -2.0}, {10.0, -4.0}, {10.0, -6.0},
      {15.0, 6.0}, {15.0, 4.0}, {15.0, 2.0}, {15.0, 0.0}, {15.0, -2.0}, {15.0, -4.0}, {15.0, -6.0},
      {20.0, 6.0}, {20.0, 4.0}, {20.0, 2.0}, {20.0, 0.0}, {20.0, -2.0}, {20.0, -4.0}, {20.0, -6.0},
      {25.0, 6.0}, {25.0, 4.0}, {25.0, 2.0}, {25.0, 0.0}, {25.0, -2.0}, {25.0, -4.0}, {25.0, -6.0},
      {30.0, 6.0}, {30.0, 4.0}, {30.0, 2.0}, {30.0, 0.0}, {30.0, -2.0}, {30.0, -4.0}, {30.0, -6.0} 
    };

    std::vector<cv::Point2d> pix_pts;
    pix_pts.reserve(gnd_pts.size());
    for (const auto& p : gnd_pts) {
      pix_pts.push_back(gndToPix(p, calib));
    }
    
    int num_x_points = 6; // Number of unique x-coordinates (5, 10, 15, 20, 25)
    int num_y_points = 7; // Number of unique y-coordinates (6, 4, 2, 0, -2, -4)

    // Draw vertical lines
    for (int i = 0; i < num_x_points; ++i) {
        for (int j = 0; j < num_y_points - 1; ++j) {
            int current_index = i * num_y_points + j;
            cv::line(dbg_img, pix_pts[current_index], pix_pts[current_index + 1], cv::Scalar(0, 255, 0), 2);
        }
    }

    // Draw horizontal lines
    for (int j = 0; j < num_y_points; ++j) {
        for (int i = 0; i < num_x_points - 1; ++i) {
            int current_index = i * num_y_points + j;
            int next_index = (i + 1) * num_y_points + j;
            cv::line(dbg_img, pix_pts[current_index], pix_pts[next_index], cv::Scalar(0, 255, 0), 2);
        }
    }
    
    auto msg_pub = std::make_unique<fo_msgs::msg::Cam2TDTLOD>();
    build_msg_from_detection(det_msg, *msg_pub);
    
    cv::imshow("Debug View", dbg_img);
    cv::waitKey(1);
    
    pub_->publish(std::move(msg_pub));
    rolling_counter++; // 0~255 순환
  }

};

int main(int argc, char **argv)
{
  // (void) argc;
  // (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Cam2TDTLODInterface>());
  rclcpp::shutdown();

  return 0;
}
