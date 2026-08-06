#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "detection_engine.h"
#include "lane_detection/helper_lane.hpp"
#include "lane_detection/helper_obj_builder.hpp"
#include "lane_detection/helper_tracking.hpp"
#include "lane_detection/LaneDetection.hpp"
#include "fo_msgs/msg/cam2_data.hpp"
#include "fo_msgs/msg/cam2_data_for_sf2.hpp"
#include "fo_msgs/msg/cam2_ld.hpp"
#include "fo_msgs/msg/cam2_od.hpp"
#include "fo_msgs/msg/cam2_td.hpp"
#include "fo_msgs/msg/cam2_tl.hpp"
#include "fo_msgs/msg/obj2_sf.hpp"

#include <utils/CalibData.hpp>
#include <utils/common.hpp>

using std::placeholders::_1;

namespace
{

constexpr char kWorkDir[] =
  "/home/wise/fo_perception/src/ws_cam2/src/lane_detection/yolopv2_trt/resource";
constexpr char kDefaultYamlPath[] =
  "/home/wise/fo_perception/src/ws_cam2/src/camera_stream/yaml/cam_parameters_220.yaml";
constexpr char kDefaultLdCfg[] =
  "/home/wise/fo_perception/src/ws_cam2/src/lane_detection/yaml/kcity.yaml";

// ===== 창 이름 (관리 패널 + 토글 레이어 팝업) =====
constexpr char kPanelWindowName[]   = "Control Panel";        // 관리 패널(버튼 토글)
constexpr char kRoiQuadWindowName[] = "ROI Quad Editor";      // L1 원본 + ROI 사다리꼴
constexpr char kRoiZoomWindowName[] = "ROI Zoom View";        // L2 사다리꼴 → 직사각형 확대
constexpr char kRoiBevWindowName[]  = "BEV View";             // L3 캘리브 조감도
constexpr char kLaneDebugWindowName[] = "Lane Detection Process";  // L4 파이프라인 모자이크
constexpr char kLaneInfoWindowName[]  = "Lane Info Panel";    // 정보 패널(config + result)
constexpr char kStructureWindowName[] = "Pipeline Structure";  // 전체 구조 라이브 다이어그램
constexpr char kDefaultRoiQuadPreset[] =
  "/home/wise/fo_perception/src/ws_cam2/src/lane_detection/yaml/roi_quad_preset.yaml";

// 창(레이어) → {제목, 그 화면을 만든 주요 함수 체인}. info 범례와 구조도가 공유하는 단일 소스.
struct LayerMeta
{
  std::string title;
  std::string func_chain;
};
inline const std::map<std::string, LayerMeta> & layerMetaTable()
{
  static const std::map<std::string, LayerMeta> kTable = {
    {kRoiQuadWindowName,   {"ROI Quad Editor (L1)", "last_frame -> fillConvexPoly -> drawQuadEditor"}},
    {kRoiZoomWindowName,   {"ROI Zoom View (L2)",   "roi_corners -> getPerspectiveTransform -> warpPerspective"}},
    {kRoiBevWindowName,    {"BEV View (L3)",        "GetBevHomography -> warpPerspective -> drawBevView"}},
    {kLaneDebugWindowName, {"Lane Process (L4)",    "Process -> DetectFromMask -> getDebugImage"}},
    {kLaneInfoWindowName,  {"Lane Info Panel",      "last_result + GetConfig -> renderInfoPanel"}},
    {kPanelWindowName,     {"Control Panel",        "buildPanelButtons -> drawControlPanel"}},
  };
  return kTable;
}

constexpr bool kEnableLd = true;
constexpr bool kEnableOd = true;
constexpr bool kEnableDa = false;
constexpr int kLaneClassId = 2;
constexpr int kDaClassId = 1;
struct DetectionConfig
{
  float objectness_threshold{0.30F};
  float score_threshold{0.30F};
  std::vector<int32_t> class_whitelist{0, 1, 2, 3, 5, 7, 9};
};

class Cam2DataComposer
{
public:
  fo_msgs::msg::Cam2Data build(
    const fo_msgs::msg::Cam2LD & ld,
    const fo_msgs::msg::Cam2TD & td,
    const fo_msgs::msg::Cam2TL & tl,
    const fo_msgs::msg::Cam2OD & od) const
  {
    fo_msgs::msg::Cam2Data msg;
    msg.ld = ld;
    msg.td = td;
    msg.tl = tl;
    msg.od = od;
    return msg;
  }
};

}  // namespace

class Yolopv2Cam2Node : public rclcpp::Node
{
public:
  Yolopv2Cam2Node()
  : Node("yolopv2_cam2_node"),
    tracking_helper_(tracking_config_)
  {
    this->declare_parameter<std::string>("input_topic", "/camera/image_rect");
    this->declare_parameter<std::string>("output_topic_ld", "/camera/lane_result");
    this->declare_parameter<std::string>("output_topic_cam2_data", "/camera/cam2data");
    this->declare_parameter<std::string>("output_topic_sf", "/camera/sf_objs");
    this->declare_parameter<std::string>("bbox_image_topic", "/camera/det_bboxes");
    this->declare_parameter<std::string>("yaml_path", "");
    this->declare_parameter<std::string>("ld_cfg", "");
    this->declare_parameter<bool>("debug_mode", true);
    this->declare_parameter<double>("debug_window_scale", 0.6);
    this->declare_parameter<std::string>("bag_path", "");
    this->declare_parameter<std::string>("bag_topic", "/camera/image_rect");

    const std::string input_topic = this->get_parameter("input_topic").as_string();
    const std::string output_topic_ld = this->get_parameter("output_topic_ld").as_string();
    const std::string output_topic_cam2_data = this->get_parameter("output_topic_cam2_data").as_string();
    const std::string output_topic_sf = this->get_parameter("output_topic_sf").as_string();
    const std::string bbox_image_topic = this->get_parameter("bbox_image_topic").as_string();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    debug_window_scale_ = this->get_parameter("debug_window_scale").as_double();
    if (debug_window_scale_ <= 0.0) {
      debug_window_scale_ = 1.0;
    }
    bag_path_ = this->get_parameter("bag_path").as_string();
    bag_topic_ = this->get_parameter("bag_topic").as_string();

    std::string yaml_path = this->get_parameter("yaml_path").as_string();
    if (yaml_path.empty()) {
      yaml_path = kDefaultYamlPath;
    }

    std::string ld_cfg = this->get_parameter("ld_cfg").as_string();
    if (ld_cfg.empty()) {
      ld_cfg = kDefaultLdCfg;
    }

    if (engine_.Initialize(kWorkDir, 4) != DetectionEngine::kRetOk) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize DetectionEngine. work_dir=%s", kWorkDir);
      throw std::runtime_error("DetectionEngine init failed");
    }
    engine_initialized_ = true;
    engine_.SetDetectionThresholds(
      detection_config_.objectness_threshold,
      detection_config_.score_threshold);
    engine_.SetDetectionClassWhitelist(detection_config_.class_whitelist);
    engine_.SetPostProcessConfig(kEnableLd, kEnableDa, kEnableOd, kLaneClassId, kDaClassId);

    cam_conf_ = getCameraConfigFromYaml(yaml_path);
    printCameraConfig(cam_conf_);
    if (cam_conf_.model == 0) {
      calib_ = CalibData::MakePinhole(
        cam_conf_.image_size, cam_conf_.K, cam_conf_.D, cam_conf_.R, cam_conf_.t);
    } else {
      calib_ = CalibData::MakeFisheye(
        cam_conf_.image_size, cam_conf_.K, cam_conf_.D, cam_conf_.R, cam_conf_.t);
    }

    GroundProjectorOptions projector_options;
    projector_options.min_bbox_height_px = object_builder_config_.min_bbox_height_px;
    projector_options.max_range_m = object_builder_config_.max_range_m;
    bbox_projector_ = std::make_unique<BboxGroundProjector>(calib_, projector_options);
    object_message_builder_ = std::make_unique<ObjectMessageBuilder>(calib_, object_builder_config_);

    lld_ = std::make_unique<LaneLineDetector>(calib_, ld_cfg);
    enable_lane_marker_ = lld_->GetEnableLaneMarker();
    lane_marker_frame_id_ = lld_->GetLaneMarkerFrameId();
    lane_marker_forward_min_m_ = std::max(0.0, lld_->GetLaneMarkerForwardMinM());
    lane_marker_forward_max_m_ =
      std::max(lane_marker_forward_min_m_ + 0.1, lld_->GetLaneMarkerForwardMaxM());
    lane_marker_dx_m_ = std::max(0.05, lld_->GetLaneMarkerDxM());
    lane_marker_line_width_m_ = std::max(0.01, lld_->GetLaneMarkerLineWidthM());
    lane_marker_origin_radius_m_ = std::max(0.01, lld_->GetLaneMarkerOriginRadiusM());
    lane_marker_z_m_ = lld_->GetLaneMarkerZ();

    const int qos_depth = 1;
    // bag 뷰어 모드에서는 토픽 구독 대신 bag을 직접 읽어 프레임 단위로 처리한다.
    if (bag_path_.empty()) {
      sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_topic, qos_depth, std::bind(&Yolopv2Cam2Node::callbackImage, this, _1));
    }
    pub_ld_ =
      this->create_publisher<fo_msgs::msg::Cam2LD>(output_topic_ld, qos_depth);
    pub_cam2_data_ =
      this->create_publisher<fo_msgs::msg::Cam2Data>(output_topic_cam2_data, qos_depth);
    pub_sf_ =
      this->create_publisher<fo_msgs::msg::Cam2DataForSF2>(output_topic_sf, qos_depth);

    if (debug_mode_) {
      pub_bbox_image_ =
        this->create_publisher<sensor_msgs::msg::Image>(bbox_image_topic, qos_depth);
      if (enable_lane_marker_) {
        pub_lane_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/camera/lane_marker", qos_depth);
      }
    }

    ROIQuadSetting();                       // ROI 파라미터/상태 초기화(GUI 없음)
    if (debug_mode_) {
      setupControlPanel();                  // 관리 패널 창 + 마우스 + (구독 모드면 렌더 타이머)
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Yolopv2Cam2Node started. input=%s lane_result=%s cam2_data=%s sf=%s debug_mode=%s bag=%s",
      input_topic.c_str(),
      output_topic_ld.c_str(),
      output_topic_cam2_data.c_str(),
      output_topic_sf.c_str(),
      debug_mode_ ? "true" : "false",
      bag_path_.empty() ? "(live)" : bag_path_.c_str());
  }

  ~Yolopv2Cam2Node() override
  {
    if (debug_mode_) {
      cv::destroyAllWindows();
    }
    if (engine_initialized_) {
      engine_.Finalize();
    }
  }

  bool isBagMode() const { return !bag_path_.empty(); }

  // ================= bag 프레임 단위 뷰어 =================
  // trackbar 콜백: 사용자가 프레임 위치를 드래그하면 seek 요청 플래그만 세운다.
  static void onTrackbar(int /*pos*/, void * userdata)
  {
    auto * self = static_cast<Yolopv2Cam2Node *>(userdata);
    if (self) {
      self->bag_seek_requested_ = true;
    }
  }

  // bag을 직접 읽어 프레임 단위로 재생/탐색. 관리 패널·레이어 팝업을 루프 안에서 렌더한다.
  // 조작: Space(재생/정지) ←·→(이전·다음) ↑·↓ 또는 ,·.(배속) 0-9+Enter(점프) q/Esc(종료)
  //       + ROI 키(p/o/z/b/r/s/l)와 관리 패널 마우스 클릭
  void runBagViewer()
  {
    rosbag2_cpp::Reader reader;
    try {
      reader.open(bag_path_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Failed to open bag '%s': %s", bag_path_.c_str(), e.what());
      return;
    }

    rosbag2_storage::StorageFilter filter;
    filter.topics.push_back(bag_topic_);
    reader.set_filter(filter);

    std::vector<int64_t> stamps;
    while (rclcpp::ok() && reader.has_next()) {
      auto bag_msg = reader.read_next();
      stamps.push_back(bag_msg->time_stamp);
    }
    const int total = static_cast<int>(stamps.size());
    if (total == 0) {
      RCLCPP_FATAL(
        get_logger(), "No messages on topic '%s' in bag '%s'.",
        bag_topic_.c_str(), bag_path_.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Bag viewer: %d frames on '%s'.", total, bag_topic_.c_str());

    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;

    auto loadFrame = [&](int i, cv::Mat & out) -> bool {
      i = std::clamp(i, 0, total - 1);
      try {
        reader.seek(stamps[i]);
        if (!reader.has_next()) {
          return false;
        }
        auto bag_msg = reader.read_next();
        rclcpp::SerializedMessage extracted(*bag_msg->serialized_data);
        auto img = std::make_shared<sensor_msgs::msg::Image>();
        serialization.deserialize_message(&extracted, img.get());
        out = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8)->image;
        return !out.empty();
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "loadFrame(%d) failed: %s", i, e.what());
        return false;
      }
    };

    // 프레임 탐색 트랙바는 관리 패널 창에 얹는다(재생 컨트롤과 토글을 한 창에).
    bag_trackbar_pos_ = 0;
    cv::createTrackbar(
      "frame", kPanelWindowName, &bag_trackbar_pos_, total - 1,
      &Yolopv2Cam2Node::onTrackbar, this);

    int idx = 0;
    int shown_idx = -1;
    bool playing = false;
    double speed = 1.0;
    std::string numbuf;
    int last_key = -1;

    while (rclcpp::ok()) {
      if (bag_seek_requested_) {
        idx = std::clamp(bag_trackbar_pos_, 0, total - 1);
        bag_seek_requested_ = false;
      }
      // stage 토글 등으로 현재 프레임 재처리가 필요하면 다시 로드
      if (reprocess_current_) {
        shown_idx = -1;
        reprocess_current_ = false;
      }

      if (idx != shown_idx) {
        cv::Mat frame;
        if (loadFrame(idx, frame)) {
          runFramePipeline(frame, makeHeader(stamps[idx]));
        }
        shown_idx = idx;
        if (bag_trackbar_pos_ != idx) {
          cv::setTrackbarPos("frame", kPanelWindowName, idx);
        }
        needs_redraw_ = true;
      }

      renderAllViews(true, idx, total, playing, speed, numbuf, last_key);

      const int delay = playing ? std::max(1, static_cast<int>(33.0 / std::max(0.1, speed))) : 30;
      const int key = cv::waitKeyEx(delay);
      if (key != -1) {
        last_key = key;
        needs_redraw_ = true;
        overlay_dirty_ = true;   // 재생/탐색/GOTO 키 → info 오버레이 갱신
      }

      // 화살표: raw 코드로 먼저 판별 (마스킹 전에)
      auto isLeft = [](int k) { return k == 65361 || k == 2424832 || k == 63234 || k == 0x51; };
      auto isRight = [](int k) { return k == 65363 || k == 2555904 || k == 63235 || k == 0x53; };
      auto isUp = [](int k) { return k == 65362 || k == 2490368 || k == 63232 || k == 0x52; };
      auto isDown = [](int k) { return k == 65364 || k == 2621440 || k == 63233 || k == 0x54; };
      const int ch = key & 0xFF;

      if (isLeft(key)) {                 // ← 이전 프레임
        playing = false;
        idx = std::max(0, idx - 1);
      } else if (isRight(key)) {         // → 다음 프레임
        playing = false;
        idx = std::min(total - 1, idx + 1);
      } else if (isUp(key)) {            // ↑ 배속 +0.1
        speed = std::min(4.0, std::round((speed + 0.1) * 10.0) / 10.0);
      } else if (isDown(key)) {          // ↓ 배속 -0.1
        speed = std::max(0.1, std::round((speed - 0.1) * 10.0) / 10.0);
      } else if (ch == ',') {            // , 배속 -0.1
        speed = std::max(0.1, std::round((speed - 0.1) * 10.0) / 10.0);
      } else if (ch == '.') {            // . 배속 +0.1
        speed = std::min(4.0, std::round((speed + 0.1) * 10.0) / 10.0);
      } else if (ch == 'q' || ch == 27) {
        break;
      } else if (ch == ' ') {
        playing = !playing;
      } else if (ch >= '0' && ch <= '9') {
        numbuf.push_back(static_cast<char>(ch));
        const size_t max_digits = std::to_string(std::max(0, total - 1)).size();
        if (numbuf.size() > max_digits) {
          numbuf.erase(numbuf.begin());
        }
      } else if (ch == 8 || ch == 127) {              // Backspace
        if (!numbuf.empty()) {
          numbuf.pop_back();
        }
      } else if (ch == 13 || ch == 10) {              // Enter → GOTO
        if (!numbuf.empty()) {
          idx = std::clamp(static_cast<int>(std::strtol(numbuf.c_str(), nullptr, 10)), 0, total - 1);
          numbuf.clear();
          playing = false;
        }
      } else if (key != -1) {
        handleQuadKey(key);   // ROI 키(p/o/z/b/r/s/l) — 뷰어 전용 키가 아니면 여기로
      } else if (playing) {
        idx = (idx + 1 <= total - 1) ? idx + 1 : 0;   // 끝에서 처음으로 루프
      }
    }
    cv::destroyAllWindows();
  }

private:
  std_msgs::msg::Header makeHeader(int64_t stamp_ns) const
  {
    std_msgs::msg::Header h;
    h.frame_id = lane_marker_frame_id_;
    h.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    h.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    return h;
  }

  void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty frame");
      return;
    }
    // 렌더/키/마우스는 onUiTimer(30ms)가 담당 → 여기선 파이프라인만.
    runFramePipeline(frame, msg->header);
  }

  // 한 프레임 전체 파이프라인. 구독/bag 뷰어가 공유.
  //   PRE crop(추론 입력) → 추론 → POST crop(lane_mask) → 차선/객체 검출 → 발행.
  //   결과는 멤버(last_frame_/last_result_/last_debug_image_)에 저장해 렌더가 재사용.
  void runFramePipeline(const cv::Mat & frame, const std_msgs::msg::Header & header)
  {
    if (frame.empty()) {
      return;
    }
    if (debug_mode_) {
      last_frame_ = frame.clone();   // 편집/Zoom/BEV 배경용 원본(크롭 전)
      needs_redraw_ = true;
      frame_updated_ = true;         // info/BEV 캐시 갱신 트리거(무거운 뷰는 새 프레임에만 재생성)
    }
    if (img_w_ != frame.cols || img_h_ != frame.rows) {
      rescaleQuadToImage(frame.cols, frame.rows);  // 프레임 크기 변경 시 quad 비례 조정
    }

    // [PRE crop] ('p') 추론 입력 자체를 사다리꼴 안쪽만 남기고 크롭.
    //   toCvShare 는 ROS 버퍼와 공유메모리 → 반드시 clone 후 크롭.
    cv::Mat infer_input = frame;
    if (roi_pre_enabled_) {
      infer_input = frame.clone();
      cropToQuad(infer_input);
    }

    DetectionEngine::Result infer_result;
    if (engine_.Process(infer_input, infer_result) != DetectionEngine::kRetOk) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Inference failed for one frame");
      return;
    }

    // [POST crop] ('o') 추론은 풀프레임으로 돌린 뒤 결과 lane_mask 에서만 바깥 제거.
    if (roi_post_enabled_ && !infer_result.mat_lane_mask.empty()) {
      cropToQuad(infer_result.mat_lane_mask);
    }

    LaneResult lane_result{};
    if (infer_result.mat_lane_mask.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "lane_mask is empty");
    } else {
      lane_result = lld_->DetectFromMask(infer_result.mat_lane_mask, frame);
      if (debug_mode_) {
        last_debug_image_ = lld_->getDebugImage();   // L4/L5 파이프라인 모자이크(show_* 반영)
      }
      publishLaneMarkers(lane_result, header.stamp);
    }
    last_result_ = lane_result;

    auto cam2_ld = buildCam2LD(lane_result, header, "yolopv2_cam2_node", rolling_counter_);

    const auto tracked_objects = tracking_helper_.run(infer_result.bbox_list);
    const auto routed_objects = object_message_builder_->routeTrackedObjects(tracked_objects);

    const auto cam2_td = object_message_builder_->buildCam2TD(
      routed_objects.td_objects, rolling_counter_);
    const auto cam2_tl = object_message_builder_->buildCam2TL(routed_objects.tl_objects);
    const auto cam2_od = object_message_builder_->buildCam2OD(
      routed_objects.od_objects, rolling_counter_);

    const auto cam2_data = cam2_data_composer_.build(cam2_ld, cam2_td, cam2_tl, cam2_od);
    const auto sf_msg = object_message_builder_->buildSfMsg(
      routed_objects.td_objects, routed_objects.od_objects, header);

    pub_cam2_data_->publish(cam2_data);
    pub_ld_->publish(cam2_ld);
    pub_sf_->publish(sf_msg);

    if (debug_mode_) {
      publishDebugBboxImage(frame, tracked_objects, header);
    }

    rolling_counter_ = static_cast<uint8_t>((rolling_counter_ + 1U) % 255U);
  }

  void publishDebugBboxImage(
    const cv::Mat & frame,
    const std::vector<TrackedObject> & tracked_objects,
    const std_msgs::msg::Header & header) const
  {
    if (!pub_bbox_image_ || frame.empty()) {
      return;
    }

    cv::Mat vis = frame.clone();
    for (const auto & obj : tracked_objects) {
      drawTrackedOverlay(vis, obj);
    }

    auto msg =
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, vis).toImageMsg();
    pub_bbox_image_->publish(*msg);
  }

  void drawTrackedOverlay(cv::Mat & image, const TrackedObject & obj) const
  {
    if (image.empty()) {
      return;
    }

    int x = static_cast<int>(std::round(obj.x));
    int y = static_cast<int>(std::round(obj.y));
    int w = static_cast<int>(std::round(obj.w));
    int h = static_cast<int>(std::round(obj.h));

    x = std::clamp(x, 0, image.cols - 1);
    y = std::clamp(y, 0, image.rows - 1);
    w = std::max(0, std::min(w, image.cols - x));
    h = std::max(0, std::min(h, image.rows - y));
    if (w <= 0 || h <= 0) {
      return;
    }

    const cv::Rect rect(x, y, w, h);
    cv::rectangle(image, rect, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    if (bbox_projector_) {
      const cv::Rect2d bbox_rect(
        static_cast<double>(obj.x),
        static_cast<double>(obj.y),
        static_cast<double>(obj.w),
        static_cast<double>(obj.h));
      const auto estimate = bbox_projector_->EstimateRect(bbox_rect);
      if (estimate.valid) {
        const cv::Point foot_px(
          static_cast<int>(std::round(std::clamp(
            estimate.foot_px.x, 0.0, static_cast<double>(image.cols - 1)))),
          static_cast<int>(std::round(std::clamp(
            estimate.foot_px.y, 0.0, static_cast<double>(image.rows - 1)))));
        cv::circle(image, foot_px, 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);

        std::ostringstream pos_ss;
        pos_ss << "id=" << obj.track_id << " ("
               << std::fixed << std::setprecision(2)
               << estimate.ground_xy_m.x << ", " << estimate.ground_xy_m.y << ")";
        cv::putText(
          image,
          pos_ss.str(),
          cv::Point(x, std::max(0, y - 4)),
          cv::FONT_HERSHEY_SIMPLEX,
          0.42,
          cv::Scalar(0, 255, 255),
          1,
          cv::LINE_AA);
      }
    }
  }

  void publishLaneMarkers(const LaneResult & res, const builtin_interfaces::msg::Time & stamp)
  {
    if (!debug_mode_ || !enable_lane_marker_ || !pub_lane_marker_) {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.reserve(6);

    auto buildDeleteMarker = [&](int id, const std::string & ns) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = lane_marker_frame_id_;
      m.header.stamp = stamp;
      m.ns = ns;
      m.id = id;
      m.action = visualization_msgs::msg::Marker::DELETE;
      return m;
    };

    auto buildLaneLineMarker = [&](const cv::Vec4d & coeffs, double lane_start_m,
                                   double lane_end_m, const std::string & ns, int id, float r,
                                   float g, float b) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = lane_marker_frame_id_;
      m.header.stamp = stamp;
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = lane_marker_line_width_m_;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 1.0;

      const double start_m = std::max(lane_marker_forward_min_m_, lane_start_m);
      const double end_m = std::min(lane_marker_forward_max_m_, lane_end_m);
      if (end_m <= start_m) {
        return m;
      }

      for (double x = start_m; x <= end_m; x += lane_marker_dx_m_) {
        const double y = coeffs[0] * x * x * x +
          coeffs[1] * x * x +
          coeffs[2] * x +
          coeffs[3];
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = lane_marker_z_m_;
        m.points.push_back(std::move(p));
      }
      return m;
    };

    auto buildAxisArrow = [&](int id, float r, float g, float b, double ex, double ey, double ez) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = lane_marker_frame_id_;
      m.header.stamp = stamp;
      m.ns = "lane_axes";
      m.id = id;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.08;
      m.scale.y = 0.16;
      m.scale.z = 0.20;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 1.0;

      geometry_msgs::msg::Point p0;
      p0.x = 0.0;
      p0.y = 0.0;
      p0.z = lane_marker_z_m_;
      geometry_msgs::msg::Point p1;
      p1.x = ex;
      p1.y = ey;
      p1.z = lane_marker_z_m_ + ez;
      m.points.push_back(std::move(p0));
      m.points.push_back(std::move(p1));
      return m;
    };

    const uint8_t quality_l = ::laneStateToQuality(res.state_L);
    const uint8_t quality_r = ::laneStateToQuality(res.state_R);

    if (quality_l >= 2) {
      auto left = buildLaneLineMarker(
        res.coeffs_L,
        std::max(0.0, res.lane_start_m_L),
        std::max(0.0, res.lane_end_m_L),
        "lane_marker", 0, 0.0F, 0.0F, 1.0F);
      if (left.points.size() >= 2) {
        marker_array.markers.push_back(std::move(left));
      } else {
        marker_array.markers.push_back(buildDeleteMarker(0, "lane_marker"));
      }
    } else {
      marker_array.markers.push_back(buildDeleteMarker(0, "lane_marker"));
    }

    if (quality_r >= 2) {
      auto right = buildLaneLineMarker(
        res.coeffs_R,
        std::max(0.0, res.lane_start_m_R),
        std::max(0.0, res.lane_end_m_R),
        "lane_marker", 1, 1.0F, 0.0F, 0.0F);
      if (right.points.size() >= 2) {
        marker_array.markers.push_back(std::move(right));
      } else {
        marker_array.markers.push_back(buildDeleteMarker(1, "lane_marker"));
      }
    } else {
      marker_array.markers.push_back(buildDeleteMarker(1, "lane_marker"));
    }

    visualization_msgs::msg::Marker origin;
    origin.header.frame_id = lane_marker_frame_id_;
    origin.header.stamp = stamp;
    origin.ns = "lane_marker";
    origin.id = 100;
    origin.type = visualization_msgs::msg::Marker::SPHERE;
    origin.action = visualization_msgs::msg::Marker::ADD;
    origin.pose.orientation.w = 1.0;
    origin.pose.position.x = 0.0;
    origin.pose.position.y = 0.0;
    origin.pose.position.z = lane_marker_z_m_;
    const double origin_d = lane_marker_origin_radius_m_ * 2.0;
    origin.scale.x = origin_d;
    origin.scale.y = origin_d;
    origin.scale.z = origin_d;
    origin.color.r = 0.0F;
    origin.color.g = 1.0F;
    origin.color.b = 0.0F;
    origin.color.a = 1.0F;
    marker_array.markers.push_back(std::move(origin));

    marker_array.markers.push_back(buildAxisArrow(101, 1.0F, 0.0F, 0.0F, 1.5, 0.0, 0.0));
    marker_array.markers.push_back(buildAxisArrow(102, 0.0F, 1.0F, 0.0F, 0.0, 1.5, 0.0));
    marker_array.markers.push_back(buildAxisArrow(103, 0.0F, 0.0F, 1.0F, 0.0, 0.0, 1.0));

    pub_lane_marker_->publish(marker_array);
  }

  // ================= 관리 패널 (Control Panel) =================
  // rviz2 Displays 패널의 경량판. 버튼을 마우스로 클릭해 각 레이어/기능을 토글.
  struct PanelButton
  {
    cv::Rect rect;
    std::string label;
    std::function<bool()> get;      // 현재 on/off (버튼 색상용)
    std::function<void()> toggle;   // 클릭 시 동작
  };

  void buildPanelButtons()
  {
    panel_buttons_.clear();
    panel_headers_.clear();
    int y = 34;
    const int x = 10, w = 230, h = 26, gap = 5;
    auto header = [&](const std::string & t) {
      panel_headers_.emplace_back(t, y + 12);
      y += 20;
    };
    auto btn = [&](const std::string & label, std::function<bool()> g, std::function<void()> t) {
      panel_buttons_.push_back({cv::Rect(x, y, w, h), label, std::move(g), std::move(t)});
      y += h + gap;
    };

    header("LAYERS");
    btn("ROI Editor", [this] { return show_editor_; }, [this] { show_editor_ = !show_editor_; });
    btn("Zoom", [this] { return zoom_view_enabled_; }, [this] { zoom_view_enabled_ = !zoom_view_enabled_; });
    btn("BEV", [this] { return bev_view_enabled_; }, [this] { bev_view_enabled_ = !bev_view_enabled_; });
    btn("Process", [this] { return show_process_; }, [this] { show_process_ = !show_process_; });
    btn("Info Panel", [this] { return show_info_; }, [this] { show_info_ = !show_info_; });
    btn("Legend", [this] { return show_legend_; }, [this] { show_legend_ = !show_legend_; });
    btn("Structure", [this] { return show_structure_; }, [this] { show_structure_ = !show_structure_; });

    header("ROI CROP");
    btn("PRE crop", [this] { return roi_pre_enabled_; },
      [this] { roi_pre_enabled_ = !roi_pre_enabled_; syncQuadParameters(); reprocess_current_ = true; });
    btn("POST crop", [this] { return roi_post_enabled_; },
      [this] { roi_post_enabled_ = !roi_post_enabled_; syncQuadParameters(); reprocess_current_ = true; });

    header("PROCESS STAGES");
    static const char * kStages[] = {"mask", "bev", "sobel", "bev_bin", "sld_win", "hls", "contour"};
    for (const char * s : kStages) {
      const std::string key = s;
      btn(std::string("stage:") + s,
        [this, key] { return lld_ && lld_->GetShowFlag(key); },
        [this, key] {
          if (lld_) { lld_->SetShowFlag(key, !lld_->GetShowFlag(key)); }
          reprocess_current_ = true;   // 현재 프레임 다시 처리해야 모자이크에 반영됨
        });
    }
    panel_height_ = y + 6;
  }

  static void onPanelMouse(int event, int x, int y, int /*flags*/, void * userdata)
  {
    if (event != cv::EVENT_LBUTTONDOWN) {
      return;
    }
    auto * self = static_cast<Yolopv2Cam2Node *>(userdata);
    for (auto & b : self->panel_buttons_) {
      if (b.rect.contains(cv::Point(x, y))) {
        b.toggle();
        self->needs_redraw_ = true;
        self->panel_dirty_ = true;   // 버튼 색 갱신
        break;
      }
    }
  }

  cv::Mat drawControlPanel() const
  {
    cv::Mat p(panel_height_, 250, CV_8UC3, cv::Scalar(32, 32, 32));
    cv::putText(p, "CONTROL PANEL", cv::Point(10, 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    for (const auto & hd : panel_headers_) {
      cv::putText(p, hd.first, cv::Point(10, hd.second),
                  cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(150, 180, 255), 1, cv::LINE_AA);
    }
    for (const auto & b : panel_buttons_) {
      const bool on = b.get();
      cv::rectangle(p, b.rect, on ? cv::Scalar(40, 110, 40) : cv::Scalar(58, 58, 58), cv::FILLED);
      cv::rectangle(p, b.rect, on ? cv::Scalar(90, 220, 90) : cv::Scalar(110, 110, 110), 1);
      cv::putText(p, b.label, cv::Point(b.rect.x + 8, b.rect.y + 18),
                  cv::FONT_HERSHEY_SIMPLEX, 0.45,
                  on ? cv::Scalar(220, 255, 220) : cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
      cv::putText(p, on ? "ON" : "off", cv::Point(b.rect.x + b.rect.width - 36, b.rect.y + 18),
                  cv::FONT_HERSHEY_SIMPLEX, 0.45,
                  on ? cv::Scalar(90, 255, 90) : cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
    }
    return p;
  }

  void setupControlPanel()
  {
    buildPanelButtons();
    cv::namedWindow(kPanelWindowName, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(kPanelWindowName, onPanelMouse, this);
    // 구독(live) 모드는 spin 하므로 GUI 펌프/렌더를 타이머로. bag 모드는 runBagViewer 루프가 직접 처리.
    if (bag_path_.empty()) {
      quad_ui_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30), std::bind(&Yolopv2Cam2Node::onUiTimer, this));
    }
  }

  // 구독 모드 GUI 펌프: 렌더 + 키 처리(30ms).
  void onUiTimer()
  {
    if (!debug_mode_) {
      return;
    }
    renderAllViews(false, -1, 0, false, 1.0, std::string(), last_ui_key_);
    const int key = cv::waitKey(1);   // GUI 이벤트 펌프 + 키 입력
    if (key != -1) {
      last_ui_key_ = key;
      handleQuadKey(key);
      needs_redraw_ = true;
    }
  }

  // 켜진 레이어만 팝업창(WINDOW_NORMAL, 크기조절 가능)에 렌더. 꺼지면 창 파괴.
  void showLayer(const std::string & name, const cv::Mat & img)
  {
    if (img.empty()) {
      return;
    }
    if (!open_windows_.count(name)) {
      cv::namedWindow(name, cv::WINDOW_NORMAL);
      cv::resizeWindow(
        name,
        std::max(1, static_cast<int>(img.cols * debug_window_scale_)),
        std::max(1, static_cast<int>(std::min(img.rows, 1200) * debug_window_scale_)));
      open_windows_.insert(name);
      if (name == kRoiQuadWindowName) {
        cv::setMouseCallback(kRoiQuadWindowName, onQuadMouse, this);  // 편집 창 드래그
      }
    }
    cv::imshow(name, img);
  }

  void hideLayer(const std::string & name)
  {
    if (open_windows_.count(name)) {
      cv::destroyWindow(name);
      open_windows_.erase(name);
    }
  }

  // [기능 A] 레이어 좌하단에 반투명 박스로 해상도 + 주요 함수 체인 범례 오버레이.
  //   저장본(last_debug_image_ 등) 훼손/누적 방지 위해 항상 clone 위에 그려 반환.
  cv::Mat withLegend(const cv::Mat & img, const std::string & window_name) const
  {
    if (img.empty() || !show_legend_) {
      return img;
    }
    cv::Mat out = img.clone();
    const auto & table = layerMetaTable();
    const auto it = table.find(window_name);
    const std::string title = (it != table.end()) ? it->second.title : window_name;
    const std::string chain = (it != table.end()) ? it->second.func_chain : "";
    const std::string res = "res " + std::to_string(img.cols) + "x" + std::to_string(img.rows);

    const int pad = 6, lh = 16;
    const int box_w = std::min(out.cols - 8, 470);
    const int box_h = pad * 2 + lh * 3;
    const int x0 = 4;
    const int y0 = std::max(0, out.rows - box_h - 4);
    cv::Rect box = cv::Rect(x0, y0, box_w, box_h) & cv::Rect(0, 0, out.cols, out.rows);
    if (box.width <= 0 || box.height <= 0) {
      return out;
    }

    cv::Mat roi = out(box);
    cv::Mat shade(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
    cv::addWeighted(shade, 0.55, roi, 0.45, 0.0, roi);   // 반투명 검정 배경
    cv::rectangle(out, box, cv::Scalar(0, 200, 255), 1);

    cv::putText(out, title, cv::Point(x0 + pad, y0 + pad + 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    cv::putText(out, res, cv::Point(x0 + pad, y0 + pad + 12 + lh),
                cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(120, 255, 120), 1, cv::LINE_AA);
    cv::putText(out, "fn: " + chain, cv::Point(x0 + pad, y0 + pad + 12 + lh * 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
    return out;
  }

  // [기능 B] 프레임→추론→검출→발행 파이프라인을 박스+화살표로 그린 라이브 구조도.
  //   PRE/POST·L5 stage·레이어 토글의 현재 on/off 를 박스 색으로 즉시 반영.
  cv::Mat drawStructureDiagram() const
  {
    const int W = 560, H = 500;
    cv::Mat d(H, W, CV_8UC3, cv::Scalar(24, 24, 28));
    cv::putText(d, "PIPELINE STRUCTURE (live)", cv::Point(12, 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

    const cv::Scalar onCol(45, 130, 45), onEdge(90, 230, 90);
    const cv::Scalar offCol(55, 55, 60), offEdge(110, 110, 110);
    auto box = [&](int x, int y, int w, int h, const std::string & label, bool on,
                   const std::string & sub = "") {
      cv::rectangle(d, cv::Rect(x, y, w, h), on ? onCol : offCol, cv::FILLED);
      cv::rectangle(d, cv::Rect(x, y, w, h), on ? onEdge : offEdge, 1);
      cv::putText(d, label, cv::Point(x + 8, y + 18),
                  cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
      if (!sub.empty()) {
        cv::putText(d, sub, cv::Point(x + 8, y + 33),
                    cv::FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
      }
    };
    auto arrow = [&](int x, int y1, int y2) {
      cv::arrowedLine(d, cv::Point(x, y1), cv::Point(x, y2),
                      cv::Scalar(150, 150, 150), 1, cv::LINE_AA, 0, 0.3);
    };

    const int cx = 30, bw = 300, bh = 40, gap = 14;
    int y = 40;
    auto step = [&](const std::string & label, bool on, const std::string & sub, bool tail = true) {
      box(cx, y, bw, bh, label, on, sub);
      if (tail) { arrow(cx + bw / 2, y + bh, y + bh + gap); }
      y += bh + gap;
    };
    step("frame (bag / topic)", true, "loadFrame / callbackImage");
    step("PRE crop (p)", roi_pre_enabled_, "cropToQuad -> infer input");
    step("engine_.Process", true, "YOLOPv2 TRT -> mat_lane_mask / bbox");
    step("POST crop (o)", roi_post_enabled_, "cropToQuad(mat_lane_mask)");
    step("lld_->DetectFromMask", true, "getDebugImage -> Process mosaic", false);

    // L5 stage 칩: 켜진 단계만 강조
    int sy = y + 6;
    cv::putText(d, "L5 stages:", cv::Point(cx, sy + 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 180, 255), 1, cv::LINE_AA);
    static const char * kStages[] = {"mask", "bev", "sobel", "bev_bin", "sld_win", "hls", "contour"};
    int sx = cx + 96, syy = sy;
    for (const char * s : kStages) {
      const bool on = lld_ && lld_->GetShowFlag(s);
      box(sx, syy, 78, 22, s, on);
      sx += 84;
      if (sx + 78 > W - 8) { sx = cx + 96; syy += 26; }
    }
    y = syy + 40;
    box(cx, y, bw, bh, "publish", true, "cam2data / lane_result / sf_objs");

    // 우측: 레이어 토글 상태 미러
    const int lx = cx + bw + 30, lw = 160;
    cv::putText(d, "LAYERS", cv::Point(lx, 52),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(150, 180, 255), 1, cv::LINE_AA);
    int ly = 62;
    auto layer = [&](const std::string & name, bool on) {
      box(lx, ly, lw, 26, name, on);
      ly += 30;
    };
    layer("ROI Editor", show_editor_);
    layer("Zoom", zoom_view_enabled_);
    layer("BEV", bev_view_enabled_);
    layer("Process", show_process_);
    layer("Info", show_info_);
    layer("Legend", show_legend_);
    return d;
  }

  // 사용자가 레이어 창을 X 로 닫으면 대응 토글을 off 로 내려 다음 렌더에 되살아나지 않게 한다.
  void syncClosedWindows()
  {
    static const std::pair<const char *, bool Yolopv2Cam2Node::*> kMap[] = {
      {kRoiQuadWindowName,   &Yolopv2Cam2Node::show_editor_},
      {kRoiZoomWindowName,   &Yolopv2Cam2Node::zoom_view_enabled_},
      {kRoiBevWindowName,    &Yolopv2Cam2Node::bev_view_enabled_},
      {kLaneDebugWindowName, &Yolopv2Cam2Node::show_process_},
      {kLaneInfoWindowName,  &Yolopv2Cam2Node::show_info_},
      {kStructureWindowName, &Yolopv2Cam2Node::show_structure_},
    };
    for (const auto & m : kMap) {
      if (!open_windows_.count(m.first)) {
        continue;
      }
      const double vis = cv::getWindowProperty(m.first, cv::WND_PROP_VISIBLE);
      if (vis >= 1.0) {
        // 이 백엔드는 VISIBLE 을 신뢰 가능하게 보고 → 이후 close 판정 신뢰.
        visible_prop_supported_ = true;
        confirmed_visible_.insert(m.first);
        continue;
      }
      // vis < 1: (a) 백엔드 미지원(항상 -1) 또는 (b) 갓 생성돼 아직 realize 전 →
      //          둘 다 '닫힘'이 아니므로 건드리지 않는다(즉시 off 오판 방지).
      if (!visible_prop_supported_ || !confirmed_visible_.count(m.first)) {
        continue;
      }
      // 예전에 realize 확인된 창이 VISIBLE 을 잃음 → 진짜 사용자 X 종료.
      this->*(m.second) = false;
      open_windows_.erase(m.first);
      confirmed_visible_.erase(m.first);
      panel_dirty_ = true;
      needs_redraw_ = true;   // 패널 버튼 색 갱신 위해 재렌더
    }
  }

  // 관리 패널 + 켜진 레이어 팝업 일괄 렌더. 무거운 뷰(info/BEV/panel)는 캐시 재사용.
  void renderAllViews(
    bool bag_mode, int idx, int total, bool playing, double speed,
    const std::string & numbuf, int last_key)
  {
    if (!debug_mode_) {
      return;
    }
    syncClosedWindows();          // 사용자가 닫은 창 반영(닫힌 레이어 off)
    if (!needs_redraw_) {
      return;                     // 정지/유휴 시 스킵
    }

    if (panel_dirty_ || panel_cache_.empty()) {
      panel_cache_ = drawControlPanel();   // 버튼 토글 시에만 재생성
      panel_dirty_ = false;
    }
    cv::imshow(kPanelWindowName, panel_cache_);

    // 편집/Zoom 은 ROI 꼭짓점에 의존 → 매 렌더 재생성(드래그 즉시 반영).
    if (show_editor_) {
      showLayer(kRoiQuadWindowName, withLegend(drawQuadEditor(), kRoiQuadWindowName));
    } else {
      hideLayer(kRoiQuadWindowName);
    }
    if (zoom_view_enabled_) {
      showLayer(kRoiZoomWindowName, withLegend(drawZoomView(), kRoiZoomWindowName));
    } else {
      hideLayer(kRoiZoomWindowName);
    }
    // BEV(warp) 는 원본 프레임에만 의존 → 새 프레임일 때만 재생성(드래그 중 스킵).
    if (bev_view_enabled_) {
      if (frame_updated_ || bev_cache_.empty()) {
        bev_cache_ = drawBevView();
      }
      showLayer(kRoiBevWindowName, withLegend(bev_cache_, kRoiBevWindowName));
    } else {
      hideLayer(kRoiBevWindowName);
    }
    // Process 모자이크는 파이프라인이 이미 만들어둔 저장본 → 재계산 없이 표시만.
    if (show_process_) {
      showLayer(kLaneDebugWindowName, withLegend(last_debug_image_, kLaneDebugWindowName));
    } else {
      hideLayer(kLaneDebugWindowName);
    }
    // Info 패널(putText ~90줄) 은 새 프레임/재생 오버레이 변경 시에만 재생성.
    if (show_info_) {
      if (frame_updated_ || overlay_dirty_ || info_cache_.empty()) {
        info_cache_ =
          renderInfoPanel(last_result_, idx, total, playing, speed, numbuf, last_key, bag_mode);
      }
      showLayer(kLaneInfoWindowName, withLegend(info_cache_, kLaneInfoWindowName));
    } else {
      hideLayer(kLaneInfoWindowName);
    }
    if (show_structure_) {
      showLayer(kStructureWindowName, drawStructureDiagram());
    } else {
      hideLayer(kStructureWindowName);
    }
    frame_updated_ = false;
    overlay_dirty_ = false;
    needs_redraw_ = false;
  }

  // ================= ROI 사다리꼴 크롭 (원본 영상 안쪽 유지) =================
  void ROIQuadSetting()
  {
    img_w_ = cam_conf_.image_size.width  > 0 ? cam_conf_.image_size.width  : 1280;
    img_h_ = cam_conf_.image_size.height > 0 ? cam_conf_.image_size.height : 720;
    ROIQuadDefault();

    roi_pre_enabled_  = this->declare_parameter<bool>("roi_quad_enable_pre", false);
    roi_post_enabled_ = this->declare_parameter<bool>("roi_quad_enable_post", false);
    roi_quad_preset_path_ = this->declare_parameter<std::string>("roi_quad_preset_path", kDefaultRoiQuadPreset);
    display_scale_    = this->declare_parameter<double>("roi_quad_display_scale", 0.6);
    const auto applied_param = this->declare_parameter<std::vector<int64_t>>("roi_quad_pts", Point2fToVector());
    VectorToQuad(applied_param);

    quad_param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&Yolopv2Cam2Node::onSetQuadParameters, this, _1));
  }

  void ROIQuadDefault()
  {
    const float W = static_cast<float>(img_w_);
    const float H = static_cast<float>(img_h_);
    roi_corners_[0] = cv::Point2f(0.30f * W, 0.55f * H);  // TL
    roi_corners_[1] = cv::Point2f(0.70f * W, 0.55f * H);  // TR
    roi_corners_[2] = cv::Point2f(0.98f * W, 0.98f * H);  // BR
    roi_corners_[3] = cv::Point2f(0.02f * W, 0.98f * H);  // BL
  }

  void rescaleQuadToImage(int w, int h)
  {
    if (w <= 0 || h <= 0) {
      return;
    }
    if (img_w_ > 0 && img_h_ > 0) {
      const float rx = static_cast<float>(w) / img_w_;
      const float ry = static_cast<float>(h) / img_h_;
      for (auto & p : roi_corners_) { p.x *= rx; p.y *= ry; }
    }
    img_w_ = w;
    img_h_ = h;
  }

  // 사다리꼴 안쪽만 유지 (바깥 0). img 크기에 맞춰 꼭짓점 좌표를 스케일.
  void cropToQuad(cv::Mat & img) const
  {
    if (img.empty() || img_w_ <= 0 || img_h_ <= 0) {
      return;
    }
    const double sx = static_cast<double>(img.cols) / img_w_;
    const double sy = static_cast<double>(img.rows) / img_h_;
    std::vector<cv::Point> poly(4);
    for (int i = 0; i < 4; ++i) {
      poly[i] = cv::Point(
        static_cast<int>(std::lround(roi_corners_[i].x * sx)),
        static_cast<int>(std::lround(roi_corners_[i].y * sy)));
    }
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8U);
    cv::fillConvexPoly(mask, poly, cv::Scalar(255));
    cv::Mat out = cv::Mat::zeros(img.size(), img.type());
    img.copyTo(out, mask);
    img = out;
  }

  std::vector<int64_t> Point2fToVector() const
  {
    std::vector<int64_t> v;
    v.reserve(8);
    for (const auto & p : roi_corners_) {
      v.push_back(static_cast<int64_t>(std::lround(p.x)));
      v.push_back(static_cast<int64_t>(std::lround(p.y)));
    }
    return v;
  }

  void VectorToQuad(const std::vector<int64_t> & v)
  {
    if (v.size() < 8) {
      return;
    }
    for (int i = 0; i < 4; ++i) {
      roi_corners_[i].x = std::clamp(static_cast<float>(v[2 * i]),     0.0f, static_cast<float>(img_w_));
      roi_corners_[i].y = std::clamp(static_cast<float>(v[2 * i + 1]), 0.0f, static_cast<float>(img_h_));
    }
  }

  cv::Mat drawQuadEditor()
  {
    cv::Mat bg;
    if (!last_frame_.empty()) {
      bg = last_frame_.clone();
    } else {
      bg = cv::Mat(img_h_ > 0 ? img_h_ : 720, img_w_ > 0 ? img_w_ : 1280,
                   CV_8UC3, cv::Scalar(50, 50, 50));
    }

    std::vector<cv::Point> poly(4);
    for (int i = 0; i < 4; ++i) {
      poly[i] = cv::Point(static_cast<int>(std::lround(roi_corners_[i].x)),
                          static_cast<int>(std::lround(roi_corners_[i].y)));
    }

    // 사다리꼴 바깥을 어둡게 → 유지 영역(안쪽)이 한눈에
    cv::Mat mask = cv::Mat::zeros(bg.size(), CV_8U);
    cv::fillConvexPoly(mask, poly, cv::Scalar(255));
    cv::Mat dark = bg.clone();
    dark *= 0.35;
    cv::Mat inv;
    cv::bitwise_not(mask, inv);
    dark.copyTo(bg, inv);

    const cv::Scalar edge = (roi_pre_enabled_ || roi_post_enabled_)
                              ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
    cv::polylines(bg, poly, true, edge, 2, cv::LINE_AA);

    const char * names[4] = {"TL", "TR", "BR", "BL"};
    for (int i = 0; i < 4; ++i) {
      cv::circle(bg, poly[i], 7, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
      cv::circle(bg, poly[i], 7, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      cv::putText(bg, names[i], poly[i] + cv::Point(9, 4),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    char b[160];
    std::snprintf(b, sizeof(b), "PRE=%s  POST=%s  scale=%.2f",
                  roi_pre_enabled_ ? "ON" : "off", roi_post_enabled_ ? "ON" : "off", display_scale_);
    cv::putText(bg, b, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::putText(bg, "drag corners  |  [p]PRE [o]POST [r]reset [s]save [l]load",
                cv::Point(12, bg.rows - 14), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    cv::Mat show;
    const double s = display_scale_ > 1e-3 ? display_scale_ : 1.0;
    cv::resize(bg, show, cv::Size(), s, s, cv::INTER_AREA);
    return show;
  }

  static void onQuadMouse(int event, int x, int y, int flags, void * userdata)
  {
    auto * self = static_cast<Yolopv2Cam2Node *>(userdata);
    const double s = self->display_scale_ > 1e-3 ? self->display_scale_ : 1.0;
    const float ox = static_cast<float>(x / s);  // 표시 좌표 → 원본 좌표
    const float oy = static_cast<float>(y / s);

    if (event == cv::EVENT_LBUTTONDOWN) {
      int best = -1;
      double bd = 1e18;
      for (int i = 0; i < 4; ++i) {
        const double d = std::hypot(self->roi_corners_[i].x - ox, self->roi_corners_[i].y - oy);
        if (d < bd) { bd = d; best = i; }
      }
      self->drag_idx_ = (bd < 40.0) ? best : -1;  // 40px 안쪽 꼭짓점만 잡기
    } else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON) &&
               self->drag_idx_ >= 0) {
      self->roi_corners_[self->drag_idx_].x = std::clamp(ox, 0.0f, static_cast<float>(self->img_w_));
      self->roi_corners_[self->drag_idx_].y = std::clamp(oy, 0.0f, static_cast<float>(self->img_h_));
      self->needs_redraw_ = true;   // 드래그 중엔 화면만 갱신 (파라미터 push 는 놓을 때 1회)
    } else if (event == cv::EVENT_LBUTTONUP) {
      if (self->drag_idx_ >= 0) {
        self->syncQuadParameters();          // 드래그 종료 시에만 ROS 파라미터 반영 (콜백 폭주 방지)
        self->reprocess_current_ = true;     // bag 모드: 바뀐 ROI 로 현재 프레임 재처리
      }
      self->drag_idx_ = -1;
    }
  }

  // 사다리꼴 영역을 직사각형으로 펴서(원근보정 warp) 확대 — 시각화 전용.
  cv::Mat drawZoomView()
  {
    if (last_frame_.empty()) {
      return cv::Mat();
    }
    cv::Point2f src[4];
    for (int i = 0; i < 4; ++i) {
      src[i] = roi_corners_[i];  // TL, TR, BR, BL (원본 좌표)
    }
    const int W = img_w_ > 0 ? img_w_ : 1280;
    const int H = img_h_ > 0 ? img_h_ : 720;
    const cv::Point2f dst[4] = {
      {0.0f, 0.0f}, {static_cast<float>(W), 0.0f},
      {static_cast<float>(W), static_cast<float>(H)}, {0.0f, static_cast<float>(H)}};
    const cv::Mat M = cv::getPerspectiveTransform(src, dst);
    cv::Mat out;
    cv::warpPerspective(last_frame_, out, M, cv::Size(W, H));
    cv::putText(out, "ROI ZOOM (visualize only)", cv::Point(12, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    cv::Mat show;
    const double s = display_scale_ > 1e-3 ? display_scale_ : 1.0;
    cv::resize(out, show, cv::Size(), s, s, cv::INTER_AREA);
    return show;
  }

  // BEV(IPM) 조감도 — 캘리브 기반 H_bev 로 원본을 탑뷰로 변환. Zoom 과 달리 물리적 정답.
  cv::Mat drawBevView()
  {
    if (last_frame_.empty() || !lld_) {
      return cv::Mat();
    }
    const cv::Mat H = lld_->GetBevHomography();
    if (H.empty()) {
      return cv::Mat();
    }
    cv::Mat bev;
    cv::warpPerspective(last_frame_, bev, H, lld_->GetBevSize());
    cv::putText(bev, "BEV (calibrated top-view)", cv::Point(12, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    return bev;
  }

  void handleQuadKey(int key)
  {
    if (key < 0) {
      return;
    }
    needs_redraw_ = true;   // 인식되는 키는 대부분 화면을 바꾸므로 재렌더 예약
    panel_dirty_ = true;    // p/o/z/b/r 등은 패널 버튼 상태를 바꿈
    switch (std::tolower(key & 0xFF)) {
      case 'p':
        roi_pre_enabled_ = !roi_pre_enabled_;
        syncQuadParameters();
        reprocess_current_ = true;
        RCLCPP_INFO(this->get_logger(), "ROI PRE crop = %s", roi_pre_enabled_ ? "ON" : "off");
        break;
      case 'o':
        roi_post_enabled_ = !roi_post_enabled_;
        syncQuadParameters();
        reprocess_current_ = true;
        RCLCPP_INFO(this->get_logger(), "ROI POST crop = %s", roi_post_enabled_ ? "ON" : "off");
        break;
      case 'r':
        ROIQuadDefault();
        syncQuadParameters();
        reprocess_current_ = true;
        RCLCPP_INFO(this->get_logger(), "ROI quad reset to default");
        break;
      case 's':
        saveQuadPreset();
        break;
      case 'l':
        loadQuadPreset();
        reprocess_current_ = true;
        break;
      case 'z':
        zoom_view_enabled_ = !zoom_view_enabled_;
        RCLCPP_INFO(this->get_logger(), "ROI Zoom view = %s", zoom_view_enabled_ ? "ON" : "off");
        break;
      case 'b':
        bev_view_enabled_ = !bev_view_enabled_;
        RCLCPP_INFO(this->get_logger(), "BEV view = %s", bev_view_enabled_ ? "ON" : "off");
        break;
      default:
        break;
    }
  }

  void saveQuadPreset()
  {
    try {
      cv::FileStorage fs(roi_quad_preset_path_, cv::FileStorage::WRITE);
      if (!fs.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "ROI quad 저장 실패(열기 불가): %s", roi_quad_preset_path_.c_str());
        return;
      }
      fs << "roi_quad_enable_pre"  << (roi_pre_enabled_ ? 1 : 0);
      fs << "roi_quad_enable_post" << (roi_post_enabled_ ? 1 : 0);
      fs << "img_w" << img_w_;
      fs << "img_h" << img_h_;
      fs << "roi_quad_pts" << "[";
      for (const auto & p : roi_corners_) {
        fs << static_cast<int>(std::lround(p.x)) << static_cast<int>(std::lround(p.y));
      }
      fs << "]";
      fs.release();
      RCLCPP_INFO(this->get_logger(), "ROI quad 저장 완료 → %s", roi_quad_preset_path_.c_str());
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "ROI quad 저장 예외: %s", e.what());
    }
  }

  void loadQuadPreset()
  {
    try {
      cv::FileStorage fs(roi_quad_preset_path_, cv::FileStorage::READ);
      if (!fs.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "ROI quad 불러오기 실패(파일 없음?): %s", roi_quad_preset_path_.c_str());
        return;
      }
      if (!fs["roi_quad_enable_pre"].empty())  roi_pre_enabled_  = static_cast<int>(fs["roi_quad_enable_pre"]) != 0;
      if (!fs["roi_quad_enable_post"].empty()) roi_post_enabled_ = static_cast<int>(fs["roi_quad_enable_post"]) != 0;
      std::vector<int> v;
      fs["roi_quad_pts"] >> v;
      if (v.size() >= 8) {
        VectorToQuad(std::vector<int64_t>(v.begin(), v.end()));
      }
      fs.release();
      syncQuadParameters();
      RCLCPP_INFO(this->get_logger(), "ROI quad 불러오기 완료 ← %s", roi_quad_preset_path_.c_str());
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "ROI quad 불러오기 예외: %s", e.what());
    }
  }

  void syncQuadParameters()
  {
    quad_updating_ = true;
    this->set_parameters({
      rclcpp::Parameter("roi_quad_enable_pre", roi_pre_enabled_),
      rclcpp::Parameter("roi_quad_enable_post", roi_post_enabled_),
      rclcpp::Parameter("roi_quad_pts", Point2fToVector()),
    });
    quad_updating_ = false;
  }

  rcl_interfaces::msg::SetParametersResult onSetQuadParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    if (quad_updating_) {
      return result;  // 내부 동기화 콜백 무시
    }
    for (const auto & p : params) {
      if (p.get_name() == "roi_quad_enable_pre") {
        roi_pre_enabled_ = p.as_bool();
      } else if (p.get_name() == "roi_quad_enable_post") {
        roi_post_enabled_ = p.as_bool();
      } else if (p.get_name() == "roi_quad_pts") {
        VectorToQuad(p.as_integer_array());
      } else if (p.get_name() == "roi_quad_preset_path") {
        roi_quad_preset_path_ = p.as_string();
      } else if (p.get_name() == "roi_quad_display_scale") {
        display_scale_ = p.as_double();
      }
    }
    needs_redraw_ = true;      // 외부(rqt 등) 파라미터 변경도 재렌더 트리거
    panel_dirty_ = true;
    reprocess_current_ = true;
    return result;
  }

  // ================= 정보 패널 (config + LaneResult 값 텍스트) =================
  cv::Mat renderInfoPanel(
    const LaneResult & r, int idx, int total, bool playing, double speed,
    const std::string & numbuf, int last_key, bool bag_mode) const
  {
    const cv::Scalar kWhite(210, 210, 210), kCyan(255, 255, 0), kYellow(0, 255, 255),
      kGreen(120, 255, 120);
    std::vector<std::pair<std::string, cv::Scalar>> lines;
    auto add = [&](const std::string & s, cv::Scalar col = cv::Scalar(210, 210, 210)) {
      lines.emplace_back(s, col);
    };
    auto f = [](double v) {
      std::ostringstream o; o << std::fixed << std::setprecision(3) << v; return o.str();
    };
    auto v4 = [](const cv::Vec4d & c) {
      std::ostringstream o; o << std::fixed << std::setprecision(4) << "[" << c[0] << ", "
        << c[1] << ", " << c[2] << ", " << c[3] << "]"; return o.str();
    };
    auto st = [](LaneState s) -> std::string {
      switch (s) {
        case LaneState::NODET: return "NODET";
        case LaneState::BAD: return "BAD";
        case LaneState::WEAK: return "WEAK";
        case LaneState::GOOD: return "GOOD";
      }
      return "?";
    };
    auto qs = [](LaneQuality s) -> std::string {
      switch (s) {
        case LaneQuality::GOOD: return "GOOD";
        case LaneQuality::WEAK: return "WEAK";
        case LaneQuality::BAD: return "BAD";
      }
      return "?";
    };
    auto bb = [](bool v) -> std::string { return v ? "true" : "false"; };
    auto sz = [](const cv::Size & s) {
      return "[" + std::to_string(s.width) + "x" + std::to_string(s.height) + "]";
    };

    add("=== PLAYBACK ===", kCyan);
    if (bag_mode) {
      add("frame: " + std::to_string(idx) + " / " + std::to_string(total - 1), kYellow);
      add(std::string(playing ? "> PLAYING" : "|| PAUSED"), kYellow);
      add("GOTO FRAME: " + numbuf + "_", kYellow);
      add("keys: [Space]play  [<-/->]step frame", kWhite);
      add("      [ , / . ]speed-/+  [0-9]+Enter jump  [q]quit", kWhite);
    } else {
      add("mode: LIVE (subscription)", kYellow);
      add("keys: [p]PRE [o]POST [z]zoom [b]BEV [r]reset [s/l]save/load", kWhite);
      add("      (panel buttons toggle layers)", kWhite);
    }
    add("last key code: " + std::to_string(last_key) + "  (verify)", cv::Scalar(160, 160, 160));
    add("");

    add("=== LaneResult (frame) ===", kCyan);
    add("coeffs_L      " + v4(r.coeffs_L), kGreen);
    add("coeffs_R      " + v4(r.coeffs_R), kGreen);
    add("coeffs_fit_L  " + v4(r.coeffs_fit_L));
    add("coeffs_fit_R  " + v4(r.coeffs_fit_R));
    add("state_L/R:      " + st(r.state_L) + " / " + st(r.state_R), kYellow);
    add("quality_L/R:    " + qs(r.quality_L) + " / " + qs(r.quality_R));
    add("detected_fit:   " + bb(r.detected_fit_L) + " / " + bb(r.detected_fit_R));
    add("is_detected:    " + bb(r.is_detected_L) + " / " + bb(r.is_detected_R));
    add("hard_fail:      " + bb(r.hard_fail_L) + " / " + bb(r.hard_fail_R));
    add("rmse_fit_m:     " + f(r.rmse_fit_m_L) + " / " + f(r.rmse_fit_m_R));
    add("pixel_count:    " + std::to_string(r.pixel_count_L) + " / " + std::to_string(r.pixel_count_R));
    add("lane_start_m:   " + f(r.lane_start_m_L) + " / " + f(r.lane_start_m_R));
    add("lane_end_m:     " + f(r.lane_end_m_L) + " / " + f(r.lane_end_m_R));
    add("span_m:         " + f(r.span_m_L) + " / " + f(r.span_m_R));
    add("view_range_m:   " + f(r.view_range_m_L) + " / " + f(r.view_range_m_R));
    add("availability:   " + std::to_string(r.availability_L) + " / " + std::to_string(r.availability_R));
    add("delta_y_m:      " + f(r.delta_y_m_L) + " / " + f(r.delta_y_m_R));
    add("delta_kappa:    " + f(r.delta_kappa_L) + " / " + f(r.delta_kappa_R));
    add("");

    const LaneConfig & c = lld_->GetConfig();
    add("=== LaneConfig ===", kCyan);
    add("waitkey: " + std::to_string(c.waitkey));
    add("mask_sobel_thresh: " + std::to_string(c.mask_sobel_thresh));
    add("mask_close_kh: " + std::to_string(c.mask_close_kh));
    add("roi_x[min,max]: " + f(c.roi_xmin) + ", " + f(c.roi_xmax));
    add("roi_y[min,max]: " + f(c.roi_ymin) + ", " + f(c.roi_ymax));
    add("bev_size: " + sz(c.bev_size));
    add("ksize_vert/noise: " + sz(c.ksize_vert) + " " + sz(c.ksize_noise));
    add("sobel_thres: " + std::to_string(c.sobel_thres));
    add("adaptive_block_size: " + std::to_string(c.adaptive_block_size));
    add("adaptive_C: " + f(c.adaptive_C));
    add("gamma: " + f(c.gamma));
    add("hls_white h/l/s min-max:");
    add("  " + std::to_string(c.hls_white_hmin) + "-" + std::to_string(c.hls_white_hmax) + " / " +
      std::to_string(c.hls_white_lmin) + "-" + std::to_string(c.hls_white_lmax) + " / " +
      std::to_string(c.hls_white_smin) + "-" + std::to_string(c.hls_white_smax));
    add("hls_yellow h/l/s min-max:");
    add("  " + std::to_string(c.hls_yellow_hmin) + "-" + std::to_string(c.hls_yellow_hmax) + " / " +
      std::to_string(c.hls_yellow_lmin) + "-" + std::to_string(c.hls_yellow_lmax) + " / " +
      std::to_string(c.hls_yellow_smin) + "-" + std::to_string(c.hls_yellow_smax));
    add("margin_sa: " + std::to_string(c.margin_sa));
    add("safe_gap_multiplier: " + f(c.safe_gap_multiplier));
    add("sa_anchor_x start/end/step: " + f(c.sa_anchor_x_start_m) + "/" +
      f(c.sa_anchor_x_end_m) + "/" + f(c.sa_anchor_x_step_m));
    add("sa_anchor_win w/h: " + std::to_string(c.sa_anchor_win_w_px) + "/" +
      std::to_string(c.sa_anchor_win_h_px));
    add("n_windows: " + std::to_string(c.n_windows));
    add("margin_sw: " + std::to_string(c.margin_sw));
    add("minpix: " + std::to_string(c.minpix));
    add("win_h/w: " + std::to_string(c.win_h) + "/" + std::to_string(c.win_w));
    add("sw_forward_limit_m: " + f(c.sw_forward_limit_m));
    add("expected_w: " + std::to_string(c.expected_w));
    add("lane_w[min,max]: " + f(c.lane_w_min) + ", " + f(c.lane_w_max));
    add("valid_view_range: " + f(c.valid_view_range));
    add("enable_lane_marker: " + bb(c.enable_lane_marker));
    add("lane_marker_fwd[min,max]: " + f(c.lane_marker_forward_min_m) + ", " +
      f(c.lane_marker_forward_max_m));
    add("roi_mid_offset_px: " + std::to_string(c.roi_mid_offset_px));
    add("roi_half_top/bot_ratio: " + f(c.roi_half_top_ratio) + "/" + f(c.roi_half_bot_ratio));
    add("roi_mask_enabled: " + bb(c.roi_mask_enabled));
    add("thickness_roi: " + std::to_string(c.thickness_roi));

    // 다단 컬럼 배치
    const int line_h = 17;
    const int per_col = 46;
    const int col_w = 350;
    const int rows = std::min(per_col, static_cast<int>(lines.size()));
    const int cols = (static_cast<int>(lines.size()) + per_col - 1) / per_col;
    cv::Mat panel(rows * line_h + 16, std::max(1, cols) * col_w, CV_8UC3, cv::Scalar(28, 28, 28));

    if (bag_mode) {
      // 재생/정지 아이콘 + 배속 게이지 + GOTO 입력 박스 (bag 모드 전용)
      {
        const int ix = col_w - 42, iy = 6, s = 18;
        if (playing) {
          std::vector<cv::Point> tri{{ix, iy}, {ix, iy + s}, {ix + s, iy + s / 2}};
          cv::fillConvexPoly(panel, tri, cv::Scalar(90, 220, 90), cv::LINE_AA);
        } else {
          const int bw = 6;
          cv::rectangle(panel, cv::Rect(ix, iy, bw, s), cv::Scalar(60, 220, 240), cv::FILLED);
          cv::rectangle(panel, cv::Rect(ix + s - bw, iy, bw, s), cv::Scalar(60, 220, 240), cv::FILLED);
        }
      }
      {
        const int gx = col_w - 190, gy = 30, gw = 176, gh = 12;
        std::ostringstream so; so << std::fixed << std::setprecision(1) << speed;
        cv::putText(panel, "SPEED  x" + so.str(), cv::Point(gx, gy + 10),
          cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(80, 230, 255), 2, cv::LINE_AA);
        const int by = gy + 18;
        cv::rectangle(panel, cv::Rect(gx, by, gw, gh), cv::Scalar(70, 70, 70), cv::FILLED);
        const double frac = std::clamp((speed - 0.1) / (4.0 - 0.1), 0.0, 1.0);
        cv::rectangle(panel, cv::Rect(gx, by, static_cast<int>(gw * frac), gh),
          cv::Scalar(60, 200, 90), cv::FILLED);
        cv::rectangle(panel, cv::Rect(gx, by, gw, gh), cv::Scalar(160, 160, 160), 1);
        const int mx = gx + static_cast<int>(gw * ((1.0 - 0.1) / (4.0 - 0.1)));
        cv::line(panel, cv::Point(mx, by - 2), cv::Point(mx, by + gh + 2), cv::Scalar(0, 165, 255), 1);
      }
      {
        const int goto_row = 3;
        const cv::Rect goto_box(6, 4 + goto_row * line_h, col_w - 16, line_h + 3);
        cv::rectangle(panel, goto_box, cv::Scalar(50, 50, 65), cv::FILLED);
        cv::rectangle(panel, goto_box, cv::Scalar(0, 160, 220), 1);
      }
    }

    for (size_t i = 0; i < lines.size(); ++i) {
      const int col = static_cast<int>(i) / per_col;
      const int row = static_cast<int>(i) % per_col;
      cv::putText(
        panel, lines[i].first, cv::Point(10 + col * col_w, 16 + row * line_h),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, lines[i].second, 1, cv::LINE_AA);
    }
    return panel;
  }

  // ================= 멤버 =================
  // ROI 사다리꼴
  std::array<cv::Point2f, 4> roi_corners_{};
  bool roi_pre_enabled_{false};
  bool roi_post_enabled_{false};
  int img_w_{0};
  int img_h_{0};
  double display_scale_{0.6};
  int drag_idx_{-1};
  bool quad_updating_{false};
  bool needs_redraw_{true};       // 화면 재렌더 필요 여부(초기 true → 첫 틱에 창 표시)
  bool reprocess_current_{false}; // bag 모드: 현재 프레임 재처리 필요
  std::string roi_quad_preset_path_;
  cv::Mat last_frame_;
  rclcpp::TimerBase::SharedPtr quad_ui_timer_;
  OnSetParametersCallbackHandle::SharedPtr quad_param_callback_handle_;

  // 관리 패널 / 레이어 토글
  bool show_editor_{true};        // L1 ROI 편집
  bool zoom_view_enabled_{false}; // L2 Zoom
  bool bev_view_enabled_{false};  // L3 BEV
  bool show_process_{true};       // L4 파이프라인 모자이크
  bool show_info_{true};          // 정보 패널
  bool show_legend_{false};       // 기능 A: 시각화 info 범례(해상도+함수) 오버레이 전역 토글
  bool show_structure_{false};    // 기능 B: 전체 구조 라이브 다이어그램 창 토글
  // --- 렌더 캐시(바뀐 것만 재생성): 드래그 시 mousemove 폭주로 무거운 뷰 재생성 방지 ---
  bool frame_updated_{false};     // 새 프레임 처리됨 → info/BEV 캐시 갱신
  bool overlay_dirty_{false};     // 재생 오버레이(프레임/배속/GOTO) 변경 → info 캐시 갱신
  bool panel_dirty_{true};        // 패널 버튼 상태 변경 → 패널 비트맵 재생성
  cv::Mat panel_cache_;
  cv::Mat info_cache_;
  cv::Mat bev_cache_;
  std::vector<PanelButton> panel_buttons_;
  std::vector<std::pair<std::string, int>> panel_headers_;
  int panel_height_{600};
  std::set<std::string> open_windows_;
  // 창 X 종료 감지(syncClosedWindows)용 — 백엔드가 WND_PROP_VISIBLE 을 신뢰 가능하게
  // 보고할 때만 close 판정. GTK 등 항상 -1 반환 백엔드에선 오판(즉시 off) 방지.
  bool visible_prop_supported_{false};       // VISIBLE≥1 을 한 번이라도 본 적 있는가
  std::set<std::string> confirmed_visible_;  // realize 확인된(한 번 이상 VISIBLE≥1) 창
  int last_ui_key_{-1};
  LaneResult last_result_{};
  cv::Mat last_debug_image_;

  // bag 뷰어
  double debug_window_scale_{0.6};
  std::string bag_path_;
  std::string bag_topic_{"/camera/image_rect"};
  int bag_trackbar_pos_{0};
  bool bag_seek_requested_{false};

  bool engine_initialized_{false};
  bool debug_mode_{false};
  uint8_t rolling_counter_{0};
  bool enable_lane_marker_{true};
  std::string lane_marker_frame_id_{"camera_link"};
  double lane_marker_forward_min_m_{0.0};
  double lane_marker_forward_max_m_{30.0};
  double lane_marker_dx_m_{0.5};
  double lane_marker_line_width_m_{0.12};
  double lane_marker_origin_radius_m_{0.25};
  double lane_marker_z_m_{0.02};

  DetectionConfig detection_config_{};
  TrackingConfig tracking_config_{};
  ObjectBuilderConfig object_builder_config_{};

  CameraConfig cam_conf_{};
  CalibData calib_{};
  std::unique_ptr<BboxGroundProjector> bbox_projector_;
  DetectionEngine engine_;
  std::unique_ptr<LaneLineDetector> lld_;
  TrackingHelper tracking_helper_;
  std::unique_ptr<ObjectMessageBuilder> object_message_builder_;
  Cam2DataComposer cam2_data_composer_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Publisher<fo_msgs::msg::Cam2LD>::SharedPtr pub_ld_;
  rclcpp::Publisher<fo_msgs::msg::Cam2Data>::SharedPtr pub_cam2_data_;
  rclcpp::Publisher<fo_msgs::msg::Cam2DataForSF2>::SharedPtr pub_sf_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_bbox_image_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_lane_marker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Yolopv2Cam2Node>();
  if (node->isBagMode()) {
    node->runBagViewer();  // bag 직접 읽어 프레임 단위 재생/탐색
  } else {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
