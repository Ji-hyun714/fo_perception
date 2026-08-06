#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "detection_engine.h"
#include "lane_detection/LaneDetection.hpp"
#include "fo_msgs/msg/cam2_ld.hpp"
#include "fo_msgs/msg/object2_d.hpp"
#include "fo_msgs/msg/track_results.hpp"

#include <utils/BboxGroundProjector.hpp>
#include <utils/CalibData.hpp>
#include <utils/common.hpp>

using std::placeholders::_1;

/* 260225 Cam2SF에서 일단 임시로 요구한 클래스 사항

10 bus
11 bicycle
12 truck
13 car
14 person

*/ 

class Yolopv2Node : public rclcpp::Node {
public:
    Yolopv2Node()
    : Node("yolopv2_node")
    {
        this->declare_parameter<std::string>("work_dir", "/home/wise/adsp_perception_camera/src/lane_detection/yolopv2_trt/resource");
        this->declare_parameter<std::string>("input_topic", "/camera/image_rect");
        this->declare_parameter<std::string>("output_topic_objs", "/camera/detections");
        this->declare_parameter<std::string>("bbox_image_topic", "/camera/det_bboxes");
        this->declare_parameter<std::string>("yaml_path", "");
        this->declare_parameter<std::string>("ld_cfg", "");
        this->declare_parameter<int>("queue_size", 1);
        this->declare_parameter<bool>("show_debug", true);
        this->declare_parameter<double>("od_objectness_threshold", 0.30);
        this->declare_parameter<double>("od_score_threshold", 0.30);
        this->declare_parameter<std::vector<int64_t>>("od_class_whitelist", std::vector<int64_t>{0, 1, 2, 3, 5, 7});
        this->declare_parameter<double>("bbox_ground_max_range_m", 120.0);
        this->declare_parameter<double>("bbox_ground_min_height_px", 4.0);

        const std::string work_dir = this->get_parameter("work_dir").as_string();
        const std::string input_topic = this->get_parameter("input_topic").as_string();
        const std::string output_topic_objs = this->get_parameter("output_topic_objs").as_string();
        const std::string bbox_image_topic = this->get_parameter("bbox_image_topic").as_string();
        const int qos_depth = this->get_parameter("queue_size").as_int();
        const float od_objectness_threshold = static_cast<float>(this->get_parameter("od_objectness_threshold").as_double());
        const float od_score_threshold = static_cast<float>(this->get_parameter("od_score_threshold").as_double());
        const double bbox_ground_max_range_m = this->get_parameter("bbox_ground_max_range_m").as_double();
        const double bbox_ground_min_height_px = this->get_parameter("bbox_ground_min_height_px").as_double();
        const auto od_class_whitelist_i64 = this->get_parameter("od_class_whitelist").as_integer_array();
        std::vector<int32_t> od_class_whitelist;
        od_class_whitelist.reserve(od_class_whitelist_i64.size());
        for (const auto id : od_class_whitelist_i64) {
            od_class_whitelist.push_back(static_cast<int32_t>(id));
        }

        show_debug_ = this->get_parameter("show_debug").as_bool();

        if (work_dir.empty()) {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'work_dir' is empty. Set it to '<...>/resource'.");
            return;
        }

        constexpr int kNumThreads = 4;
        if (engine_.Initialize(work_dir, kNumThreads) != DetectionEngine::kRetOk) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize DetectionEngine. work_dir=%s", work_dir.c_str());
            return;
        }
        engine_.SetDetectionThresholds(od_objectness_threshold, od_score_threshold);
        engine_.SetDetectionClassWhitelist(od_class_whitelist);
        engine_.SetPostProcessConfig(kEnableLd_, kEnableDa_, kEnableOd_, kLaneClassId_, kDaClassId_);
        engine_initialized_ = true;

        const DetectionEngine::ModelOutputInfo model_output_info = engine_.GetModelOutputInfo();
        std::ostringstream model_outputs_ss;
        for (size_t i = 0; i < model_output_info.output_names.size(); ++i) {
            if (i > 0) model_outputs_ss << ", ";
            model_outputs_ss << model_output_info.output_names[i];
        }
        RCLCPP_INFO(this->get_logger(),
            "model_outputs=[%s] seg=%s ll=%s pred0=%s pred1=%s pred2=%s",
            model_outputs_ss.str().c_str(),
            model_output_info.has_seg ? "true" : "false",
            model_output_info.has_ll ? "true" : "false",
            model_output_info.has_pred0 ? "true" : "false",
            model_output_info.has_pred1 ? "true" : "false",
            model_output_info.has_pred2 ? "true" : "false");
        if ((kEnableLd_ || kEnableDa_) && !(model_output_info.has_seg && model_output_info.has_ll)) {
            RCLCPP_WARN(this->get_logger(),
                "enable_ld/enable_da requested but model outputs don't include both seg and ll. related masks may be empty.");
        }
        if (kEnableOd_ && !(model_output_info.has_pred0 && model_output_info.has_pred1 && model_output_info.has_pred2)) {
            RCLCPP_WARN(this->get_logger(),
                "enable_od requested but model outputs don't include pred0/pred1/pred2. bbox list may be empty.");
        }

        // std::string yp = "/home/wise/adsp_perception_camera/src/utils/yaml/ioniq_econ.yaml";
        // std::string yp = "/home/wise/adsp_perception_camera/src/utils/yaml/econ_refined_260130.yaml";
        // std::string yp = "/home/wise/adsp_perception_camera/src/utils/yaml/econ_260311_ippe.yaml";
        std::string yp = "/home/wise/adsp_perception_camera/src/utils/yaml/econ_260311_refined.yaml";
        
        
        std::string yaml_path = this->get_parameter("yaml_path").as_string();
        if (yaml_path.empty()) yaml_path = yp;

        cam_conf_ = getCameraConfigFromYaml(yaml_path);
        printCameraConfig(cam_conf_);
        if (cam_conf_.model == 0) {
            calib_ = CalibData::MakePinhole(cam_conf_.image_size, cam_conf_.K, cam_conf_.D, cam_conf_.R, cam_conf_.t);
        } else {
            calib_ = CalibData::MakeFisheye(cam_conf_.image_size, cam_conf_.K, cam_conf_.D, cam_conf_.R, cam_conf_.t);
        }
        GroundProjectorOptions projector_options;
        projector_options.min_bbox_height_px = bbox_ground_min_height_px;
        projector_options.max_range_m = bbox_ground_max_range_m;
        bbox_projector_ = std::make_unique<BboxGroundProjector>(calib_, projector_options);

        std::string ldcfg = "/home/wise/adsp_perception_camera/src/lane_detection/yaml/kcity.yaml";
        std::string ld_cfg = this->get_parameter("ld_cfg").as_string();
        if (ld_cfg.empty()) ld_cfg = ldcfg;
        lld_ = std::make_unique<LaneLineDetector>(calib_, ld_cfg);
        enable_lane_marker_ = lld_->GetEnableLaneMarker();
        lane_marker_frame_id_ = lld_->GetLaneMarkerFrameId();
        lane_marker_forward_min_m_ = std::max(0.0, lld_->GetLaneMarkerForwardMinM());
        lane_marker_forward_max_m_ = std::max(lane_marker_forward_min_m_ + 0.1, lld_->GetLaneMarkerForwardMaxM());
        lane_marker_dx_m_ = std::max(0.05, lld_->GetLaneMarkerDxM());
        lane_marker_line_width_m_ = std::max(0.01, lld_->GetLaneMarkerLineWidthM());
        lane_marker_origin_radius_m_ = std::max(0.01, lld_->GetLaneMarkerOriginRadiusM());
        lane_marker_z_m_ = lld_->GetLaneMarkerZ();

        sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, qos_depth, std::bind(&Yolopv2Node::callbackInference, this, _1));

        pub_ld_ = this->create_publisher<fo_msgs::msg::Cam2LD>(kLdTopic_, qos_depth);
        pub_detections_ = this->create_publisher<fo_msgs::msg::TrackResults>(output_topic_objs, qos_depth);
        pub_bbox_image_ = this->create_publisher<sensor_msgs::msg::Image>(bbox_image_topic, qos_depth);
        if (enable_lane_marker_) {
            pub_lane_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kLaneMarkerTopic_, qos_depth);
        }

        RCLCPP_INFO(this->get_logger(),
            "Yolopv2Node started. input=%s output=%s work_dir=%s",
            input_topic.c_str(), kLdTopic_, work_dir.c_str());
        RCLCPP_INFO(this->get_logger(),
            "flags: enable_ld=%s enable_od=%s enable_da=%s lane_class_id=%d da_class_id=%d",
            kEnableLd_ ? "true" : "false", kEnableOd_ ? "true" : "false", kEnableDa_ ? "true" : "false",
            kLaneClassId_, kDaClassId_);
        std::ostringstream whitelist_ss;
        if (od_class_whitelist.empty()) {
            whitelist_ss << "ALL";
        } else {
            for (size_t i = 0; i < od_class_whitelist.size(); ++i) {
                if (i > 0) whitelist_ss << ",";
                whitelist_ss << od_class_whitelist[i];
            }
        }
        // RCLCPP_INFO(this->get_logger(),
        //     "od thresholds: objectness=%.2f score=%.2f whitelist=[%s]",
        //     od_objectness_threshold, od_score_threshold, whitelist_ss.str().c_str());
        RCLCPP_INFO(this->get_logger(), "detections topic=%s", output_topic_objs.c_str());
        RCLCPP_INFO(this->get_logger(), "bbox image topic=%s", bbox_image_topic.c_str());
        if (enable_lane_marker_) {
            RCLCPP_INFO(this->get_logger(),
                "marker enabled topic=%s frame=%s x_range=[%.1f, %.1f] dx=%.2f line_w=%.2f origin_r=%.2f z=%.2f",
                kLaneMarkerTopic_, lane_marker_frame_id_.c_str(),
                lane_marker_forward_min_m_, lane_marker_forward_max_m_,
                lane_marker_dx_m_, lane_marker_line_width_m_, lane_marker_origin_radius_m_, lane_marker_z_m_);
        } else {
            RCLCPP_INFO(this->get_logger(), "marker disabled");
        }
    }

    ~Yolopv2Node() override
    {
        if (show_debug_) {
            cv::destroyWindow("Lane Detection Process");
        }
        if (engine_initialized_) {
            engine_.Finalize();
        }
    }

private:
    void callbackInference(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        if (!engine_initialized_) return;

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty frame");
            return;
        }

        DetectionEngine::Result det_result;
        if (engine_.Process(frame, det_result) != DetectionEngine::kRetOk) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Inference failed for one frame");
            return;
        }
        const std::vector<BoundingBox>& bbox_list_for_use = det_result.bbox_list;
        // DetectionEngine의 BoundingBox는 center-based xywh가 아니라
        // left-top x/y + width/height 형식

        const cv::Mat& lane_mask = det_result.mat_lane_mask;
        if (lane_mask.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "lane_mask is empty");
            publishLaneMarkers(LaneResult{}, msg->header.stamp);
        } else {
            LaneResult res = lld_->DetectFromMask(lane_mask, frame);
            const cv::Mat dbg = lld_->getDebugImage();
            if (!dbg.empty()) {
                if (show_debug_) {
                    cv::imshow("Lane Detection Process", dbg);
                    cv::waitKey(1);
                }
            }
            if (pub_ld_) {
                fo_msgs::msg::Cam2LD msg_ld;
                msg_ld.header_sen.stamp = msg->header.stamp;
                msg_ld.header_sen.frame_id = msg->header.frame_id;
                msg_ld.header_ld.stamp = this->now();
                msg_ld.header_ld.frame_id = "lane_detection";

                fillMsgLD(res, msg_ld);
                pub_ld_->publish(msg_ld);

                rolling_counter_ = (rolling_counter_ + 1) % 255;
            }
            publishLaneMarkers(res, msg->header.stamp);
        }

        if (pub_detections_) {
            const auto det_msg = makeDetectionMsg(bbox_list_for_use, msg->header);
            pub_detections_->publish(det_msg);
        }

        if (pub_bbox_image_) {
            publishBboxImage(frame, bbox_list_for_use, msg->header);
        }
    }

    void publishBboxImage(
        const cv::Mat& frame,
        const std::vector<BoundingBox>& bbox_list,
        const std_msgs::msg::Header& header) const
    {
        if (!pub_bbox_image_ || frame.empty()) return;

        cv::Mat vis = frame.clone();
        for (const auto& bbox : bbox_list) {
            drawBboxOverlay(vis, bbox);
        }

        auto msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, vis).toImageMsg();
        pub_bbox_image_->publish(*msg);
    }

    void drawBboxOverlay(cv::Mat& image, const BoundingBox& bbox) const
    {
        if (image.empty()) return;

        int x = static_cast<int>(std::round(bbox.x));
        int y = static_cast<int>(std::round(bbox.y));
        int w = static_cast<int>(std::round(bbox.w));
        int h = static_cast<int>(std::round(bbox.h));

        x = (std::max)(0, (std::min)(x, image.cols - 1));
        y = (std::max)(0, (std::min)(y, image.rows - 1));
        w = (std::max)(0, (std::min)(w, image.cols - x));
        h = (std::max)(0, (std::min)(h, image.rows - y));
        if (w <= 0 || h <= 0) return;

        const cv::Rect rect(x, y, w, h);
        const cv::Scalar box_color(0, 255, 255);
        const cv::Scalar point_color(0, 0, 255);
        const cv::Scalar text_color(0, 255, 255);

        cv::rectangle(image, rect, box_color, 2, cv::LINE_AA);

        const cv::Rect2d bbox_rect(
            static_cast<double>(bbox.x),
            static_cast<double>(bbox.y),
            static_cast<double>(bbox.w),
            static_cast<double>(bbox.h));
        const GroundPointEstimate estimate = bbox_projector_
            ? bbox_projector_->EstimateRect(bbox_rect)
            : GroundPointEstimate {};
        const cv::Point bottom_center(
            static_cast<int>(std::round(std::clamp(estimate.foot_px.x, 0.0, static_cast<double>(image.cols - 1)))),
            static_cast<int>(std::round(std::clamp(estimate.foot_px.y, 0.0, static_cast<double>(image.rows - 1)))));
        cv::circle(image, bottom_center, 4, point_color, cv::FILLED, cv::LINE_AA);

        std::ostringstream pos_ss;
        if (estimate.valid) {
            pos_ss << std::fixed << std::setprecision(2)
                   << "(" << estimate.ground_xy_m.x << ", " << estimate.ground_xy_m.y << ")";
        } else {
            pos_ss << "(nan, nan)";
        }
        const std::string pos_text = pos_ss.str();

        constexpr double kFontScale = 0.42;
        constexpr int kThickness = 1;
        int baseline = 0;
        const cv::Size text_size = cv::getTextSize(
            pos_text, cv::FONT_HERSHEY_SIMPLEX, kFontScale, kThickness, &baseline);
        const int text_x = (std::max)(0, (std::min)(rect.x, image.cols - text_size.width - 1));
        const int preferred_text_y = rect.y + rect.height + text_size.height + 4;
        const int text_y = (std::min)(image.rows - baseline - 1, preferred_text_y);
        if (text_y > rect.y + rect.height) {
            cv::putText(
                image,
                pos_text,
                cv::Point(text_x, text_y),
                cv::FONT_HERSHEY_SIMPLEX,
                kFontScale,
                text_color,
                kThickness,
                cv::LINE_AA);
        }
    }

    fo_msgs::msg::TrackResults makeDetectionMsg(
        const std::vector<BoundingBox>& bbox_list,
        const std_msgs::msg::Header& header) const
    {
        fo_msgs::msg::TrackResults msg;
        msg.header = header;
        msg.objs.reserve(bbox_list.size());

        // DetectionEngine의 BoundingBox는 center-based xywh가 아니라
        // left-top x/y + width/height 형식이다.
        for (const auto& bbox : bbox_list) {
            fo_msgs::msg::Object2D obj;
            obj.class_id = bbox.class_id;
            obj.score = bbox.score;
            obj.x = static_cast<float>(bbox.x);
            obj.y = static_cast<float>(bbox.y);
            obj.w = static_cast<float>(bbox.w);
            obj.h = static_cast<float>(bbox.h);
            // obj.validity = false;
            obj.validity = obj.score > threshold_valid ? true : false;

            obj.x_m = 0.0F;
            obj.y_m = 0.0F;
            obj.range_m = 0.0F;
            obj.angle_deg = 0.0F;
            obj.track_id = 0U;
            obj.track_age = 0U;
            obj.track_status = fo_msgs::msg::Object2D::TRACK_STATUS_NEW;
            msg.objs.push_back(obj);
        }

        return msg;
    }

    void fillMsgLD(const LaneResult res, fo_msgs::msg::Cam2LD &msg_pub)
    {
        const uint8_t quality_L = laneStateToQuality(res.state_L);
        const uint8_t quality_R = laneStateToQuality(res.state_R);
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
    }

    void publishLaneMarkers(const LaneResult& res, const builtin_interfaces::msg::Time& stamp)
    {
        if (!enable_lane_marker_ || !pub_lane_marker_) return;

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.reserve(6);

        auto buildDeleteMarker = [&](int id, const std::string& ns) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = lane_marker_frame_id_;
            m.header.stamp = stamp;
            m.ns = ns;
            m.id = id;
            m.action = visualization_msgs::msg::Marker::DELETE;
            return m;
        };

        auto buildLaneLineMarker = [&](const cv::Vec4d& coeffs, double lane_start_m, double lane_end_m,
                                       const std::string& ns, int id, float r, float g, float b) {
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
                const double y = coeffs[0] * x * x * x
                               + coeffs[1] * x * x
                               + coeffs[2] * x
                               + coeffs[3];
                geometry_msgs::msg::Point p;
                p.x = x;
                p.y = y;
                p.z = lane_marker_z_m_;
                m.points.push_back(std::move(p));
            }

            if (!m.points.empty() && m.points.back().x < end_m) {
                const double x = end_m;
                const double y = coeffs[0] * x * x * x
                               + coeffs[1] * x * x
                               + coeffs[2] * x
                               + coeffs[3];
                geometry_msgs::msg::Point p;
                p.x = x;
                p.y = y;
                p.z = lane_marker_z_m_;
                m.points.push_back(std::move(p));
            }
            return m;
        };

        auto buildAxisArrow = [&](int id, float r, float g, float b,
                                  double ex, double ey, double ez) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = lane_marker_frame_id_;
            m.header.stamp = stamp;
            m.ns = "lane_axes";
            m.id = id;
            m.type = visualization_msgs::msg::Marker::ARROW;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.08;  // shaft diameter
            m.scale.y = 0.16;  // head diameter
            m.scale.z = 0.20;  // head length
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

        const uint8_t quality_L = laneStateToQuality(res.state_L);
        const uint8_t quality_R = laneStateToQuality(res.state_R);

        if (quality_L >= 2) {
            visualization_msgs::msg::Marker left = buildLaneLineMarker(
                res.coeffs_L, std::max(0.0, res.lane_start_m_L), std::max(0.0, res.lane_end_m_L),
                "lane_marker", 0, 0.0F, 0.0F, 1.0F);
            if (left.points.size() >= 2) marker_array.markers.push_back(std::move(left));
            else marker_array.markers.push_back(buildDeleteMarker(0, "lane_marker"));
        } else {
            marker_array.markers.push_back(buildDeleteMarker(0, "lane_marker"));
        }

        if (quality_R >= 2) {
            visualization_msgs::msg::Marker right = buildLaneLineMarker(
                res.coeffs_R, std::max(0.0, res.lane_start_m_R), std::max(0.0, res.lane_end_m_R),
                "lane_marker", 1, 1.0F, 0.0F, 0.0F);
            if (right.points.size() >= 2) marker_array.markers.push_back(std::move(right));
            else marker_array.markers.push_back(buildDeleteMarker(1, "lane_marker"));
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

        // Coordinate axes at origin: X(red), Y(green), Z(blue)
        marker_array.markers.push_back(buildAxisArrow(101, 1.0F, 0.0F, 0.0F, 1.5, 0.0, 0.0));
        marker_array.markers.push_back(buildAxisArrow(102, 0.0F, 1.0F, 0.0F, 0.0, 1.5, 0.0));
        marker_array.markers.push_back(buildAxisArrow(103, 0.0F, 0.0F, 1.0F, 0.0, 0.0, 1.0));

        pub_lane_marker_->publish(marker_array);
    }

    static uint8_t laneStateToQuality(LaneState state)
    {
        switch (state) {
            case LaneState::GOOD:  return 3;
            case LaneState::WEAK:  return 2;
            case LaneState::BAD:   return 1;
            case LaneState::NODET:
            default:               return 0;
        }
    }

private:
    bool engine_initialized_ = false;
    bool show_debug_ = false;
    uint8_t rolling_counter_ = 0;
    bool enable_lane_marker_ = true;
    std::string lane_marker_frame_id_ = "camera_link";
    double lane_marker_forward_min_m_ = 0.0;
    double lane_marker_forward_max_m_ = 30.0;
    double lane_marker_dx_m_ = 0.5;
    double lane_marker_line_width_m_ = 0.12;
    double lane_marker_origin_radius_m_ = 0.25;
    double lane_marker_z_m_ = 0.02;

    float threshold_valid = 0.25;

    CameraConfig cam_conf_;
    CalibData calib_;
    std::unique_ptr<BboxGroundProjector> bbox_projector_;

    DetectionEngine engine_;
    std::unique_ptr<LaneLineDetector> lld_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
    rclcpp::Publisher<fo_msgs::msg::Cam2LD>::SharedPtr pub_ld_;
    rclcpp::Publisher<fo_msgs::msg::TrackResults>::SharedPtr pub_detections_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_bbox_image_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_lane_marker_;

    static constexpr const char* kLdTopic_ = "/camera/lane_result";
    static constexpr const char* kLaneMarkerTopic_ = "/camera/lane_marker";
    static constexpr bool kEnableLd_ = true;
    static constexpr bool kEnableOd_ = true;
    static constexpr bool kEnableDa_ = false;
    static constexpr int kLaneClassId_ = 2;
    static constexpr int kDaClassId_ = 1;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Yolopv2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
