#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "detection_engine.h"
#include "lane_detection/LaneDetection.hpp"
#include "fo_msgs/msg/cam2_ld.hpp"
#include "fo_msgs/msg/cam2_data_for_sf2.hpp"
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
        this->declare_parameter<std::string>("yaml_path", "");
        this->declare_parameter<std::string>("ld_cfg", "");
        this->declare_parameter<int>("queue_size", 1);
        this->declare_parameter<bool>("log_video", false);
        this->declare_parameter<bool>("log_perf", true);
        this->declare_parameter<bool>("show_debug", true);
        this->declare_parameter<double>("od_objectness_threshold", 0.30);
        this->declare_parameter<double>("od_score_threshold", 0.30);
        this->declare_parameter<std::vector<int64_t>>("od_class_whitelist", std::vector<int64_t>{0, 1, 2, 3, 5, 7});
        this->declare_parameter<double>("bbox_ground_max_range_m", 120.0);
        this->declare_parameter<double>("bbox_ground_min_height_px", 4.0);

        const std::string work_dir = this->get_parameter("work_dir").as_string();
        const std::string input_topic = this->get_parameter("input_topic").as_string();
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

        log_video_ = this->get_parameter("log_video").as_bool();
        log_perf_ = this->get_parameter("log_perf").as_bool();
        show_debug_ = this->get_parameter("show_debug").as_bool();
        if (log_video_) {
            video_file_name_ = makeVideoFileName();
        }

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

        std::string yp = "/home/wise/adsp_perception_camera/src/utils/yaml/ioniq_econ.yaml";
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

        const auto t_pub_create0 = std::chrono::steady_clock::now();
        pub_ld_ = this->create_publisher<fo_msgs::msg::Cam2LD>(kLdTopic_, qos_depth);
        const auto t_pub_create1 = std::chrono::steady_clock::now();
        pub_sf_ = this->create_publisher<fo_msgs::msg::Cam2DataForSF2>(kSfTopic_, qos_depth);
        const auto t_pub_create2 = std::chrono::steady_clock::now();
        if (enable_lane_marker_) {
            pub_lane_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kLaneMarkerTopic_, qos_depth);
        }
        const auto t_pub_create3 = std::chrono::steady_clock::now();
        const double ms_pub_create_ld = std::chrono::duration<double, std::milli>(t_pub_create1 - t_pub_create0).count();
        const double ms_pub_create_sf = std::chrono::duration<double, std::milli>(t_pub_create2 - t_pub_create1).count();
        const double ms_pub_create_marker = std::chrono::duration<double, std::milli>(t_pub_create3 - t_pub_create2).count();

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
        RCLCPP_INFO(this->get_logger(),
            "od thresholds: objectness=%.2f score=%.2f whitelist=[%s]",
            od_objectness_threshold, od_score_threshold, whitelist_ss.str().c_str());
        RCLCPP_INFO(this->get_logger(),
            "publisher_create_time[ms] ld_topic=%.3f sf_objs=%.3f",
            ms_pub_create_ld, ms_pub_create_sf);
        if (enable_lane_marker_) {
            RCLCPP_INFO(this->get_logger(),
                "marker enabled topic=%s frame=%s x_range=[%.1f, %.1f] dx=%.2f line_w=%.2f origin_r=%.2f z=%.2f pub_create=%.3fms",
                kLaneMarkerTopic_, lane_marker_frame_id_.c_str(),
                lane_marker_forward_min_m_, lane_marker_forward_max_m_,
                lane_marker_dx_m_, lane_marker_line_width_m_, lane_marker_origin_radius_m_, lane_marker_z_m_,
                ms_pub_create_marker);
        } else {
            RCLCPP_INFO(this->get_logger(), "marker disabled");
        }
        if (log_video_) {
            const std::string viz_mask_name = kEnableLd_ ? "lane_mask" : (kEnableDa_ ? "da_mask" : "empty");
            RCLCPP_INFO(this->get_logger(),
                "Video logging enabled. output=%s (left: original%s 640x360, right: %s 640x360)",
                video_file_name_.c_str(),
                kEnableOd_ ? "+bbox" : "",
                viz_mask_name.c_str());
            if (qos_depth <= 1) {
                RCLCPP_WARN(this->get_logger(),
                    "queue_size=%d. When processing is slower than input, frames are dropped and logged video may look choppy.",
                    qos_depth);
            }
        }
    }

    ~Yolopv2Node() override
    {
        flushTimingWindow(true);
        closeVideoWriter();
        if (engine_initialized_) {
            engine_.Finalize();
        }
    }

private:
    void callbackInference(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        if (!engine_initialized_) return;

        const auto t0 = std::chrono::steady_clock::now();

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
        const auto t1 = std::chrono::steady_clock::now();

        DetectionEngine::Result det_result;
        if (engine_.Process(frame, det_result) != DetectionEngine::kRetOk) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Inference failed for one frame");
            return;
        }
        const std::vector<BoundingBox>& bbox_list_for_use = det_result.bbox_list;
        last_bbox_count_ = bbox_list_for_use.size();
        if (!bbox_list_for_use.empty()) {
            const auto& top_bbox = bbox_list_for_use.front();
            last_bbox_class_id_ = top_bbox.class_id;
            last_bbox_label_ = top_bbox.label;
            last_bbox_score_ = top_bbox.score;
        } else {
            last_bbox_class_id_ = -1;
            last_bbox_label_.clear();
            last_bbox_score_ = 0.0F;
        }
        const auto t2 = std::chrono::steady_clock::now();

        const cv::Mat& lane_mask = det_result.mat_lane_mask;
        const cv::Mat& da_mask = det_result.mat_da_mask;
        const auto t3 = std::chrono::steady_clock::now();

        const auto t_video0 = std::chrono::steady_clock::now();
        const cv::Mat& mask_for_log = (!lane_mask.empty() && kEnableLd_) ? lane_mask : da_mask;
        writeLogFrame(frame, mask_for_log, bbox_list_for_use);
        const auto t_video1 = std::chrono::steady_clock::now();

        double ms_lane_exec = 0.0;
        double ms_pub_ld = 0.0;
        if (lane_mask.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "lane_mask is empty");
            publishLaneMarkers(LaneResult{}, msg->header.stamp);
        } else {
            const auto t_lane0 = std::chrono::steady_clock::now();
            LaneResult res = lld_->DetectFromMask(lane_mask, frame);
            if (show_debug_) {
                const cv::Mat dbg = lld_->getDebugImage();
                if (!dbg.empty()) {
                    cv::imshow("Lane Detection Process", dbg);
                    cv::waitKey(1);
                }
            }
            const auto t_lane1 = std::chrono::steady_clock::now();
            ms_lane_exec = std::chrono::duration<double, std::milli>(t_lane1 - t_lane0).count();

            if (pub_ld_) {
                const auto t_pub_ld0 = std::chrono::steady_clock::now();
                fo_msgs::msg::Cam2LD msg_ld;
                msg_ld.header_sen.stamp = msg->header.stamp;
                msg_ld.header_sen.frame_id = msg->header.frame_id;
                msg_ld.header_ld.stamp = this->now();
                msg_ld.header_ld.frame_id = "lane_detection";
                fillMsgLD(res, msg_ld);
                pub_ld_->publish(msg_ld);
                const auto t_pub_ld1 = std::chrono::steady_clock::now();
                ms_pub_ld = std::chrono::duration<double, std::milli>(t_pub_ld1 - t_pub_ld0).count();
                rolling_counter_ = (rolling_counter_ + 1) % 255;
            }
            publishLaneMarkers(res, msg->header.stamp);
        }

        double ms_pub_sf = 0.0;
        if (pub_sf_) {
            const auto t_pub_sf0 = std::chrono::steady_clock::now();
            fo_msgs::msg::Cam2DataForSF2 msg_sf;
            msg_sf.header = msg->header;
            if (!bbox_list_for_use.empty()) {
                msg_sf.objs_td.reserve(bbox_list_for_use.size());
                for (std::size_t i = 0; i < bbox_list_for_use.size(); ++i) {
                    const auto& bbox = bbox_list_for_use[i];
                    const auto obj = makeObj2SfFromBbox(bbox, static_cast<uint8_t>(i % 255));
                    if (obj.has_value()) {
                        msg_sf.objs_td.push_back(*obj);
                    }
                }
            }
            pub_sf_->publish(msg_sf);
            const auto t_pub_sf1 = std::chrono::steady_clock::now();
            ms_pub_sf = std::chrono::duration<double, std::milli>(t_pub_sf1 - t_pub_sf0).count();
        }

        const auto t4 = std::chrono::steady_clock::now();

        const double ms_decode = std::chrono::duration<double, std::milli>(t1 - t0).count();
        const double ms_infer_call = std::chrono::duration<double, std::milli>(t2 - t1).count();
        const double ms_mask = std::chrono::duration<double, std::milli>(t3 - t2).count();
        const double ms_publish = std::chrono::duration<double, std::milli>(t4 - t3).count();
        const double ms_total = std::chrono::duration<double, std::milli>(t4 - t0).count();
        const double ms_pub_mask = 0.0;
        const double ms_pub_io = ms_pub_sf + ms_pub_ld;
        const double ms_video = std::chrono::duration<double, std::milli>(t_video1 - t_video0).count();

        appendTimingSample(
            t4,
            ms_total, ms_decode, ms_infer_call, ms_mask, ms_publish,
            ms_pub_io, ms_pub_mask, ms_pub_ld, ms_lane_exec, ms_video,
            det_result.time_pre_process, det_result.time_inference, det_result.time_post_process);
        flushTimingWindow(false);
    }

    void appendTimingSample(
        const std::chrono::steady_clock::time_point& sample_time,
        double ms_total, double ms_decode, double ms_infer_call, double ms_mask, double ms_publish,
        double ms_pub_io, double ms_pub_mask, double ms_pub_ld, double ms_lane_exec, double ms_video,
        double engine_pre, double engine_inf, double engine_post)
    {
        if (!log_perf_) return;
        if (timing_window_sample_count_ == 0) {
            timing_window_start_ = sample_time;
        }

        ++timing_window_sample_count_;

        last_cb_total_ms_ = ms_total;
        last_cb_decode_ms_ = ms_decode;
        last_cb_infer_call_ms_ = ms_infer_call;
        last_cb_mask_ms_ = ms_mask;
        last_cb_publish_ms_ = ms_publish;
        last_cb_pub_io_ms_ = ms_pub_io;
        last_cb_pub_mask_ms_ = ms_pub_mask;
        last_cb_pub_ld_ms_ = ms_pub_ld;
        last_cb_lane_exec_ms_ = ms_lane_exec;
        last_cb_video_ms_ = ms_video;

        last_engine_pre_ms_ = engine_pre;
        last_engine_inf_ms_ = engine_inf;
        last_engine_post_ms_ = engine_post;
    }

    void clearTimingWindow()
    {
        timing_window_sample_count_ = 0;
    }

    void flushTimingWindow(bool force)
    {
        if (!log_perf_ || timing_window_sample_count_ == 0) return;

        const auto now = std::chrono::steady_clock::now();
        if (!force && (now - timing_window_start_ < std::chrono::seconds(1))) return;

        std::ostringstream line;
        line << std::fixed << std::setprecision(2)
             << "[Perf/1s] samples=" << timing_window_sample_count_
             << " \ncb(total=" << last_cb_total_ms_ << "ms"
             << " decode=" << last_cb_decode_ms_ << "ms"
             << " infer_call=" << last_cb_infer_call_ms_ << "ms"
             << " mask=" << last_cb_mask_ms_ << "ms"
             << " pub=" << last_cb_publish_ms_ << "ms"
             << " pub_io=" << last_cb_pub_io_ms_ << "ms"
             << " pub_mask=" << last_cb_pub_mask_ms_ << "ms"
             << " pub_ld=" << last_cb_pub_ld_ms_ << "ms"
             << " lane_exec=" << last_cb_lane_exec_ms_ << "ms"
             << " video=" << last_cb_video_ms_ << "ms)"
             << " \nengine(pre=" << last_engine_pre_ms_ << "ms"
             << " inf=" << last_engine_inf_ms_ << "ms"
             << " post=" << last_engine_post_ms_ << "ms)"
             << " \ndet(bbox_count=" << last_bbox_count_;
        if (last_bbox_count_ > 0) {
            line << " top_class_id=" << last_bbox_class_id_
                 << " top_label=" << last_bbox_label_
                 << " top_score=" << last_bbox_score_;
        }
        line << ")";
        RCLCPP_INFO(this->get_logger(), "%s", line.str().c_str());

        clearTimingWindow();
        timing_window_start_ = now;
    }

    std::string makeVideoFileName() const
    {
        const auto now = std::chrono::system_clock::now();
        const std::time_t now_tt = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm {};
#if defined(_WIN32)
        localtime_s(&now_tm, &now_tt);
#else
        localtime_r(&now_tt, &now_tm);
#endif
        std::ostringstream oss;
        oss << "lld_mask_" << std::put_time(&now_tm, "%y%m%d_%H%M%S") << ".mp4";
        return oss.str();
    }

    void initVideoWriterIfNeeded()
    {
        if (!log_video_ || video_writer_.isOpened() || video_writer_failed_) return;

        const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        const cv::Size output_size(kLogWidth_ * 2, kLogHeight_);
        if (!video_writer_.open(video_file_name_, fourcc, kLogFps_, output_size, true)) {
            video_writer_failed_ = true;
            RCLCPP_ERROR(this->get_logger(), "Failed to open video writer: %s", video_file_name_.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Video logging started: %s", video_file_name_.c_str());
    }

    void writeLogFrame(const cv::Mat& frame, const cv::Mat& vis_mask, const std::vector<BoundingBox>& bbox_list)
    {
        if (!log_video_ || frame.empty() || video_writer_failed_) return;

        initVideoWriterIfNeeded();
        if (!video_writer_.isOpened()) return;

        cv::Mat frame_small;
        cv::resize(frame, frame_small, cv::Size(kLogWidth_, kLogHeight_), 0.0, 0.0, cv::INTER_LINEAR);
        const float scale_x = static_cast<float>(kLogWidth_) / static_cast<float>(frame.cols);
        const float scale_y = static_cast<float>(kLogHeight_) / static_cast<float>(frame.rows);
        for (const auto& bbox : bbox_list) {
            int x = static_cast<int>(bbox.x * scale_x);
            int y = static_cast<int>(bbox.y * scale_y);
            int w = static_cast<int>(bbox.w * scale_x);
            int h = static_cast<int>(bbox.h * scale_y);

            x = (std::max)(0, (std::min)(x, kLogWidth_ - 1));
            y = (std::max)(0, (std::min)(y, kLogHeight_ - 1));
            w = (std::max)(0, (std::min)(w, kLogWidth_ - x));
            h = (std::max)(0, (std::min)(h, kLogHeight_ - y));
            if (w <= 0 || h <= 0) continue;

            cv::rectangle(frame_small, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 255), 2);

            std::ostringstream label_ss;
            label_ss << bbox.label << ":" << std::fixed << std::setprecision(2) << bbox.score;
            cv::putText(frame_small, label_ss.str(), cv::Point(x, (std::max)(0, y - 4)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }

        cv::Mat mask_small;
        if (vis_mask.empty()) {
            mask_small = cv::Mat::zeros(kLogHeight_, kLogWidth_, CV_8UC1);
        } else {
            cv::resize(vis_mask, mask_small, cv::Size(kLogWidth_, kLogHeight_), 0.0, 0.0, cv::INTER_NEAREST);
        }

        cv::Mat mask_bgr;
        if (mask_small.channels() == 1) {
            cv::cvtColor(mask_small, mask_bgr, cv::COLOR_GRAY2BGR);
        } else {
            mask_bgr = mask_small;
        }

        cv::Mat composed;
        std::vector<cv::Mat> views = {frame_small, mask_bgr};
        cv::hconcat(views, composed);

        video_writer_.write(composed);
        ++logged_frame_count_;
    }

    void closeVideoWriter()
    {
        if (!video_writer_.isOpened()) return;
        video_writer_.release();
        RCLCPP_INFO(this->get_logger(), "Video logging finished: %s (frames=%zu)",
            video_file_name_.c_str(), logged_frame_count_);
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
    bool log_video_ = false;
    bool log_perf_ = true;
    bool show_debug_ = false;
    bool video_writer_failed_ = false;
    uint8_t rolling_counter_ = 0;
    bool enable_lane_marker_ = true;
    std::string lane_marker_frame_id_ = "camera_link";
    double lane_marker_forward_min_m_ = 0.0;
    double lane_marker_forward_max_m_ = 30.0;
    double lane_marker_dx_m_ = 0.5;
    double lane_marker_line_width_m_ = 0.12;
    double lane_marker_origin_radius_m_ = 0.25;
    double lane_marker_z_m_ = 0.02;
    std::size_t logged_frame_count_ = 0;
    std::string video_file_name_;
    cv::VideoWriter video_writer_;
    std::chrono::steady_clock::time_point timing_window_start_;
    std::size_t timing_window_sample_count_ = 0;

    double last_cb_total_ms_ = 0.0;
    double last_cb_decode_ms_ = 0.0;
    double last_cb_infer_call_ms_ = 0.0;
    double last_cb_mask_ms_ = 0.0;
    double last_cb_publish_ms_ = 0.0;
    double last_cb_pub_io_ms_ = 0.0;
    double last_cb_pub_mask_ms_ = 0.0;
    double last_cb_pub_ld_ms_ = 0.0;
    double last_cb_lane_exec_ms_ = 0.0;
    double last_cb_video_ms_ = 0.0;

    double last_engine_pre_ms_ = 0.0;
    double last_engine_inf_ms_ = 0.0;
    double last_engine_post_ms_ = 0.0;

    std::size_t last_bbox_count_ = 0;
    int32_t last_bbox_class_id_ = -1;
    std::string last_bbox_label_;
    float last_bbox_score_ = 0.0F;

    CameraConfig cam_conf_;
    CalibData calib_;
    std::unique_ptr<BboxGroundProjector> bbox_projector_;

    DetectionEngine engine_;
    std::unique_ptr<LaneLineDetector> lld_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
    rclcpp::Publisher<fo_msgs::msg::Cam2LD>::SharedPtr pub_ld_;
    rclcpp::Publisher<fo_msgs::msg::Cam2DataForSF2>::SharedPtr pub_sf_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_lane_marker_;


    static uint8_t mapCustomClassIdToSfType(const int32_t class_id)
    {
        switch (class_id) {
            case 0: return 2; // bus
            case 1: return 3; // bicycle / motorcycle
            case 2: return 2; // truck
            case 3: return 1; // car
            case 4: return 4; // person
            default: return 0;
        }
    }

    std::optional<fo_msgs::msg::Obj2SF> makeObj2SfFromBbox(const BoundingBox& bbox, const uint8_t track_id_hint) const
    {
        if (!bbox_projector_) {
            return std::nullopt;
        }

        const cv::Rect2d bbox_rect(
            static_cast<double>(bbox.x),
            static_cast<double>(bbox.y),
            static_cast<double>(bbox.w),
            static_cast<double>(bbox.h));
        const GroundPointEstimate estimate = bbox_projector_->EstimateRect(bbox_rect);
        if (!estimate.valid) {
            return std::nullopt;
        }

        fo_msgs::msg::Obj2SF obj;
        obj.cam2_track_id = track_id_hint;
        obj.object_type = mapCustomClassIdToSfType(bbox.class_id);
        obj.confidence = bbox.score;
        obj.pos_x = static_cast<float>(estimate.ground_xy_m.x);
        obj.pos_y = static_cast<float>(estimate.ground_xy_m.y);
        return obj;
    }

    static constexpr int kLogWidth_ = 640;
    static constexpr int kLogHeight_ = 360;
    static constexpr double kLogFps_ = 30.0;
    static constexpr const char* kLdTopic_ = "/camera/lane_result";
    static constexpr const char* kSfTopic_ = "/camera/sf_objs";
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
