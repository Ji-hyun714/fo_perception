#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ByteTrack/BYTETracker.h"
#include "fo_msgs/msg/cam2_data_for_sf2.hpp"
#include "fo_msgs/msg/object2_d.hpp"
#include "fo_msgs/msg/track_results.hpp"
#include <utils/BboxGroundProjector.hpp>
#include <utils/CalibData.hpp>
#include <utils/common.hpp>

using std::placeholders::_1;

class ObjTrackingNode : public rclcpp::Node
{
public:
    ObjTrackingNode()
    : Node("obj_tracking_node"),
      tracker_(declareAndGetFrameRate(),
               declareAndGetTrackBuffer(),
               declareAndGetFloatParam("track_thresh", 0.5F),
               declareAndGetFloatParam("high_thresh", 0.6F),
               declareAndGetFloatParam("match_thresh", 0.8F))
    {
        const std::string input_topic_objs =
            this->declare_parameter<std::string>("input_topic_objs", "/camera/detections");
        const std::string output_topic_tracks =
            this->declare_parameter<std::string>("output_topic_tracks", "/camera/tracks");
        const std::string output_topic_sf =
            this->declare_parameter<std::string>("output_topic_sf", "/camera/sf_objs");

        std::string yaml_path = this->declare_parameter<std::string>("yaml_path", "");
        const int qos_depth = this->declare_parameter<int>("queue_size", 1);

        debug_mode_ = this->declare_parameter<bool>("debug_mode", false);
        const std::string output_topic_tracks_debug =
            this->declare_parameter<std::string>("output_topic_tracks_debug", "/camera/tracks/debug_image");

        if (yaml_path.empty()) {
            yaml_path = "/home/wise/adsp_perception_camera/src/utils/yaml/econ_260311_refined.yaml";
        }

        cam_conf_ = getCameraConfigFromYaml(yaml_path);
        if (cam_conf_.model == 0) {
            calib_ = CalibData::MakePinhole(cam_conf_.image_size, cam_conf_.K, cam_conf_.D, cam_conf_.R, cam_conf_.t);
        } else {
            calib_ = CalibData::MakeFisheye(cam_conf_.image_size, cam_conf_.K, cam_conf_.D, cam_conf_.R, cam_conf_.t);
        }
        bbox_projector_ = std::make_unique<BboxGroundProjector>(calib_);

        sub_detections_ = this->create_subscription<fo_msgs::msg::TrackResults>(
            input_topic_objs, qos_depth, std::bind(&ObjTrackingNode::callbackDetections, this, _1));
        pub_tracks_ = this->create_publisher<fo_msgs::msg::TrackResults>(output_topic_tracks, qos_depth);
        pub_sf_ = this->create_publisher<fo_msgs::msg::Cam2DataForSF2>(output_topic_sf, qos_depth);
        if (debug_mode_) {
            pub_tracks_debug_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_tracks_debug, qos_depth);
        }

        RCLCPP_INFO(this->get_logger(), "ObjTrackingNode started. input=%s tracks=%s sf=%s debug_mode=%s",
                    input_topic_objs.c_str(), output_topic_tracks.c_str(), output_topic_sf.c_str(),
                    debug_mode_ ? "true" : "false");
        if (debug_mode_) {
            RCLCPP_INFO(this->get_logger(), "tracks debug image topic=%s", output_topic_tracks_debug.c_str());
        }
    }

private:
    int declareAndGetFrameRate()
    {
        return this->declare_parameter<int>("frame_rate", 30);
    }

    int declareAndGetTrackBuffer()
    {
        return this->declare_parameter<int>("track_buffer", 30);
    }

    float declareAndGetFloatParam(const std::string &name, const float default_value)
    {
        return static_cast<float>(this->declare_parameter<double>(name, default_value));
    }

    void callbackDetections(const fo_msgs::msg::TrackResults::SharedPtr msg)
    {
        if (!pub_tracks_) {
            return;
        }

        std::vector<byte_track::Object> objects;
        objects.reserve(msg->objs.size());

        for (const auto &obj : msg->objs) {
            objects.emplace_back(
                byte_track::Rect<float>(obj.x, obj.y, obj.w, obj.h),
                obj.class_id,
                obj.score);
        }

        const auto tracks = tracker_.update(objects);
        auto out_msg = buildTrackedMsg(tracks, *msg);
        pub_tracks_->publish(out_msg);
        if (pub_sf_) {
            pub_sf_->publish(buildSfMsg(out_msg));
        }
        publishDebugImage(out_msg); // debug_mode 일 때만
    }

    fo_msgs::msg::TrackResults buildTrackedMsg(
        const std::vector<byte_track::BYTETracker::STrackPtr> &tracks,
        const fo_msgs::msg::TrackResults &src_msg)
    {
        fo_msgs::msg::TrackResults out_msg;
        out_msg.header = src_msg.header;
        out_msg.header.stamp = this->now();
        out_msg.objs.reserve(tracks.size());
        std::set<std::size_t> active_track_ids;

        for (const auto &track : tracks) {
            fo_msgs::msg::Object2D out_obj;
            const auto &rect = track->getRect();
            const auto class_id = resolveClassId(*track, src_msg.objs);

            out_obj.class_id = class_id;
            out_obj.score = track->getScore();
            out_obj.x = rect.x();
            out_obj.y = rect.y();
            out_obj.w = rect.width();
            out_obj.h = rect.height();
            // out_obj.validity = false;
            out_obj.validity = out_obj.score > threshold_valid ? true : false;

            const auto estimate = estimateGroundPoint(rect);
            if (estimate.has_value()) {
                out_obj.x_m = static_cast<float>(estimate->ground_xy_m.x);
                out_obj.y_m = static_cast<float>(estimate->ground_xy_m.y);
            } else {
                out_obj.x_m = 0.0F;
                out_obj.y_m = 0.0F;
            }
            out_obj.range_m = 0.0F;
            out_obj.angle_deg = 0.0F;
            
            out_obj.track_id = static_cast<uint32_t>(track->getTrackId());
            out_obj.track_age = static_cast<uint32_t>((track->getFrameId() - track->getStartFrameId()) % 256U);
            out_obj.track_status = convertTrackStatus(track->getSTrackState());

            track_id_to_class_id_[track->getTrackId()] = class_id;
            active_track_ids.insert(track->getTrackId());
            out_msg.objs.push_back(out_obj);
        }

        pruneTrackClassCache(active_track_ids);

        return out_msg;
    }

    int32_t resolveClassId(
        const byte_track::STrack &track,
        const std::vector<fo_msgs::msg::Object2D> &det_objs) const
    {
        const auto &track_rect = track.getRect();
        float best_iou = 0.0F;
        int32_t best_class_id = -1;

        for (const auto &det : det_objs) {
            const float iou = calcIou(track_rect, det);
            if (iou > best_iou) {
                best_iou = iou;
                best_class_id = det.class_id;
            }
        }

        if (best_class_id >= 0) {
            return best_class_id;
        }

        const auto it = track_id_to_class_id_.find(track.getTrackId());
        if (it != track_id_to_class_id_.end()) {
            return it->second;
        }

        return 0;
    }

    void pruneTrackClassCache(const std::set<std::size_t> &active_track_ids)
    {
        for (auto it = track_id_to_class_id_.begin(); it != track_id_to_class_id_.end(); ) {
            if (active_track_ids.find(it->first) == active_track_ids.end()) {
                it = track_id_to_class_id_.erase(it);
            } else {
                ++it;
            }
        }
    }

    std::optional<GroundPointEstimate> estimateGroundPoint(const byte_track::Rect<float> &rect) const
    {
        if (!bbox_projector_) {
            return std::nullopt;
        }

        const cv::Rect2d bbox_rect(
            static_cast<double>(rect.x()),
            static_cast<double>(rect.y()),
            static_cast<double>(rect.width()),
            static_cast<double>(rect.height()));

        const GroundPointEstimate estimate = bbox_projector_->EstimateRect(bbox_rect);
        if (!estimate.valid) {
            return std::nullopt;
        }
        return estimate;
    }

    fo_msgs::msg::Cam2DataForSF2 buildSfMsg(const fo_msgs::msg::TrackResults &tracks_msg) const
    {
        fo_msgs::msg::Cam2DataForSF2 msg_sf;
        msg_sf.header = tracks_msg.header;
        msg_sf.objs_td.reserve(tracks_msg.objs.size());

        for (const auto &obj : tracks_msg.objs) {
            if (!std::isfinite(obj.x_m) || !std::isfinite(obj.y_m)) {
                continue;
            }

            fo_msgs::msg::Obj2SF sf_obj;
            sf_obj.cam2_track_id = static_cast<uint8_t>(obj.track_id % 256U);
            sf_obj.object_type = mapCustomClassIdToSfType(obj.class_id);
            sf_obj.confidence = obj.score;
            sf_obj.pos_x = obj.x_m;
            sf_obj.pos_y = obj.y_m;
            msg_sf.objs_td.push_back(sf_obj);
        }

        return msg_sf;
    }

    void publishDebugImage(const fo_msgs::msg::TrackResults &tracks_msg) const
    {
        if (!debug_mode_ || !pub_tracks_debug_) {
            return;
        }

        cv::Mat debug_image(kDebugImageHeight_, kDebugImageWidth_, CV_8UC3, cv::Scalar(0, 0, 0));

        for (const auto &obj : tracks_msg.objs) {
            int x = static_cast<int>(std::round(obj.x));
            int y = static_cast<int>(std::round(obj.y));
            int w = static_cast<int>(std::round(obj.w));
            int h = static_cast<int>(std::round(obj.h));

            x = std::clamp(x, 0, kDebugImageWidth_ - 1);
            y = std::clamp(y, 0, kDebugImageHeight_ - 1);
            w = std::max(0, std::min(w, kDebugImageWidth_ - x));
            h = std::max(0, std::min(h, kDebugImageHeight_ - y));
            if (w <= 0 || h <= 0) {
                continue;
            }

            cv::rectangle(debug_image, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

            const std::string label = std::to_string(obj.track_id);
            cv::putText(debug_image, label, cv::Point(x, std::max(0, y - 4)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }

        auto debug_msg = cv_bridge::CvImage(
            tracks_msg.header, sensor_msgs::image_encodings::BGR8, debug_image).toImageMsg();
        pub_tracks_debug_->publish(*debug_msg);
    }

    static uint8_t convertTrackStatus(const byte_track::STrackState state)
    {
        switch (state) {
            case byte_track::STrackState::New:
                return fo_msgs::msg::Object2D::TRACK_STATUS_NEW;
            case byte_track::STrackState::Tracked:
                return fo_msgs::msg::Object2D::TRACK_STATUS_TRACKED;
            case byte_track::STrackState::Lost:
                return fo_msgs::msg::Object2D::TRACK_STATUS_LOST;
            case byte_track::STrackState::Removed:
            default:
                return fo_msgs::msg::Object2D::TRACK_STATUS_REMOVED;
        }
    }

    static uint8_t mapCustomClassIdToSfType(const int32_t class_id)
    {
        switch (class_id) {
            case 0: return 2;
            case 1: return 3;
            case 2: return 2;
            case 3: return 1;
            case 4: return 4;
            default: return 0;
        }
    }

    static float calcIou(const byte_track::Rect<float> &track_rect, const fo_msgs::msg::Object2D &det)
    {
        const float x1 = std::max(track_rect.x(), det.x);
        const float y1 = std::max(track_rect.y(), det.y);
        const float x2 = std::min(track_rect.x() + track_rect.width(), det.x + det.w);
        const float y2 = std::min(track_rect.y() + track_rect.height(), det.y + det.h);

        const float inter_w = std::max(0.0F, x2 - x1);
        const float inter_h = std::max(0.0F, y2 - y1);
        const float inter_area = inter_w * inter_h;
        const float track_area = std::max(0.0F, track_rect.width()) * std::max(0.0F, track_rect.height());
        const float det_area = std::max(0.0F, det.w) * std::max(0.0F, det.h);
        const float union_area = track_area + det_area - inter_area;

        if (union_area <= std::numeric_limits<float>::epsilon()) {
            return 0.0F;
        }
        return inter_area / union_area;
    }

    byte_track::BYTETracker tracker_;
    std::unordered_map<std::size_t, int32_t> track_id_to_class_id_;
    bool debug_mode_ = false;
    CameraConfig cam_conf_;
    CalibData calib_;
    std::unique_ptr<BboxGroundProjector> bbox_projector_;

    rclcpp::Subscription<fo_msgs::msg::TrackResults>::SharedPtr sub_detections_;
    rclcpp::Publisher<fo_msgs::msg::TrackResults>::SharedPtr pub_tracks_;
    rclcpp::Publisher<fo_msgs::msg::Cam2DataForSF2>::SharedPtr pub_sf_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_tracks_debug_;

    static constexpr int kDebugImageWidth_ = 1280;
    static constexpr int kDebugImageHeight_ = 720;

    float threshold_valid = 0.25;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjTrackingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
