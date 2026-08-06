#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "ByteTrack/BYTETracker.h"
#include "detection_engine.h"
#include "fo_msgs/msg/object2_d.hpp"

struct TrackingConfig
{
  int frame_rate{30};
  int track_buffer{30};
  float track_thresh{0.5F};
  float high_thresh{0.6F};
  float match_thresh{0.8F};
};

struct TrackedObject
{
  int32_t class_id{0};
  float score{0.0F};
  float x{0.0F};
  float y{0.0F};
  float w{0.0F};
  float h{0.0F};
  uint32_t track_id{0U};
  uint32_t track_age{0U};
  uint8_t track_status{fo_msgs::msg::Object2D::TRACK_STATUS_NEW};
};

class TrackingHelper
{
public:
  explicit TrackingHelper(const TrackingConfig & config);

  std::vector<TrackedObject> run(const std::vector<BoundingBox> & detections);

private:
  static float calcIou(const byte_track::Rect<float> & track_rect, const BoundingBox & det);
  int32_t resolveClassId(const byte_track::STrack & track, const std::vector<BoundingBox> & detections) const;
  void pruneTrackClassCache(const std::vector<std::size_t> & active_track_ids);
  static uint8_t convertTrackStatus(byte_track::STrackState state);

  byte_track::BYTETracker tracker_;
  std::unordered_map<std::size_t, int32_t> track_id_to_class_id_;
};
