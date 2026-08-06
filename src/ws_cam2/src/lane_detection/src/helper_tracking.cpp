#include "lane_detection/helper_tracking.hpp"

#include <algorithm>
#include <limits>

TrackingHelper::TrackingHelper(const TrackingConfig & config)
: tracker_(
    config.frame_rate,
    config.track_buffer,
    config.track_thresh,
    config.high_thresh,
    config.match_thresh)
{
}

std::vector<TrackedObject> TrackingHelper::run(const std::vector<BoundingBox> & detections)
{
  std::vector<byte_track::Object> objects;
  objects.reserve(detections.size());

  for (const auto & bbox : detections) {
    if (bbox.w <= 0 || bbox.h <= 0) {
      continue;
    }

    objects.emplace_back(
      byte_track::Rect<float>(
        static_cast<float>(bbox.x),
        static_cast<float>(bbox.y),
        static_cast<float>(bbox.w),
        static_cast<float>(bbox.h)),
      bbox.class_id,
      bbox.score);
  }

  const auto tracks = tracker_.update(objects);

  std::vector<TrackedObject> tracked_objects;
  tracked_objects.reserve(tracks.size());

  std::vector<std::size_t> active_track_ids;
  active_track_ids.reserve(tracks.size());

  for (const auto & track : tracks) {
    const auto & rect = track->getRect();

    TrackedObject tracked;
    tracked.class_id = resolveClassId(*track, detections);
    tracked.score = track->getScore();
    tracked.x = rect.x();
    tracked.y = rect.y();
    tracked.w = rect.width();
    tracked.h = rect.height();
    tracked.track_id = static_cast<uint32_t>(track->getTrackId());
    tracked.track_age = static_cast<uint32_t>(
      (track->getFrameId() - track->getStartFrameId()) % 256U);
    tracked.track_status = convertTrackStatus(track->getSTrackState());

    track_id_to_class_id_[track->getTrackId()] = tracked.class_id;
    active_track_ids.push_back(track->getTrackId());
    tracked_objects.push_back(tracked);
  }

  pruneTrackClassCache(active_track_ids);
  return tracked_objects;
}

float TrackingHelper::calcIou(const byte_track::Rect<float> & track_rect, const BoundingBox & det)
{
  const float det_x = static_cast<float>(det.x);
  const float det_y = static_cast<float>(det.y);
  const float det_w = static_cast<float>(det.w);
  const float det_h = static_cast<float>(det.h);

  const float x1 = std::max(track_rect.x(), det_x);
  const float y1 = std::max(track_rect.y(), det_y);
  const float x2 = std::min(track_rect.x() + track_rect.width(), det_x + det_w);
  const float y2 = std::min(track_rect.y() + track_rect.height(), det_y + det_h);

  const float inter_w = std::max(0.0F, x2 - x1);
  const float inter_h = std::max(0.0F, y2 - y1);
  const float inter_area = inter_w * inter_h;
  const float track_area = std::max(0.0F, track_rect.width()) * std::max(0.0F, track_rect.height());
  const float det_area = std::max(0.0F, det_w) * std::max(0.0F, det_h);
  const float union_area = track_area + det_area - inter_area;

  if (union_area <= std::numeric_limits<float>::epsilon()) {
    return 0.0F;
  }
  return inter_area / union_area;
}

int32_t TrackingHelper::resolveClassId(
  const byte_track::STrack & track,
  const std::vector<BoundingBox> & detections) const
{
  const auto & track_rect = track.getRect();
  float best_iou = 0.0F;
  int32_t best_class_id = -1;

  for (const auto & det : detections) {
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

void TrackingHelper::pruneTrackClassCache(const std::vector<std::size_t> & active_track_ids)
{
  for (auto it = track_id_to_class_id_.begin(); it != track_id_to_class_id_.end();) {
    if (std::find(active_track_ids.begin(), active_track_ids.end(), it->first) == active_track_ids.end()) {
      it = track_id_to_class_id_.erase(it);
    } else {
      ++it;
    }
  }
}

uint8_t TrackingHelper::convertTrackStatus(byte_track::STrackState state)
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
