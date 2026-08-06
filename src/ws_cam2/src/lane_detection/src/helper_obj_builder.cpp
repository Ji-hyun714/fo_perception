#include "lane_detection/helper_obj_builder.hpp"

#include <algorithm>
#include <cmath>

namespace
{

constexpr std::size_t kCamTrackNum = 16;

}  // namespace

ObjectMessageBuilder::ObjectMessageBuilder(const CalibData & calib, const ObjectBuilderConfig & config)
: config_(config)
{
  GroundProjectorOptions options;
  options.min_bbox_height_px = config_.min_bbox_height_px;
  options.max_range_m = config_.max_range_m;
  projector_ = std::make_unique<BboxGroundProjector>(calib, options);

  class_domain_map_.fill(ObjectDomain::Ignore);
  class_domain_map_[0] = ObjectDomain::TD;
  class_domain_map_[1] = ObjectDomain::TD;
  class_domain_map_[2] = ObjectDomain::TD;
  class_domain_map_[3] = ObjectDomain::TD;
  class_domain_map_[5] = ObjectDomain::TD;
  class_domain_map_[7] = ObjectDomain::TD;
  class_domain_map_[9] = ObjectDomain::TL;

  td_label_map_ = {
    {0, 0x4}, // coco 기준
    {1, 0x3},
    {2, 0x1},
    {3, 0x3},
    {5, 0x2},
    {7, 0x2},
  };
}

RoutedTrackedObjects ObjectMessageBuilder::routeTrackedObjects(
  const std::vector<TrackedObject> & tracked_objects) const
{
  RoutedTrackedObjects routed;

  for (const auto & tracked : tracked_objects) {
    switch (classify(tracked.class_id)) {
      case ObjectDomain::TD:
        routed.td_objects.push_back(tracked);
        break;
      case ObjectDomain::TL:
        routed.tl_objects.push_back(tracked);
        break;
      case ObjectDomain::OD:
        routed.od_objects.push_back(tracked);
        break;
      case ObjectDomain::Ignore:
      default:
        break;
    }
  }

  return routed;
}

fo_msgs::msg::Cam2TD ObjectMessageBuilder::buildCam2TD(
  const std::vector<TrackedObject> & td_objects,
  uint8_t rolling_counter) const
{
  fo_msgs::msg::Cam2TD msg;

  for (std::size_t slot = 0; slot < std::min(td_objects.size(), kCamTrackNum); ++slot) {
    const auto & obj = td_objects[slot];
    const auto projection = project(obj);

    auto & td_a = msg.tda[slot];
    auto & td_b = msg.tdb[slot];

    td_a.rolling_counter = rolling_counter;
    td_a.object_age = static_cast<uint8_t>(obj.track_age % 256U);
    td_a.angle_rate = 0.0F;
    td_a.angle_left = projection.angle_rad > 0.0F ? projection.angle_rad : 0.0F;
    td_a.angle_right = projection.angle_rad < 0.0F ? std::abs(projection.angle_rad) : 0.0F;
    td_a.motion_status = obj.track_status;
    td_a.object_lane = 0U;
    td_a.cam2_obstacle_brake_lights = 0U;

    td_b.rolling_counter = rolling_counter;
    td_b.range = projection.range_m;
    td_b.object_vaildity = (obj.score > config_.td_validity_threshold && projection.valid) ? 1U : 0U;
    td_b.range_rate = 0.0F;
    td_b.cam2_obstacle_physical_width = obj.w;
    td_b.cam2_track_id = static_cast<uint8_t>(obj.track_id % 256U);
    td_b.object_type = mapTdType(obj.class_id);
  }

  return msg;
}

fo_msgs::msg::Cam2TL ObjectMessageBuilder::buildCam2TL(
  const std::vector<TrackedObject> & tl_objects) const
{
  fo_msgs::msg::Cam2TL msg;
  if (tl_objects.empty()) {
    return msg;
  }

  const auto best_it = std::max_element(
    tl_objects.begin(), tl_objects.end(),
    [](const TrackedObject & lhs, const TrackedObject & rhs) {
      return lhs.score < rhs.score;
    });

  msg.traffic_light_data = 0U;
  msg.traffic_light_valid_flag = best_it->score > config_.tl_validity_threshold ? 1U : 0U;
  msg.traffic_light_accuracy = static_cast<uint16_t>(
    std::clamp(best_it->score * 100.0F, 0.0F, 65535.0F));
  return msg;
}

fo_msgs::msg::Cam2OD ObjectMessageBuilder::buildCam2OD(
  const std::vector<TrackedObject> & od_objects,
  uint8_t rolling_counter) const
{
  fo_msgs::msg::Cam2OD msg;

  for (std::size_t slot = 0; slot < std::min(od_objects.size(), kCamTrackNum); ++slot) {
    const auto & obj = od_objects[slot];
    const auto projection = project(obj);
    auto & od = msg.sod[slot];
    od.rolling_count_1 = rolling_counter;
    od.camera2_static_obj_type = mapOdType(obj.class_id);
    od.static_object_status = 0U;
    od.static_object_pos_y = projection.y_m;
    od.static_object_pos_x = projection.x_m;
    od.static_object_pos2_y = 0.0F;
    od.static_object_pos2_x = 0.0F;
  }

  return msg;
}

fo_msgs::msg::Cam2DataForSF2 ObjectMessageBuilder::buildSfMsg(
  const std::vector<TrackedObject> & td_objects,
  const std::vector<TrackedObject> & od_objects,
  const std_msgs::msg::Header & header) const
{
  fo_msgs::msg::Cam2DataForSF2 msg;
  msg.header = header;

  for (const auto & obj : td_objects) {
    const auto projection = project(obj);
    if (!projection.valid) {
      continue;
    }

    fo_msgs::msg::Obj2SF sf_obj;
    sf_obj.cam2_track_id = static_cast<uint8_t>(obj.track_id % 256U);
    sf_obj.object_type = mapTdType(obj.class_id);
    sf_obj.confidence = obj.score;
    sf_obj.pos_x = projection.x_m;
    sf_obj.pos_y = projection.y_m;
    msg.objs_td.push_back(sf_obj);
  }

  for (const auto & obj : od_objects) {
    const auto projection = project(obj);
    if (!projection.valid) {
      continue;
    }

    fo_msgs::msg::Obj2SF sf_obj;
    sf_obj.cam2_track_id = static_cast<uint8_t>(obj.track_id % 256U);
    sf_obj.object_type = mapOdType(obj.class_id);
    sf_obj.confidence = obj.score;
    sf_obj.pos_x = projection.x_m;
    sf_obj.pos_y = projection.y_m;
    msg.objs_od.push_back(sf_obj);
  }

  return msg;
}

ObjectMessageBuilder::ObjectDomain ObjectMessageBuilder::classify(int32_t class_id) const
{
  if (class_id < 0 || static_cast<std::size_t>(class_id) >= class_domain_map_.size()) {
    return ObjectDomain::Ignore;
  }
  return class_domain_map_[class_id];
}

ObjectMessageBuilder::ProjectionResult ObjectMessageBuilder::project(const TrackedObject & obj) const
{
  ProjectionResult result;
  if (!projector_) {
    return result;
  }

  const cv::Rect2d rect(
    static_cast<double>(obj.x),
    static_cast<double>(obj.y),
    static_cast<double>(obj.w),
    static_cast<double>(obj.h));

  const auto estimate = projector_->EstimateRect(rect);
  if (!estimate.valid) {
    return result;
  }

  result.valid = true;
  result.x_m = static_cast<float>(estimate.ground_xy_m.x);
  result.y_m = static_cast<float>(estimate.ground_xy_m.y);
  result.range_m = std::hypot(result.x_m, result.y_m);
  result.angle_rad = std::atan2(result.y_m, result.x_m);
  return result;
}

uint8_t ObjectMessageBuilder::mapTdType(int32_t class_id) const
{
  const auto it = td_label_map_.find(class_id);
  if (it == td_label_map_.end()) {
    return 0U;
  }
  return it->second;
}

uint8_t ObjectMessageBuilder::mapOdType(int32_t class_id) const
{
  const auto it = od_label_map_.find(class_id);
  if (it == od_label_map_.end()) {
    return 0U;
  }
  return it->second;
}
