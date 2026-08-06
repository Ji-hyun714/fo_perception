#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "fo_msgs/msg/cam2_data_for_sf2.hpp"
#include "fo_msgs/msg/cam2_od.hpp"
#include "fo_msgs/msg/cam2_td.hpp"
#include "fo_msgs/msg/cam2_tl.hpp"
#include "lane_detection/helper_tracking.hpp"
#include "std_msgs/msg/header.hpp"

#include <utils/BboxGroundProjector.hpp>
#include <utils/CalibData.hpp>

struct ObjectBuilderConfig
{
  double max_range_m{120.0};
  double min_bbox_height_px{4.0};
  float td_validity_threshold{0.25F};
  float tl_validity_threshold{0.30F};
};

struct RoutedTrackedObjects
{
  std::vector<TrackedObject> td_objects;
  std::vector<TrackedObject> tl_objects;
  std::vector<TrackedObject> od_objects;
};

class ObjectMessageBuilder
{
public:
  ObjectMessageBuilder(const CalibData & calib, const ObjectBuilderConfig & config);

  RoutedTrackedObjects routeTrackedObjects(const std::vector<TrackedObject> & tracked_objects) const;

  fo_msgs::msg::Cam2TD buildCam2TD(
    const std::vector<TrackedObject> & td_objects,
    uint8_t rolling_counter) const;

  fo_msgs::msg::Cam2TL buildCam2TL(const std::vector<TrackedObject> & tl_objects) const;

  fo_msgs::msg::Cam2OD buildCam2OD(
    const std::vector<TrackedObject> & od_objects,
    uint8_t rolling_counter) const;

  fo_msgs::msg::Cam2DataForSF2 buildSfMsg(
    const std::vector<TrackedObject> & td_objects,
    const std::vector<TrackedObject> & od_objects,
    const std_msgs::msg::Header & header) const;

private:
  enum class ObjectDomain : uint8_t
  {
    TD,
    TL,
    OD,
    Ignore
  };

  struct ProjectionResult
  {
    bool valid{false};
    float x_m{0.0F};
    float y_m{0.0F};
    float range_m{0.0F};
    float angle_rad{0.0F};
  };

  ObjectDomain classify(int32_t class_id) const;
  ProjectionResult project(const TrackedObject & obj) const;
  uint8_t mapTdType(int32_t class_id) const;
  uint8_t mapOdType(int32_t class_id) const;

  ObjectBuilderConfig config_{};
  std::unique_ptr<BboxGroundProjector> projector_;
  std::array<ObjectDomain, 80> class_domain_map_{};
  std::unordered_map<int32_t, uint8_t> td_label_map_;
  std::unordered_map<int32_t, uint8_t> od_label_map_;
};
