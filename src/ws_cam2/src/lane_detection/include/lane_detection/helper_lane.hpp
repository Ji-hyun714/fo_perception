#pragma once

#include <cstdint>
#include <string>

#include "fo_msgs/msg/cam2_ld.hpp"
#include "lane_detection/LaneDetection.hpp"
#include "std_msgs/msg/header.hpp"

uint8_t laneStateToQuality(LaneState state);

fo_msgs::msg::Cam2LD buildCam2LD(
  const LaneResult & res,
  const std_msgs::msg::Header & sensor_header,
  const std::string & processing_frame_id,
  uint8_t rolling_counter);
