#include "lane_detection/helper_lane.hpp"

#include <algorithm>

uint8_t laneStateToQuality(LaneState state)
{
  switch (state) {
    case LaneState::GOOD:
      return 3;
    case LaneState::WEAK:
      return 2;
    case LaneState::BAD:
      return 1;
    case LaneState::NODET:
    default:
      return 0;
  }
}

fo_msgs::msg::Cam2LD buildCam2LD(
  const LaneResult & res,
  const std_msgs::msg::Header & sensor_header,
  const std::string & processing_frame_id,
  uint8_t rolling_counter)
{
  fo_msgs::msg::Cam2LD msg_pub;
  msg_pub.header_sen = sensor_header;
  msg_pub.header_ld.stamp = sensor_header.stamp;
  msg_pub.header_ld.frame_id = processing_frame_id;

  const uint8_t quality_l = laneStateToQuality(res.state_L);
  const uint8_t quality_r = laneStateToQuality(res.state_R);
  const bool use_left_coeffs = (quality_l >= 2);
  const bool use_right_coeffs = (quality_r >= 2);

  const float left_model_a = use_left_coeffs ? static_cast<float>(res.coeffs_L[0]) : 0.0F;
  const float left_heading = use_left_coeffs ? static_cast<float>(res.coeffs_L[1]) : 0.0F;
  const float left_pos = use_left_coeffs ? static_cast<float>(res.coeffs_L[2]) : 0.0F;
  const float left_model_da = use_left_coeffs ? static_cast<float>(res.coeffs_L[3]) : 0.0F;
  const float left_view_range_m = std::max(0.0F, static_cast<float>(res.view_range_m_L));
  const float left_lane_start_m =
    use_left_coeffs ? std::max(0.0F, static_cast<float>(res.lane_start_m_L)) : 0.0F;
  const float left_lane_end_m =
    use_left_coeffs ? std::max(0.0F, static_cast<float>(res.lane_end_m_L)) : 0.0F;
  const uint8_t left_view_validity = (res.availability_L > 0) ? 1U : 0U;

  const float right_model_a = use_right_coeffs ? static_cast<float>(res.coeffs_R[0]) : 0.0F;
  const float right_heading = use_right_coeffs ? static_cast<float>(res.coeffs_R[1]) : 0.0F;
  const float right_pos = use_right_coeffs ? static_cast<float>(res.coeffs_R[2]) : 0.0F;
  const float right_model_da = use_right_coeffs ? static_cast<float>(res.coeffs_R[3]) : 0.0F;
  const float right_view_range_m = std::max(0.0F, static_cast<float>(res.view_range_m_R));
  const float right_lane_start_m =
    use_right_coeffs ? std::max(0.0F, static_cast<float>(res.lane_start_m_R)) : 0.0F;
  const float right_lane_end_m =
    use_right_coeffs ? std::max(0.0F, static_cast<float>(res.lane_end_m_R)) : 0.0F;
  const uint8_t right_view_validity = (res.availability_R > 0) ? 1U : 0U;

  msg_pub.la.lane_mark_type = 0xff;
  msg_pub.la.lane_mark_quality = quality_l;
  msg_pub.la.lane_mark_position = left_pos;
  msg_pub.la.lane_mark_model_a = left_model_a;
  msg_pub.la.lane_mark_width = 0xff;

  msg_pub.ra.lane_mark_type = 0xff;
  msg_pub.ra.lane_mark_quality = quality_r;
  msg_pub.ra.lane_mark_position = right_pos;
  msg_pub.ra.lane_mark_model_a = right_model_a;
  msg_pub.ra.lane_mark_width = 0xff;

  msg_pub.lb.lane_mark_heading_angle = left_heading;
  msg_pub.lb.lane_mark_model_view_range = left_view_range_m;
  msg_pub.lb.lane_mark_model_view_range_availability = left_view_validity;
  msg_pub.lb.lane_mark_model_da = left_model_da;
  msg_pub.lb.lane_start = left_lane_start_m;
  msg_pub.lb.lane_end = left_lane_end_m;

  msg_pub.rb.lane_mark_heading_angle = right_heading;
  msg_pub.rb.lane_mark_model_view_range = right_view_range_m;
  msg_pub.rb.lane_mark_model_view_range_availability = right_view_validity;
  msg_pub.rb.lane_mark_model_da = right_model_da;
  msg_pub.rb.lane_start = right_lane_start_m;
  msg_pub.rb.lane_end = right_lane_end_m;

  msg_pub.add.rolling_counter = rolling_counter;
  msg_pub.add.rh_guardrail = false;
  msg_pub.add.lh_guardrail = false;
  msg_pub.add.right_lane_color_information = 0xff;
  msg_pub.add.left_lane_color_information = 0xff;

  return msg_pub;
}
