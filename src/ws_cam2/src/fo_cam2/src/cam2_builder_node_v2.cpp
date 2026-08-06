/* CPP */
#include <chrono>
#include <mutex>
#include <string>
#include <utility>

/* ROS2 */
#include "rclcpp/rclcpp.hpp"
#include "fo_msgs/msg/cam2_data.hpp"
#include "fo_msgs/msg/cam2_ld.hpp"
#include "fo_msgs/msg/cam2_od.hpp"
#include "fo_msgs/msg/cam2_td.hpp"
#include "fo_msgs/msg/cam2_tl.hpp"

class Cam2DataBuilderV2 : public rclcpp::Node
{
public:
  Cam2DataBuilderV2()
  : Node("cam2_builder_v2")
  {
    RCLCPP_INFO(this->get_logger(), "Cam2DataBuilderV2 init");

    this->declare_parameter<std::string>("input_topic_ld", "/cam2_ld");
    this->declare_parameter<std::string>("input_topic_td", "/cam2_td");
    this->declare_parameter<std::string>("input_topic_tl", "/cam2_tl");
    this->declare_parameter<std::string>("input_topic_od", "/cam2_od");
    this->declare_parameter<std::string>("output_topic", "/cam2_data");
    this->declare_parameter<int>("publish_period_ms", 20);

    this->declare_parameter<int>("stale_timeout_ld_ms", 200);
    this->declare_parameter<int>("stale_timeout_td_ms", 200);
    this->declare_parameter<int>("stale_timeout_tl_ms", 200);
    this->declare_parameter<int>("stale_timeout_od_ms", 200);

    this->declare_parameter<bool>("reset_on_stale_ld", false);
    this->declare_parameter<bool>("reset_on_stale_td", false);
    this->declare_parameter<bool>("reset_on_stale_tl", false);
    this->declare_parameter<bool>("reset_on_stale_od", false);

    this->declare_parameter<bool>("log_stale", true);
    this->declare_parameter<int>("log_rate_limit_ms", 2000);

    const auto input_topic_ld = this->get_parameter("input_topic_ld").as_string();
    const auto input_topic_td = this->get_parameter("input_topic_td").as_string();
    const auto input_topic_tl = this->get_parameter("input_topic_tl").as_string();
    const auto input_topic_od = this->get_parameter("input_topic_od").as_string();
    const auto output_topic = this->get_parameter("output_topic").as_string();
    const auto publish_period_ms = static_cast<int>(this->get_parameter("publish_period_ms").as_int());

    stale_timeout_ld_ms_ = static_cast<int>(this->get_parameter("stale_timeout_ld_ms").as_int());
    stale_timeout_td_ms_ = static_cast<int>(this->get_parameter("stale_timeout_td_ms").as_int());
    stale_timeout_tl_ms_ = static_cast<int>(this->get_parameter("stale_timeout_tl_ms").as_int());
    stale_timeout_od_ms_ = static_cast<int>(this->get_parameter("stale_timeout_od_ms").as_int());

    reset_on_stale_ld_ = this->get_parameter("reset_on_stale_ld").as_bool();
    reset_on_stale_td_ = this->get_parameter("reset_on_stale_td").as_bool();
    reset_on_stale_tl_ = this->get_parameter("reset_on_stale_tl").as_bool();
    reset_on_stale_od_ = this->get_parameter("reset_on_stale_od").as_bool();

    log_stale_ = this->get_parameter("log_stale").as_bool();
    log_rate_limit_ms_ = static_cast<int>(this->get_parameter("log_rate_limit_ms").as_int());

    callback_group_ld_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_td_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_tl_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_od_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_compose_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_opts_ld;
    sub_opts_ld.callback_group = callback_group_ld_;
    rclcpp::SubscriptionOptions sub_opts_td;
    sub_opts_td.callback_group = callback_group_td_;
    rclcpp::SubscriptionOptions sub_opts_tl;
    sub_opts_tl.callback_group = callback_group_tl_;
    rclcpp::SubscriptionOptions sub_opts_od;
    sub_opts_od.callback_group = callback_group_od_;

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1));

    sub_ld_ = this->create_subscription<fo_msgs::msg::Cam2LD>(
      input_topic_ld, qos,
      std::bind(&Cam2DataBuilderV2::callbackCam2LD, this, std::placeholders::_1),
      sub_opts_ld);
    sub_td_ = this->create_subscription<fo_msgs::msg::Cam2TD>(
      input_topic_td, qos,
      std::bind(&Cam2DataBuilderV2::callbackCam2TD, this, std::placeholders::_1),
      sub_opts_td);
    sub_tl_ = this->create_subscription<fo_msgs::msg::Cam2TL>(
      input_topic_tl, qos,
      std::bind(&Cam2DataBuilderV2::callbackCam2TL, this, std::placeholders::_1),
      sub_opts_tl);
    sub_od_ = this->create_subscription<fo_msgs::msg::Cam2OD>(
      input_topic_od, qos,
      std::bind(&Cam2DataBuilderV2::callbackCam2OD, this, std::placeholders::_1),
      sub_opts_od);

    pub_cam2_ = this->create_publisher<fo_msgs::msg::Cam2Data>(output_topic, qos);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      std::bind(&Cam2DataBuilderV2::callbackComposeAndPublish, this),
      callback_group_compose_);

    RCLCPP_INFO(
      this->get_logger(),
      "Cam2DataBuilderV2 started. ld=%s td=%s tl=%s od=%s out=%s period=%dms",
      input_topic_ld.c_str(), input_topic_td.c_str(), input_topic_tl.c_str(),
      input_topic_od.c_str(), output_topic.c_str(), publish_period_ms);
    RCLCPP_INFO(
      this->get_logger(),
      "stale timeout ms: ld=%d td=%d tl=%d od=%d",
      stale_timeout_ld_ms_, stale_timeout_td_ms_, stale_timeout_tl_ms_, stale_timeout_od_ms_);
    RCLCPP_INFO(
      this->get_logger(),
      "reset_on_stale: ld=%s td=%s tl=%s od=%s",
      reset_on_stale_ld_ ? "true" : "false",
      reset_on_stale_td_ ? "true" : "false",
      reset_on_stale_tl_ ? "true" : "false",
      reset_on_stale_od_ ? "true" : "false");
  }

private:
  template<typename MsgT>
  struct LatestSlot
  {
    // Shared latest-value state written by one input callback and read by the composer.
    MsgT msg{};
    std::chrono::steady_clock::time_point last_rx_time{};
    bool received{false};
  };

  template<typename MsgT>
  void updateSlot(LatestSlot<MsgT> & slot, std::mutex & mtx, const MsgT & msg)
  {
    std::lock_guard<std::mutex> lock(mtx);
    slot.msg = msg;
    slot.last_rx_time = std::chrono::steady_clock::now();
    slot.received = true;
  }

  template<typename MsgT>
  LatestSlot<MsgT> snapshotSlot(const LatestSlot<MsgT> & slot, std::mutex & mtx)
  {
    std::lock_guard<std::mutex> lock(mtx);
    return LatestSlot<MsgT>{slot.msg, slot.last_rx_time, slot.received};
  }

  bool isStale(
    const std::chrono::steady_clock::time_point & now,
    const std::chrono::steady_clock::time_point & last_rx_time,
    bool received,
    int timeout_ms) const
  {
    if (!received) {
      return true;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx_time).count() > timeout_ms;
  }

  void logStaleIfNeeded(
    const char * field_name,
    bool stale_now,
    bool & stale_prev,
    std::chrono::steady_clock::time_point & last_log_time)
  {
    if (!log_stale_) {
      stale_prev = stale_now;
      return;
    }

    const auto now = std::chrono::steady_clock::now();

    if (stale_now && !stale_prev) {
      RCLCPP_WARN(this->get_logger(), "%s input became stale.", field_name);
      last_log_time = now;
    } else if (!stale_now && stale_prev) {
      RCLCPP_INFO(this->get_logger(), "%s input recovered.", field_name);
      last_log_time = now;
    } else if (stale_now &&
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() >= log_rate_limit_ms_)
    {
      RCLCPP_WARN(this->get_logger(), "%s input is stale.", field_name);
      last_log_time = now;
    }

    stale_prev = stale_now;
  }

  void callbackCam2LD(const fo_msgs::msg::Cam2LD::SharedPtr msg)
  {
    updateSlot(latest_ld_, mtx_ld_, *msg);
  }

  void callbackCam2TD(const fo_msgs::msg::Cam2TD::SharedPtr msg)
  {
    updateSlot(latest_td_, mtx_td_, *msg);
  }

  void callbackCam2TL(const fo_msgs::msg::Cam2TL::SharedPtr msg)
  {
    updateSlot(latest_tl_, mtx_tl_, *msg);
  }

  void callbackCam2OD(const fo_msgs::msg::Cam2OD::SharedPtr msg)
  {
    updateSlot(latest_od_, mtx_od_, *msg);
  }

  void callbackComposeAndPublish()
  {
    const auto now = std::chrono::steady_clock::now();

    // Local copies used only within this compose cycle.
    const auto ld_latest = snapshotSlot(latest_ld_, mtx_ld_);
    const auto td_latest = snapshotSlot(latest_td_, mtx_td_);
    const auto tl_latest = snapshotSlot(latest_tl_, mtx_tl_);
    const auto od_latest = snapshotSlot(latest_od_, mtx_od_);

    const bool ld_stale = isStale(now, ld_latest.last_rx_time, ld_latest.received, stale_timeout_ld_ms_);
    const bool td_stale = isStale(now, td_latest.last_rx_time, td_latest.received, stale_timeout_td_ms_);
    const bool tl_stale = isStale(now, tl_latest.last_rx_time, tl_latest.received, stale_timeout_tl_ms_);
    const bool od_stale = isStale(now, od_latest.last_rx_time, od_latest.received, stale_timeout_od_ms_);

    logStaleIfNeeded("Cam2LD", ld_stale, ld_stale_prev_, ld_last_log_time_);
    logStaleIfNeeded("Cam2TD", td_stale, td_stale_prev_, td_last_log_time_);
    logStaleIfNeeded("Cam2TL", tl_stale, tl_stale_prev_, tl_last_log_time_);
    logStaleIfNeeded("Cam2OD", od_stale, od_stale_prev_, od_last_log_time_);

    fo_msgs::msg::Cam2Data out{};

    if (ld_latest.received && !(ld_stale && reset_on_stale_ld_)) {
      out.ld = ld_latest.msg;
    }
    if (td_latest.received && !(td_stale && reset_on_stale_td_)) {
      out.td = td_latest.msg;
    }
    if (tl_latest.received && !(tl_stale && reset_on_stale_tl_)) {
      out.tl = tl_latest.msg;
    }
    if (od_latest.received && !(od_stale && reset_on_stale_od_)) {
      out.od = od_latest.msg;
    }

    pub_cam2_->publish(out);
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_ld_;
  rclcpp::CallbackGroup::SharedPtr callback_group_td_;
  rclcpp::CallbackGroup::SharedPtr callback_group_tl_;
  rclcpp::CallbackGroup::SharedPtr callback_group_od_;
  rclcpp::CallbackGroup::SharedPtr callback_group_compose_;

  rclcpp::Subscription<fo_msgs::msg::Cam2LD>::SharedPtr sub_ld_;
  rclcpp::Subscription<fo_msgs::msg::Cam2TD>::SharedPtr sub_td_;
  rclcpp::Subscription<fo_msgs::msg::Cam2TL>::SharedPtr sub_tl_;
  rclcpp::Subscription<fo_msgs::msg::Cam2OD>::SharedPtr sub_od_;
  rclcpp::Publisher<fo_msgs::msg::Cam2Data>::SharedPtr pub_cam2_;
  rclcpp::TimerBase::SharedPtr timer_;

  LatestSlot<fo_msgs::msg::Cam2LD> latest_ld_;
  LatestSlot<fo_msgs::msg::Cam2TD> latest_td_;
  LatestSlot<fo_msgs::msg::Cam2TL> latest_tl_;
  LatestSlot<fo_msgs::msg::Cam2OD> latest_od_;

  std::mutex mtx_ld_;
  std::mutex mtx_td_;
  std::mutex mtx_tl_;
  std::mutex mtx_od_;

  int stale_timeout_ld_ms_{200};
  int stale_timeout_td_ms_{200};
  int stale_timeout_tl_ms_{200};
  int stale_timeout_od_ms_{200};

  bool reset_on_stale_ld_{false};
  bool reset_on_stale_td_{false};
  bool reset_on_stale_tl_{false};
  bool reset_on_stale_od_{false};

  bool log_stale_{true};
  int log_rate_limit_ms_{2000};

  bool ld_stale_prev_{true};
  bool td_stale_prev_{true};
  bool tl_stale_prev_{true};
  bool od_stale_prev_{true};

  std::chrono::steady_clock::time_point ld_last_log_time_{};
  std::chrono::steady_clock::time_point td_last_log_time_{};
  std::chrono::steady_clock::time_point tl_last_log_time_{};
  std::chrono::steady_clock::time_point od_last_log_time_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Cam2DataBuilderV2>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
