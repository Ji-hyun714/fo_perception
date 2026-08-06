#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
// #include "fo_msgs/msg/radar_tr.hpp"
// #include "fo_msgs/msg/radar_tr_array.hpp"
#include "fo_msgs/msg/can_frame.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>
#include <sys/ioctl.h>

#define DELAY_THRESHOLD_MS 60
#define LOST_THRESHOLD_MS 100
#define QOS_QUEUE 10
#define TIMER_PERIOD 20ms

class RadarBagNode : public rclcpp::Node
{
public:
    RadarBagNode();
    virtual ~RadarBagNode() override;

    bool init();

private:
    bool open_can_socket();
    void general_callback();
    void radar_bag(const struct can_frame &frame);
    void recv_loop();

    // thread
    std::thread recv_thread_;
    std::atomic<bool> running_{false};

    // ROS2 parameter
    std::string can_channel_;
    double delay_threshold_;
    double lost_threshold_;

    // publisher
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_radar_general_;
    rclcpp::Publisher<fo_msgs::msg::CanFrame>::SharedPtr pub_can_frame_;

    // timer
    rclcpp::TimerBase::SharedPtr timer_general;
    rclcpp::TimerBase::SharedPtr timer_data;

    // variable
    rclcpp::Time last_msg_time_;
    uint8_t fail_flag_;
    uint8_t alive_counter_;
    uint8_t failure_state_;
    uint8_t rolling_count_;
    bool is_disconnected;

    int delay_ms, lost_ms;
    int socket_ = -1;

    struct ifreq ifr;
    struct sockaddr_can addr;
    struct can_frame frame;
};
