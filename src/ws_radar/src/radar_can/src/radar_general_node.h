#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <thread>
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
#define TIMER_PERIOD 50ms

class RadarGeneral : public rclcpp::Node
{
public:
    RadarGeneral();
    virtual ~RadarGeneral() override;

    bool init();

private:
    bool open_can_socket();
    void general_callback();
    void radar_process(const struct can_frame &frame);
    void recv_loop();

    // ROS2 parameter
    std::string can_channel_;
    double delay_threshold_;
    double lost_threshold_;

    // publisher
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_radar_general_;

    // timer
    rclcpp::TimerBase::SharedPtr timer_general;

    // thread
    std::thread recv_thread_;
    std::atomic<bool> running_{false};

    // variable
    rclcpp::Time last_msg_time_;
    uint8_t fail_flag_;
    uint8_t alive_counter_;
    uint8_t failure_state_;
    bool is_disconnected;

    int socket_ = -1;

    struct ifreq ifr;
    struct sockaddr_can addr;
    struct can_frame frame;
};