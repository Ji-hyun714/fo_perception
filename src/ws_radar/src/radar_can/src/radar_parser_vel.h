#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "fo_msgs/msg/radar_tr.hpp"
#include "fo_msgs/msg/radar_tr_array.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_pvt.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <cstdint>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/sockios.h>
#include <cmath>

#define DELAY_THRESHOLD_MS 60
#define LOST_THRESHOLD_MS 100
#define QOS_QUEUE 10
#define TIMER_PERIOD 20ms
// 커널 CAN 수신 버퍼 크기(byte). 파워 차단 시 밀린 프레임이 오래 쌓여
// fail 감지가 지연되지 않도록 기본값보다 작게 설정한다.
// (정상 동작 중 프레임 유실이 보이면 값을 키운다.)
#define CAN_RCVBUF_BYTES 65536

class RadarParser : public rclcpp::Node
{
public:
    RadarParser();
    virtual ~RadarParser() override;

    bool init();

private:
    bool open_can_socket();
    void general_callback();
    void publish_latest_tr_array();
    void recv_loop();
    fo_msgs::msg::RadarTr tr_parser(const struct can_frame &frame);
    // /base/ubx_nav_pvt 로부터 자차(ego) 지면 속도를 갱신한다.
    void pvt_callback(const ublox_ubx_msgs::msg::UBXNavPVT::SharedPtr msg);
    
    // thread
    std::thread recv_thread_;
    std::atomic<bool> running_{false};
    std::mutex tr_mutex;

    // ROS2 parameter
    fo_msgs::msg::RadarTrArray tr_array;
    fo_msgs::msg::RadarTrArray latest_tr_array_;
    bool has_latest_tr_array_{false};

    // publisher
    rclcpp::Publisher<fo_msgs::msg::RadarTrArray>::SharedPtr pub_radar_tr_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_radar_general_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr timer_general_;

    // subscriber (GNSS)
    rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavPVT>::SharedPtr sub_pvt_;

    // ego-motion (GNSS /base/ubx_nav_pvt) : 상대속도 -> 절대속도 보정용
    bool absolute_velocity_{true};                 // true면 절대속도로 변환
    std::string pvt_topic_;                        // 구독 토픽명
    std::atomic<float> ego_speed_{0.0f};           // [m/s] 자차 전방 지면속도
    std::atomic<int64_t> ego_speed_stamp_ns_{0};   // 마지막 갱신 시각(ns), 신선도 판단용
    // 위치 차분용 상태(PVT 콜백 스레드에서만 접근)
    bool have_prev_pvt_{false};
    double prev_lat_{0.0}, prev_lon_{0.0}, prev_itow_{0.0};

    // variable
    std::string can_channel_;
    rclcpp::Time last_msg_time_;
    uint8_t failure_state_;

    bool start;
    int tr_objects;

    int socket_ = -1;

    struct ifreq ifr;
    struct sockaddr_can addr;
    struct can_frame frame;
};
