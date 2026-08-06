#include "radar_general_node.h"

using namespace std::chrono_literals;

RadarGeneral::RadarGeneral()
: Node("radar_general_node")
{
    RCLCPP_INFO(this->get_logger(), "RadarGeneral Node started.");
}

RadarGeneral::~RadarGeneral() {
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
    close(socket_);
}

bool RadarGeneral::init() {
    can_channel_ = "can0";
    delay_threshold_ = DELAY_THRESHOLD_MS / 1000.0;
    lost_threshold_  = LOST_THRESHOLD_MS  / 1000.0;

    last_msg_time_ = this->now();

    fail_flag_ = 0;
    alive_counter_ = 0;
    is_disconnected = false;

    // publisher
    pub_radar_general_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/radar_general", QOS_QUEUE);

    // CAN socket
    if (!open_can_socket()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket");
        return false;
    } else {
        RCLCPP_INFO(this->get_logger(), "CAN listening on: %s", can_channel_.c_str());
    }

    // radar_general timer
    timer_general = this->create_wall_timer(TIMER_PERIOD, std::bind(&RadarGeneral::general_callback, this));

    // CAN 수신 전용 스레드
    running_ = true;
    recv_thread_ = std::thread(&RadarGeneral::recv_loop, this);

    return true;
}


bool RadarGeneral::open_can_socket() {
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
        perror("Failed to create socket");
        return false;
    }

    // selects can interface (can0 / can1 / ...)
    std::strncpy(ifr.ifr_name, can_channel_.c_str(), IFNAMSIZ-1);
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) return false;

    // socket binding
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        return false;

    return true;
}


void RadarGeneral::recv_loop() {
    while (running_) {
        int nbytes = recv(socket_, &frame, sizeof(struct can_frame), 0);  // 0: 수신 버퍼에 있는 데이터를 읽고 버퍼에서 제거

        if (!running_) break;  // 종료 요청 시 break

        if (nbytes == sizeof(struct can_frame)) {
            last_msg_time_ = this->now();

            radar_process(frame);
        }
    }
}


void RadarGeneral::general_callback() {
    auto now = this->now();
    double elapsed_sec = (now - last_msg_time_).seconds();

    // 정상
    if (elapsed_sec > 0 && elapsed_sec < delay_threshold_) {
        fail_flag_ = 0;
        failure_state_ = 0;
        is_disconnected = false;
    }
    
    // 지연
    else if (elapsed_sec >= delay_threshold_ && elapsed_sec < lost_threshold_) {
        fail_flag_ = 1;
        failure_state_ = 1;
        is_disconnected = false;
    }

    // 단절
    else if (elapsed_sec >= lost_threshold_) {
        fail_flag_ = 1;
        failure_state_ = 0xf;

        if (is_disconnected == false) {
            is_disconnected = true;
        }
    }

    std_msgs::msg::UInt8MultiArray msg;
    msg.data = {fail_flag_, alive_counter_, failure_state_};
    pub_radar_general_->publish(msg);

    if (elapsed_sec > 0 && elapsed_sec < lost_threshold_) {
        alive_counter_ = (alive_counter_ + 1) % 0x80;   // 0x80 == 128
    }
}

void RadarGeneral::radar_process(const struct can_frame &frame) {
    last_msg_time_ = this->now();
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RadarGeneral>();
    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize RadarGeneral");
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}