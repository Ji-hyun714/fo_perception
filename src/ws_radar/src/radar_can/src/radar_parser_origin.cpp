#include "radar_parser.h"

using namespace std::chrono_literals;

RadarParser::RadarParser()
: Node("radar_tr_parser_node")
{}

RadarParser::~RadarParser() {
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
    close(socket_);
}

bool RadarParser::init() {
    can_channel_ = "can0";
    delay_threshold_ = DELAY_THRESHOLD_MS / 1000.0;
    lost_threshold_  = LOST_THRESHOLD_MS  / 1000.0;

    last_msg_time_ = this->now();

    fail_flag_ = 0;
    alive_counter_ = 0;
    is_disconnected = false;
    tr_objects = 64;

    // publisher
    pub_radar_tr_ = this->create_publisher<fo_msgs::msg::RadarTrArray>("/radar_tr", QOS_QUEUE);

    // CAN socket
    if (!open_can_socket()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket");
        return false;
    } else {
        RCLCPP_INFO(this->get_logger(), "CAN listening on: %s", can_channel_.c_str());
    }

    // CAN 수신 전용 스레드 시작
    running_ = true;
    recv_thread_ = std::thread(&RadarParser::recv_loop, this);

    return true;
}


bool RadarParser::open_can_socket() {
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


void RadarParser::recv_loop() {
    while (running_) {
        int nbytes = recv(socket_, &frame, sizeof(struct can_frame), 0);  // 0: 수신 버퍼에 있는 데이터를 읽고 버퍼에서 제거

        if (!running_) break;  // 종료 요청 시 break

        if (nbytes == sizeof(struct can_frame)) {
            std::lock_guard<std::mutex> lock(tr_mutex);

            if (frame.can_id == 0x500) {
                //std::cout << "[turn_end] tr_objects: " << tr_objects << std::endl;
                // std::cout << "" << std::endl;
                start = true;
                tr_objects = 0;
                last_msg_time_ = this->now();

                tr_array.tracks.clear();
                tr_array.header.stamp = last_msg_time_;
                tr_array.header.frame_id = "radar2_tr_array";
                fo_msgs::msg::RadarTr tr = tr_parser(frame);
                tr_array.tracks.push_back(tr);
                tr_objects += 1;
            }

            if (!start) continue;

            if (frame.can_id >= 0x501 && frame.can_id <= 0x53F) {
                fo_msgs::msg::RadarTr tr = tr_parser(frame);
                tr_array.tracks.push_back(tr);
                tr_objects += 1;
            }

            if (tr_objects == 32) {
                pub_radar_tr_->publish(tr_array);
                //std::cout << "[turn_64] tr_objects: " << tr_objects << std::endl;
                // start = false;
            }
        }
    }
}

static int signed_value(int raw, int n_bit) {
    if (raw > ((1 << (n_bit - 1)) - 1)) {
        raw -= (1 << n_bit);
    }
    return raw;
}

fo_msgs::msg::RadarTr RadarParser::tr_parser(const struct can_frame &frame) {
    int id = frame.can_id;
    // std::cout << "canid: " << id << std::endl;
    // int id = frame.can_id & CAN_SFF_MASK;

    fo_msgs::msg::RadarTr tr;
    tr.track_id       = id - 0x500 + 1;  // 1~64
    tr.track_status   = (frame.data[1] >> 5) & 0x07;
    tr.track_valid    = 0;
    tr.track_type     = 0;
    tr.length         = 1.0f;

    // pos [m]
    int raw_range = ((frame.data[2] & 0x07) << 8) | frame.data[3];
    int raw_angle = ((frame.data[1] & 0x1f) << 5) | ((frame.data[2] >> 3) & 0x1f);

    float deg_angle = signed_value(raw_angle, 10) * 0.1f;  // [degree](signed)
    float rad = -1.0f * deg_angle * M_PI / 180.0f;

    float pos_x = raw_range * std::cos(rad) * 0.1f;
    float pos_y = raw_range * std::sin(rad) * 0.1f;
    tr.track_pos_x       = pos_x;
    tr.track_pos_y       = pos_y;

    // vel [m/s]
    int raw_range_rate = ((frame.data[6] & 0x3F) << 8) | frame.data[7];
    float range_rate = signed_value(raw_range_rate, 14) * 0.01f;  // [m/s](signed)

    float vel_x = range_rate * std::cos(rad);
    float vel_y = range_rate * std::sin(rad);
    tr.track_vel_x       = vel_x;
    tr.track_vel_y       = vel_y;

    // width [m]
    int raw_width     = (frame.data[4] >> 2) & 0x0f;
    tr.width          = raw_width * 0.5f;

    // track rolling count
    tr.rolling_count  = (frame.data[4] >> 6) & 0x01;
    
    // std::cout << tr_objects << " | can_id: 0x" << std::hex << id << std::dec
    //           << " / posx: " << pos_x << " / posy: " << pos_y
    //           << " / velx: " << vel_x << " / vely: " << vel_y
    //           << " / width: " << tr.width << " / rc: " << tr.rolling_count << std::endl;
    return tr;
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RadarParser>();
    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize RadarParser");
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}