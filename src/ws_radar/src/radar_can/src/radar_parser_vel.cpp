#include "radar_parser_vel.h"

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
    last_msg_time_ = this->now();

    start = false;
    tr_objects = 0;

    // parameters (절대속도 변환 on/off, GNSS 토픽)
    absolute_velocity_ = this->declare_parameter<bool>("absolute_velocity", true);
    pvt_topic_ = this->declare_parameter<std::string>("pvt_topic", "/base/ubx_nav_pvt");

    // publisher
    pub_radar_tr_ = this->create_publisher<fo_msgs::msg::RadarTrArray>("/radar_tr", QOS_QUEUE);
    publish_timer_ = this->create_wall_timer(
        // 20ms, std::bind(&RadarParser::publish_latest_tr_array, this));  // 50hz
        50ms, std::bind(&RadarParser::publish_latest_tr_array, this));  // 20hz

    // subscriber: GNSS 자차 속도 (상대->절대 속도 보정)
    if (absolute_velocity_) {
        sub_pvt_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavPVT>(
            pvt_topic_, QOS_QUEUE,
            std::bind(&RadarParser::pvt_callback, this, std::placeholders::_1));
        // RCLCPP_INFO(this->get_logger(),
        //     "absolute velocity ON, ego speed from: %s", pvt_topic_.c_str());
    } else {
        // RCLCPP_INFO(this->get_logger(), "absolute velocity OFF (relative velocity 출력)");
    }

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
                // 50 Hz 타이머가 사용할 가장 최근의 완성된 스캔을 보관한다.
                latest_tr_array_ = tr_array;
                has_latest_tr_array_ = true;
                //std::cout << "[turn_64] tr_objects: " << tr_objects << std::endl;
                // 다음 0x500 프레임에서 새 스캔이 시작될 때까지 현재 스캔을 닫는다.
                start = false;
            }
        }
    }
}

void RadarParser::publish_latest_tr_array() {
    fo_msgs::msg::RadarTrArray latest;

    {
        std::lock_guard<std::mutex> lock(tr_mutex);
        if (!has_latest_tr_array_) return;
        latest = latest_tr_array_;
    }

    pub_radar_tr_->publish(latest);
}

void RadarParser::pvt_callback(const ublox_ubx_msgs::msg::UBXNavPVT::SharedPtr msg) {
    // 자차 지면속도[m/s] 계산.
    //  1) 수신기가 도플러 속도를 채워주면(g_speed/vel_n/vel_e != 0) 그대로 사용(정확·저지연).
    //  2) 속도 필드가 0이면(현재 드라이버가 속도 미출력) 위치(lat/lon)를 차분해 추정.
    static constexpr double DEG2RAD = M_PI / 180.0;
    static constexpr double R_EARTH = 6378137.0;  // WGS84 장반경[m]

    const double lat = msg->lat * 1e-7 * DEG2RAD;
    const double lon = msg->lon * 1e-7 * DEG2RAD;
    const double itow = msg->itow * 1e-3;  // [s]

    // 위치 차분 속도(항상 상태 유지)
    double diff_speed = -1.0;
    if (have_prev_pvt_) {
        const double dt = itow - prev_itow_;
        if (dt > 0.0 && dt < 2.0) {  // 주 롤오버/중복 프레임 방지
            const double dn = (lat - prev_lat_) * R_EARTH;
            const double de = (lon - prev_lon_) * R_EARTH * std::cos(lat);
            diff_speed = std::hypot(dn, de) / dt;
        }
    }
    prev_lat_ = lat;
    prev_lon_ = lon;
    prev_itow_ = itow;
    have_prev_pvt_ = true;

    double speed;
    const bool has_doppler = (msg->g_speed != 0 || msg->vel_n != 0 || msg->vel_e != 0);
    if (has_doppler) {
        speed = msg->g_speed * 1e-3;  // 2D 지면속도[m/s]
    } else if (diff_speed >= 0.0) {
        speed = 0.5 * ego_speed_.load() + 0.5 * diff_speed;  // 차분 노이즈 저감(1차 LPF)
    } else {
        return;  // 첫 프레임 등 아직 속도 없음
    }

    ego_speed_.store(static_cast<float>(speed));
    ego_speed_stamp_ns_.store(this->now().nanoseconds());

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "ego_speed=%.2f m/s (%.1f km/h) [%s]",
        speed, speed * 3.6, has_doppler ? "doppler" : "pos-diff");
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
    tr.track_valid    = 1;
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
    float range_rate = signed_value(raw_range_rate, 14) * 0.01f;  // [m/s] radial(상대속도, signed)

    // 자차 이동 보정: 상대 range-rate -> 절대 range-rate.
    //   레이더는 시선방향(radial) 성분만 측정하므로, 자차 속도의 시선방향 투영
    //   (ego_speed * cos(bearing))을 더해 자차 이동분을 상쇄한다.
    //   정지물체는 abs_rate≈0, 이동물체는 실제 절대속도가 남는다.
    //   (bearing = rad, 전방 = +x. GNSS 신선(<0.5s)할 때만 보정)
    if (absolute_velocity_) {
        const int64_t stamp = ego_speed_stamp_ns_.load();
        if (stamp > 0 && (this->now().nanoseconds() - stamp) < 500000000LL) {
            range_rate += ego_speed_.load() * std::cos(rad);
        }
    }

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
