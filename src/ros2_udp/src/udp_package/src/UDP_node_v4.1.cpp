#include <iostream>
#include <cstring>
#include <mutex>
#include <bitset>
#include <algorithm>
#include <fstream>
#include <cmath>

#include "UDP_node_v4.1.h"

#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

// bool test_mode = true;   // test mode
bool test_mode = false;  // sensor mode

// buffer
UDP_DATA buffer_[2];   // UDP_DATA 타입의 배열 2개 선언 (buffer_[0], buffer_[1])
std::atomic<int> write_idx_{0};
std::mutex buffer_mutex;


MsgSubscriber::MsgSubscriber()
: Node("udp_node")
{
    RCLCPP_INFO(this->get_logger(), "MsgSubscriber Node started.");
}


MsgSubscriber::~MsgSubscriber(){
    // 필요한 자원 정리
    close(server_socket);
}

bool MsgSubscriber::init(){
    for (int i = 0; i < 2; i++) memset(&buffer_[i], 0, sizeof(UDP_DATA));    // 버퍼 초기화

    // socket 생성
    server_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (server_socket < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }

    // 서버 주소 정보 설정
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

    // Cam2_OD 정적 객체 후보 좌표(EPSG:4326) 로드
    loadStaticObjectGpsPoints("/home/wise/fo_perception/drum.txt");

    if (!test_mode) {  
        // Subscribe General
        lidar_general_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/lidar2_general", 10,
            std::bind(&MsgSubscriber::callback_lidar_general, this, std::placeholders::_1));
        
        radar_general_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/radar_general", 10,
            std::bind(&MsgSubscriber::callback_radar_general, this, std::placeholders::_1));

        camera_general_sub_ = this->create_subscription<fo_msgs::msg::StatusCamera>(
            "/camera_general", 10,
            std::bind(&MsgSubscriber::callback_camera_general, this, std::placeholders::_1));

        // Subscribe lidar_delay
        lidar_delay_fl_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/lidar1_delay_counts", 10,
            std::bind(&MsgSubscriber::callback_lidar1_delay, this, std::placeholders::_1));

        lidar_delay_fr_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/lidar2_delay_counts", 10,
            std::bind(&MsgSubscriber::callback_lidar2_delay, this, std::placeholders::_1));

        lidar_delay_r_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/lidar3_delay_counts", 10,
            std::bind(&MsgSubscriber::callback_lidar3_delay, this, std::placeholders::_1));

        // Subscribe lidar_td
        lidar_obj_sub_ = this->create_subscription<fo_msgs::msg::LidarobjListsForSF2>(
            "/lidar/sf_objs", 10,
            std::bind(&MsgSubscriber::callback_lidar_td, this, std::placeholders::_1));

        // Subscribe radar_tr
        radar_tr_sub_ = this->create_subscription<fo_msgs::msg::RadarTrArray>(
            "/radar_tr", 10,
            std::bind(&MsgSubscriber::callback_radar_tr, this, std::placeholders::_1));

        // Subscribe GNSS
        gnss_pvt_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavPVT>(
            "/base/ubx_nav_pvt", 10,
            std::bind(&MsgSubscriber::callback_gnss_pvt, this, std::placeholders::_1));

        gnss_heading_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavRelPosNED>(
            "/rover/ubx_nav_rel_pos_ned", 10,
            std::bind(&MsgSubscriber::callback_gnss_heading, this, std::placeholders::_1));

        gnss_cal_heading_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/cal_heading", 10,
            std::bind(&MsgSubscriber::callback_gnss_cal_heading, this, std::placeholders::_1));
            
        // Subscribe Cam2_Data
        cam2_sub_ = this->create_subscription<fo_msgs::msg::Cam2Data>(
            "/camera/cam2data", 10,
            std::bind(&MsgSubscriber::callback_cam2, this, std::placeholders::_1));

        // Subscribe SF2
        sf2_sub_ = this->create_subscription<fo_msgs::msg::FusionobjListForSF2>(
            "/fusion/sf_objs", 10,
            std::bind(&MsgSubscriber::callback_sf2, this, std::placeholders::_1));
        }
        
    else {
        loadYAML();
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&MsgSubscriber::timer_callback, this));

    return true;
}

void MsgSubscriber::loadYAML() {
    const std::string yaml_path = "/home/wise/fo_perception/src/ros2_udp/src/udp_package/config/testBuff.yaml";

    try {
        YAML::Node config = YAML::LoadFile(yaml_path);

        // general
        param_camera2_fail_flag_       = config["param_camera2_fail_flag_"].as<int>();
        param_camera2_alive_counter_   = config["param_camera2_alive_counter_"].as<int>();
        param_radar2_c_fail_flag_      = config["param_radar2_c_fail_flag_"].as<int>();
        param_radar2_c_alive_counter_  = config["param_radar2_c_alive_counter_"].as<int>();
        param_radar2_f_fail_flag_      = config["param_radar2_f_fail_flag_"].as<int>();
        param_radar2_f_alive_counter_  = config["param_radar2_f_alive_counter_"].as<int>();
        param_radar2_r_fail_flag_      = config["param_radar2_r_fail_flag_"].as<int>();
        param_radar2_r_alive_counter_  = config["param_radar2_r_alive_counter_"].as<int>();
        param_lidar2_fl_fail_flag_     = config["param_lidar2_fl_fail_flag_"].as<int>();
        param_lidar2_fl_alive_counter_ = config["param_lidar2_fl_alive_counter_"].as<int>();
        param_lidar2_fr_fail_flag_     = config["param_lidar2_fr_fail_flag_"].as<int>();
        param_lidar2_fr_alive_counter_ = config["param_lidar2_fr_alive_counter_"].as<int>();
        param_lidar2_r_fail_flag_      = config["param_lidar2_r_fail_flag_"].as<int>();
        param_lidar2_r_alive_counter_  = config["param_lidar2_r_alive_counter_"].as<int>();
        param_gnss_fail_flag_          = config["param_gnss_fail_flag_"].as<int>();
        param_gnss_alive_counter_      = config["param_gnss_alive_counter_"].as<int>();
        param_v2x_fail_flag_           = config["param_v2x_fail_flag_"].as<int>();
        param_v2x_alive_counter_       = config["param_v2x_alive_counter_"].as<int>();

        param_camera2_failure_state_   = config["param_camera2_failure_state_"].as<int>();
        param_radar2_c_failure_state_  = config["param_radar2_c_failure_state_"].as<int>();
        param_radar2_f_failure_state_  = config["param_radar2_f_failure_state_"].as<int>();
        param_radar2_r_failure_state_  = config["param_radar2_r_failure_state_"].as<int>();
        param_lidar2_fl_failure_state_ = config["param_lidar2_fl_failure_state_"].as<int>();
        param_lidar2_fr_failure_state_ = config["param_lidar2_fr_failure_state_"].as<int>();
        param_lidar2_r_failure_state_  = config["param_lidar2_r_failure_state_"].as<int>();
        param_gnss_failure_state_      = config["param_gnss_failure_state_"].as<int>();
        param_v2x_failure_state_       = config["param_v2x_failure_state_"].as<int>();

        param_sense_staute_rolling_counter_ = config["param_sense_staute_rolling_counter_"].as<int>();
        param_SW_VehiCommS_                 = config["param_SW_VehiCommS_"].as<int>();
        param_SW_SensProc_                  = config["param_SW_SensProc_"].as<int>();
        param_SW_SensFusion_                = config["param_SW_SensFusion_"].as<int>();
        param_SW_ADSMonitorS_               = config["param_SW_ADSMonitorS_"].as<int>();
        param_HW_ProcSense_                 = config["param_HW_ProcSense_"].as<int>();
        param_HW_ExtNPU_                    = config["param_HW_ExtNPU_"].as<int>();
        param_HW_PMICS_                     = config["param_HW_PMICS_"].as<int>();
        param_HW_EthPhySen_                 = config["param_HW_EthPhySen_"].as<int>();
        param_HW_EthPhyCam_                 = config["param_HW_EthPhyCam_"].as<int>();
        param_HW_CCANS_                     = config["param_HW_CCANS_"].as<int>();
        param_HW_IntCANS_                   = config["param_HW_IntCANS_"].as<int>();
        param_HW_DDR_                       = config["param_HW_DDR_"].as<int>();
        param_HW_eMMC_                      = config["param_HW_eMMC_"].as<int>();
        param_VEH_Chassis_                  = config["param_VEH_Chassis_"].as<int>();
        param_DNN_IMG_                      = config["param_DNN_IMG_"].as<int>();
        param_DNN_PCD_                      = config["param_DNN_PCD_"].as<int>();

        // camera ld
        param_lane_mark_type_               = config["param_lane_mark_type_"].as<int>();
        param_lane_mark_quality_            = config["param_lane_mark_quality_"].as<int>();
        param_lane_mark_position_           = config["param_lane_mark_position_"].as<double>();
        param_lane_mark_model_a_            = config["param_lane_mark_model_a_"].as<double>();

        param_lane_mark_heading_angle_      = config["param_lane_mark_heading_angle_"].as<double>();
        param_lane_mark_model_view_range_   = config["param_lane_mark_model_view_range_"].as<double>();
        param_lane_mark_model_view_range_availability_ = config["param_lane_mark_model_view_range_availability_"].as<int>();
        param_lane_mark_model_da_           = config["param_lane_mark_model_da_"].as<double>();

        param_alive_counter_ld_             = config["param_alive_counter_ld_"].as<int>();

        // camera td
        param_alive_counter_td_a_           = config["param_alive_counter_td_a_"].as<int>();
        param_object_age_                   = config["param_object_age_"].as<int>();
        param_angle_rate_                   = config["param_angle_rate_"].as<double>();
        param_angle_left_                   = config["param_angle_left_"].as<double>();
        param_angle_right_                  = config["param_angle_right_"].as<double>();
        param_motion_status_                = config["param_motion_status_"].as<int>();
        param_object_lane_                  = config["param_object_lane_"].as<int>();
        param_cam2_obstacle_brake_lights_   = config["param_cam2_obstacle_brake_lights_"].as<int>();

        param_alive_counter_td_b_           = config["param_alive_counter_td_b_"].as<int>();
        param_range_                        = config["param_range_"].as<double>();
        param_object_vaildity_              = config["param_object_vaildity_"].as<int>();
        param_range_rate_                   = config["param_range_rate_"].as<double>();
        param_cam2_obstacle_physical_width_ = config["param_cam2_obstacle_physical_width_"].as<double>();
        param_cam2_track_id_                = config["param_cam2_track_id_"].as<int>();
        param_object_type_                  = config["param_object_type_"].as<int>();

        // camera tl
        param_traffic_light_data_           = config["param_traffic_light_data_"].as<int>();
        param_traffic_light_valid_flag_     = config["param_traffic_light_valid_flag_"].as<int>();
        param_traffic_light_accuracy_       = config["param_traffic_light_accuracy_"].as<int>();

        // camera od
        param_rolling_count_od_             = config["param_rolling_count_od_"].as<int>();
        param_camera2_static_obj_type_      = config["param_camera2_static_obj_type_"].as<int>();
        param_static_object_status_         = config["param_static_object_status_"].as<int>();
        param_static_object_pos_y_          = config["param_static_object_pos_y_"].as<double>();
        param_static_object_pos_x_          = config["param_static_object_pos_x_"].as<double>();
        param_static_object_pos2_y_         = config["param_static_object_pos2_y_"].as<double>();
        param_static_object_pos2_x_         = config["param_static_object_pos2_x_"].as<double>();

        // lidar td
        param_track_status_             = config["param_track_status_"].as<int>();
        param_track_valid_lidar_        = config["param_track_valid_lidar_"].as<int>();
        param_track_pos_y_              = config["param_track_pos_y_"].as<double>();
        param_track_pos_x_              = config["param_track_pos_x_"].as<double>();
        param_track_rolling_count_a_    = config["param_track_rolling_count_a_"].as<int>();

        param_lidar_track_id_           = config["param_lidar_track_id_"].as<int>();
        param_track_add_info_status_    = config["param_track_add_info_status_"].as<int>();
        param_track_width_              = config["param_track_width_"].as<double>();
        param_track_length_             = config["param_track_length_"].as<double>();
        param_track_rolling_count_b_    = config["param_track_rolling_count_b_"].as<int>();

        // radar
        param_radar2_track_id_          = config["param_radar2_track_id_"].as<int>();
        param_radar2_track_status_      = config["param_radar2_track_status_"].as<int>();
        param_radar2_track_pos_y_       = config["param_radar2_track_pos_y_"].as<double>();
        param_radar2_track_pos_x_       = config["param_radar2_track_pos_x_"].as<double>();
        param_track_valid_radar_        = config["param_track_valid_radar_"].as<int>();
        param_radar2_track_type_        = config["param_radar2_track_type_"].as<int>();
        param_radar2_track_vel_y_       = config["param_radar2_track_vel_y_"].as<double>();
        param_radar2_rolling_count_     = config["param_radar2_rolling_count_"].as<int>();
        param_radar2_track_vel_x_       = config["param_radar2_track_vel_x_"].as<double>();

        // sf2
        param_track_valid_sf2_          = config["param_track_valid_sf2_"].as<int>();
        param_sf2_track_pos_y_          = config["param_sf2_track_pos_y_"].as<double>();
        param_sf2_track_pos_x_          = config["param_sf2_track_pos_x_"].as<double>();
        param_sf2_rolling_count_        = config["param_sf2_rolling_count_"].as<int>();
        param_sf2_track_type_           = config["param_sf2_track_type_"].as<int>();
        param_sf2_track_vel_y_          = config["param_sf2_track_vel_y_"].as<double>();
        param_sf2_track_vel_x_          = config["param_sf2_track_vel_x_"].as<double>();

        param_sf2_track_id_                 = config["param_sf2_track_id_"].as<int>();

        // gps
        param_rtk_latitude_             = config["param_rtk_latitude_"].as<int>();
        param_rtk_longitude_            = config["param_rtk_longitude_"].as<int>();

        param_rtk_rolling_count_        = config["param_rtk_rolling_count_"].as<int>();
        param_rtk_heading_              = config["param_rtk_heading_"].as<int>();
        param_rtk_fix_flag_             = config["param_rtk_fix_flag_"].as<int>();

        RCLCPP_INFO(this->get_logger(), "Loaded test config from %s", yaml_path.c_str());
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load YAML: %s", e.what());
    }
}

void MsgSubscriber::timer_callback() {
    if (test_mode) {
        testBuff();
        swapBuff();
        sendBuff();
        // std::cout << "buff_size: " << sizeof(UDP_DATA) << std::endl;
    }
    else {
        checkSensorProcState();
        checkGeneralSenseState();
        swapBuff();
        sendBuff();
        // std::cout << "buff_size: " << sizeof(UDP_DATA) << std::endl;
    }
}

// callback_sf2에서 갱신되는 sf2_counter가 20ms timer tick 대비 콜백 주기가 느려 몇 tick 동일하게 유지되는 것은 정상이지만,
// 10 tick(200ms) 이상 값이 그대로면 SF2 수신이 끊긴 것으로 보고 SW_SensFusion을 0xF(fail)로, 값이 다시 변하면 0x0(정상)으로 갱신
void MsgSubscriber::checkGeneralSenseState() {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();

    //

    if (sf2_counter == sf2_prev_counter) {
        sf2_stale_tick_count++;
    } else {
        sf2_stale_tick_count = 0;
    }
    sf2_prev_counter = sf2_counter;

    if (sf2_stale_tick_count >= 10) {
        buffer_[idx].general_sense_state.SW_SensFusion = 0xF;
    } else {
        buffer_[idx].general_sense_state.SW_SensFusion = 0x0;
    }

    buffer_[idx].general_sense_state.DNN_IMG = 0x0;
    buffer_[idx].general_sense_state.DNN_PCD = 0x0;
}

// /lidar2_general(fl/fr/r), /radar_general(c/f/r), /camera_general fail_flag 중 하나라도 1이면
// 센서 처리 실패로 보고 SW_SensProc을 0xF(fail)로, 모두 0이면 0x0(정상)으로 갱신
void MsgSubscriber::checkSensorProcState() {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();

    bool any_fail =
        buffer_[idx].general_sensor2_status.camera2_fail_flag == 1 ||
        buffer_[idx].general_sensor2_status.radar2_c_fail_flag == 1 ||
        buffer_[idx].general_sensor2_status.radar2_f_fail_flag == 1 ||
        buffer_[idx].general_sensor2_status.radar2_r_fail_flag == 1 ||
        buffer_[idx].general_sensor2_status.lidar2_fl_fail_flag == 1 ||
        buffer_[idx].general_sensor2_status.lidar2_fr_fail_flag == 1 ||
        buffer_[idx].general_sensor2_status.lidar2_r_fail_flag == 1;

    buffer_[idx].general_sense_state.SW_SensProc = any_fail ? 0xF : 0x0;
}

void MsgSubscriber::swapBuff() {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int send_idx = (write_idx_ + 1) % 2;

    std::memcpy(&buffer_[send_idx],
                &buffer_[write_idx_],
                sizeof(UDP_DATA));

    write_idx_ = (write_idx_ + 1) % 2;
}

void MsgSubscriber::sendBuff() {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int send_idx = (write_idx_ + 1) % 2;
    if (sendto(server_socket, &buffer_[send_idx], sizeof(UDP_DATA), 0,
            (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        fprintf(stderr, "sendto failed\n");
    }
}

void MsgSubscriber::callback_camera_general(const fo_msgs::msg::StatusCamera::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();

    buffer_[idx].general_sensor2_status.camera2_fail_flag = static_cast<uint8_t>(msg->fail_flag);
    buffer_[idx].general_sensor2_status.camera2_alive_counter = static_cast<uint8_t>(msg->rolling_counter);
    buffer_[idx].general_sensor2_status.camera2_failure_state = static_cast<uint8_t>(msg->failure_state);

    // std::cout << "camera2_fail_flag: " << static_cast<int>(buffer_[idx].general_sensor2_status.camera2_fail_flag) << std::endl;
    // std::cout << "camera2_alive_counter: " << static_cast<int>(buffer_[idx].general_sensor2_status.camera2_alive_counter) << std::endl;
    // std::cout << "camera2_failure_state: " << static_cast<int>(buffer_[idx].general_sensor2_status.camera2_failure_state) << std::endl;
}

//-----------------------------------------------------------------------------------------------
// callback func (lidar)
void MsgSubscriber::callback_lidar_general(const std_msgs::msg::UInt8MultiArray::SharedPtr msg){
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();
    buffer_[idx].general_sensor2_status.lidar2_fl_fail_flag = static_cast<uint8_t>(msg->data[0]);
    buffer_[idx].general_sensor2_status.lidar2_fl_alive_counter = static_cast<uint8_t>(msg->data[1]);
    buffer_[idx].general_sensor2_status.lidar2_fl_failure_state = static_cast<uint8_t>(msg->data[2]);

    buffer_[idx].general_sensor2_status.lidar2_fr_fail_flag = static_cast<uint8_t>(msg->data[3]);
    buffer_[idx].general_sensor2_status.lidar2_fr_alive_counter = static_cast<uint8_t>(msg->data[4]);
    buffer_[idx].general_sensor2_status.lidar2_fr_failure_state = static_cast<uint8_t>(msg->data[5]);

    buffer_[idx].general_sensor2_status.lidar2_r_fail_flag = static_cast<uint8_t>(msg->data[6]);
    buffer_[idx].general_sensor2_status.lidar2_r_alive_counter = static_cast<uint8_t>(msg->data[7]);
    buffer_[idx].general_sensor2_status.lidar2_r_failure_state = static_cast<uint8_t>(msg->data[8]);
}

void MsgSubscriber::callback_lidar1_delay(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();
    // 데이터가 비어 있지 않으면 fail, 비어 있으면 정상
    if (msg->data) {
        buffer_[idx].general_sensor2_status.lidar2_fl_failure_state = static_cast<uint8_t>(1);
    } else {
        buffer_[idx].general_sensor2_status.lidar2_fl_failure_state = static_cast<uint8_t>(0);
    }
}

void MsgSubscriber::callback_lidar2_delay(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();
    if (msg->data) {
        buffer_[idx].general_sensor2_status.lidar2_fr_failure_state = static_cast<uint8_t>(1);
    } else {
        buffer_[idx].general_sensor2_status.lidar2_fr_failure_state = static_cast<uint8_t>(0);
    }
}

void MsgSubscriber::callback_lidar3_delay(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();
    if (msg->data) {
        buffer_[idx].general_sensor2_status.lidar2_r_failure_state = static_cast<uint8_t>(1);
    } else {
        buffer_[idx].general_sensor2_status.lidar2_r_failure_state = static_cast<uint8_t>(0);
    }
}

//-----------------------------------------------------------------------------------------------
// callback func (radar)
void MsgSubscriber::callback_radar_general(const std_msgs::msg::UInt8MultiArray::SharedPtr msg){
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();
    buffer_[idx].general_sensor2_status.radar2_c_fail_flag = static_cast<uint8_t>(msg->data[0]);
    buffer_[idx].general_sensor2_status.radar2_c_alive_counter = static_cast<uint8_t>(msg->data[1]);
    buffer_[idx].general_sensor2_status.radar2_c_failure_state = static_cast<uint8_t>(msg->data[2]);
    // std::cout << "radar2_c_general: "
    //           << static_cast<int>(buffer_[idx].general_sensor2_status.radar2_c_fail_flag) << " / "
    //           << static_cast<int>(buffer_[idx].general_sensor2_status.radar2_c_alive_counter) << " / " 
    //           << static_cast<int>(buffer_[idx].general_sensor2_status.radar2_c_failure_state) << " / "  << std::endl;
    
    buffer_[idx].general_sensor2_status.radar2_f_fail_flag = static_cast<uint8_t>(msg->data[0]);
    buffer_[idx].general_sensor2_status.radar2_f_alive_counter = static_cast<uint8_t>(msg->data[1]);
    buffer_[idx].general_sensor2_status.radar2_f_failure_state = static_cast<uint8_t>(msg->data[2]);

    buffer_[idx].general_sensor2_status.radar2_r_fail_flag = static_cast<uint8_t>(msg->data[0]);
    buffer_[idx].general_sensor2_status.radar2_r_alive_counter = static_cast<uint8_t>(msg->data[1]);
    buffer_[idx].general_sensor2_status.radar2_r_failure_state = static_cast<uint8_t>(msg->data[2]);
}


//-----------------------------------------------------------------------------------------------
void MsgSubscriber::callback_cam2(const fo_msgs::msg::Cam2Data::SharedPtr msg){
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();

    memset(&buffer_[idx].cam2_ld.cam2_left_lane_a, 0, sizeof(buffer_[idx].cam2_ld.cam2_left_lane_a));
    memset(&buffer_[idx].cam2_ld.cam2_right_lane_a, 0, sizeof(buffer_[idx].cam2_ld.cam2_right_lane_a));
    memset(&buffer_[idx].cam2_ld.cam2_left_lane_b, 0, sizeof(buffer_[idx].cam2_ld.cam2_left_lane_b));
    memset(&buffer_[idx].cam2_ld.cam2_right_lane_b, 0, sizeof(buffer_[idx].cam2_ld.cam2_right_lane_b));
    memset(&buffer_[idx].cam2_ld.cam2_lane_additional_data_1, 0, sizeof(buffer_[idx].cam2_ld.cam2_lane_additional_data_1));

    // Cam2_LD
    // Cam2_LD / Cam2_X_Lane_A (Left)
    buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_type = static_cast<uint64_t>(msg->ld.la.lane_mark_type);
    buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_quality = static_cast<uint64_t>(msg->ld.la.lane_mark_quality);
    buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_position = float_to_bit_signed(((msg->ld.la.lane_mark_position)*256), 16);
    buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_model_a = float_to_bit_unsigned((((msg->ld.la.lane_mark_model_a)*(1ULL<<32))+0x7FFFFFFF), 32);

    // Cam2_LD / Cam2_X_Lane_A (Right)
    buffer_[idx].cam2_ld.cam2_right_lane_a.lane_mark_type = static_cast<uint64_t>(msg->ld.ra.lane_mark_type);
    buffer_[idx].cam2_ld.cam2_right_lane_a.lane_mark_quality = static_cast<uint64_t>(msg->ld.ra.lane_mark_quality);
    buffer_[idx].cam2_ld.cam2_right_lane_a.lane_mark_position = float_to_bit_signed(((msg->ld.ra.lane_mark_position)*256), 16);
    buffer_[idx].cam2_ld.cam2_right_lane_a.lane_mark_model_a = float_to_bit_unsigned((((msg->ld.ra.lane_mark_model_a)*(1ULL<<32))+0x7FFFFFFF), 32);

    // Cam2_LD / Cam2_X_Lane_B (Left)
    float ld_left_heading_angle = std::clamp(msg->ld.lb.lane_mark_heading_angle, -0.357f, 0.357f);
    buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_heading_angle = float_to_bit_unsigned((((ld_left_heading_angle)*(0x10000))+0x7FFF), 16);
    buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_model_view_range = float_to_bit_unsigned(((msg->ld.lb.lane_mark_model_view_range)*256), 15);
    buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_model_view_range_availability = static_cast<uint64_t>(msg->ld.lb.lane_mark_model_view_range_availability);
    buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_model_da = float_to_bit_unsigned((((msg->ld.lb.lane_mark_model_da)*(1ULL<<36))+0x7FFFFFFF), 32);

    // Cam2_LD / Cam2_X_Lane_B (Right)
    float ld_right_heading_angle = std::clamp(msg->ld.rb.lane_mark_heading_angle, -0.357f, 0.357f);
    buffer_[idx].cam2_ld.cam2_right_lane_b.lane_mark_heading_angle = float_to_bit_unsigned((((ld_right_heading_angle)*(0x10000))+0x7FFF), 16);
    buffer_[idx].cam2_ld.cam2_right_lane_b.lane_mark_model_view_range = float_to_bit_unsigned(((msg->ld.rb.lane_mark_model_view_range)*256), 15);
    buffer_[idx].cam2_ld.cam2_right_lane_b.lane_mark_model_view_range_availability = static_cast<uint64_t>(msg->ld.rb.lane_mark_model_view_range_availability);
    buffer_[idx].cam2_ld.cam2_right_lane_b.lane_mark_model_da = float_to_bit_unsigned((((msg->ld.rb.lane_mark_model_da)*(1ULL<<36))+0x7FFFFFFF), 32);
    
    // Cam2_LD / Cam2_Lane_Additional_Data_1
    buffer_[idx].cam2_ld.cam2_lane_additional_data_1.alive_counter = static_cast<uint8_t>(msg->ld.add.rolling_counter);


    // Cam2_TD
    memset(buffer_[idx].cam2_td.cam2_obstacle_data_a, 0, sizeof(buffer_[idx].cam2_td.cam2_obstacle_data_a));
    memset(buffer_[idx].cam2_td.cam2_obstacle_data_b, 0, sizeof(buffer_[idx].cam2_td.cam2_obstacle_data_b));
    for (size_t i = 0; i < CAM_TRACK_NUM; i++) {
        // Cam2_Obstacle_Data_A
        float angle_rate = std::clamp(msg->td.tda[i].angle_rate, -100.0f, 100.0f);
        float angle_left = std::clamp(msg->td.tda[i].angle_left, -100.0f, 100.0f);
        float angle_right = std::clamp(msg->td.tda[i].angle_right, -100.0f, 100.0f);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].alive_counter = static_cast<uint64_t>(msg->td.tda[i].rolling_counter);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].object_age = static_cast<uint64_t>(msg->td.tda[i].object_age);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].angle_rate = float_to_bit_unsigned((((angle_rate)+100)/0.05), 12);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].angle_left = float_to_bit_unsigned((((angle_left)+100)/0.05), 12);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].angle_right = float_to_bit_unsigned((((angle_right)+100)/0.05), 12);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].motion_status = static_cast<uint64_t>(msg->td.tda[i].motion_status);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].object_lane = static_cast<uint64_t>(msg->td.tda[i].object_lane);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].cam2_obstacle_brake_lights = static_cast<uint64_t>(msg->td.tda[i].cam2_obstacle_brake_lights);

        // Cam2_Obstacle_Data_B
        double range = msg->td.tdb[i].range;
        uint64_t range_64 = float_to_bit_unsigned((range/0.1), 12);
        double range_rate = msg->td.tdb[i].range_rate;
        uint64_t range_rate_64 = float_to_bit_unsigned(((range_rate + 100)/0.1), 12);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].alive_counter = static_cast<uint8_t>(msg->td.tdb[i].rolling_counter);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].range_0 = static_cast<uint8_t>(range_64 & 0x3F);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].range_1 = static_cast<uint8_t>((range_64 >> 6) & 0x3F);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].object_vaildity = static_cast<uint8_t>(msg->td.tdb[i].object_vaildity);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].range_rate_0 = static_cast<uint8_t>(range_rate_64 & 0x3F);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].range_rate_1 = static_cast<uint8_t>((range_rate_64 >> 6) & 0x3F);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].cam2_obstacle_physical_width = float_to_bit_unsigned(((msg->td.tdb[i].cam2_obstacle_physical_width)/0.05), 8);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].cam2_track_id = static_cast<uint8_t>(msg->td.tdb[i].cam2_track_id);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].object_type = static_cast<uint8_t>(msg->td.tdb[i].object_type);
    }


    // Cam2_TL
    memset(&buffer_[idx].cam2_tl_cam2_traffic_light_data, 0, sizeof(buffer_[idx].cam2_tl_cam2_traffic_light_data));
    int tl_accuracy = std::clamp(static_cast<int>(msg->tl.traffic_light_accuracy), 0, 100);
    int64_t tl_accuracy_64 = float_to_bit_unsigned(tl_accuracy, 16);
    buffer_[idx].cam2_tl_cam2_traffic_light_data.traffic_light_data = static_cast<uint8_t>(msg->tl.traffic_light_data);
    buffer_[idx].cam2_tl_cam2_traffic_light_data.traffic_light_valid_flag = static_cast<uint8_t>(msg->tl.traffic_light_valid_flag);
    buffer_[idx].cam2_tl_cam2_traffic_light_data.traffic_light_accuracy_0 = static_cast<uint8_t>(tl_accuracy_64 & 0xFF);
    buffer_[idx].cam2_tl_cam2_traffic_light_data.traffic_light_accuracy_1 = static_cast<uint8_t>((tl_accuracy_64 >> 8) & 0xFF);


    // Cam2_OD
    memset(buffer_[idx].cam2_od.camera2_static_object_data, 0, sizeof(buffer_[idx].cam2_od.camera2_static_object_data));
    for (size_t i = 0; i < CAM_TRACK_NUM; i++) {
        buffer_[idx].cam2_od.camera2_static_object_data[i].rolling_count_1 = static_cast<uint8_t>(msg->od.sod[i].rolling_count_1);
        buffer_[idx].cam2_od.camera2_static_object_data[i].camera2_static_obj_type = static_cast<uint8_t>(msg->od.sod[i].camera2_static_obj_type);
        buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_status = static_cast<uint8_t>(msg->od.sod[i].static_object_status);
        buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos_y = float_to_bit_signed(((msg->od.sod[i].static_object_pos_y)/0.1), 10);
        buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos_x = float_to_bit_signed(((msg->od.sod[i].static_object_pos_x)/0.1), 11);
        buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos2_y = float_to_bit_signed(((msg->od.sod[i].static_object_pos2_y)/0.05), 10);
        buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos2_x = float_to_bit_signed(((msg->od.sod[i].static_object_pos2_x)/0.01), 11);
    }

    // Cam2_OD / drum.txt(EPSG:4326) 기반 정적 객체 - 자차 GPS(ego_gps_latitude_/longitude_/heading_deg_) 기준 로컬좌표 변환
    if (!static_object_gps_points_.empty()) {
        size_t track_slot = 0;
        for (size_t p = 0; p < static_object_gps_points_.size() && track_slot < CAM_TRACK_NUM; p++) {
            double local_x = 0.0, local_y = 0.0;
            latLonToLocalXY(static_object_gps_points_[p].first, static_object_gps_points_[p].second,
                             ego_gps_latitude_, ego_gps_longitude_, ego_gps_heading_deg_, local_x, local_y);

            if (std::hypot(local_x, local_y) > STATIC_OBJECT_RANGE_M) continue;

            buffer_[idx].cam2_od.camera2_static_object_data[track_slot].static_object_status = 1; // valid
            buffer_[idx].cam2_od.camera2_static_object_data[track_slot].static_object_pos_x = float_to_bit_signed((local_x/0.1), 11);
            buffer_[idx].cam2_od.camera2_static_object_data[track_slot].static_object_pos_y = float_to_bit_signed((local_y/0.1), 10);
            buffer_[idx].cam2_od.camera2_static_object_data[track_slot].static_object_pos2_x = float_to_bit_signed((local_x/0.01), 11);
            buffer_[idx].cam2_od.camera2_static_object_data[track_slot].static_object_pos2_y = float_to_bit_signed((local_y/0.05), 10);
            track_slot++;
        }
    }
}
//-----------------------------------------------------------------------------------------------
void MsgSubscriber::callback_lidar_td(const fo_msgs::msg::LidarobjListsForSF2::SharedPtr msg){
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();

    memset(buffer_[idx].lidar2_td.lidar2_track_a_fl, 0, sizeof(buffer_[idx].lidar2_td.lidar2_track_a_fl));
    memset(buffer_[idx].lidar2_td.lidar2_track_b_fl, 0, sizeof(buffer_[idx].lidar2_td.lidar2_track_b_fl));

    memset(buffer_[idx].lidar2_td.lidar2_track_a_fr, 0, sizeof(buffer_[idx].lidar2_td.lidar2_track_a_fr));
    memset(buffer_[idx].lidar2_td.lidar2_track_b_fr, 0, sizeof(buffer_[idx].lidar2_td.lidar2_track_b_fr));

    memset(buffer_[idx].lidar2_td.lidar2_track_a_r, 0, sizeof(buffer_[idx].lidar2_td.lidar2_track_a_r));
    memset(buffer_[idx].lidar2_td.lidar2_track_b_r, 0, sizeof(buffer_[idx].lidar2_td.lidar2_track_b_r));

    size_t idx_fl = 0, idx_fr = 0, idx_r = 0;

    for (size_t i = 0; i < msg->objs.size(); i++){
        auto& obj = msg->objs[i];

        // lidar_td_fl
        if (obj.pos_x >= 0 && obj.pos_y >= 0 && idx_fl < LID_TRACK_NUM) {
            // TD Track A FL
            buffer_[idx].lidar2_td.lidar2_track_a_fl[idx_fl].track_status = static_cast<uint64_t>(1);  // NTC
            buffer_[idx].lidar2_td.lidar2_track_a_fl[idx_fl].track_valid = static_cast<uint64_t>(obj.lidar_track_valid);
            buffer_[idx].lidar2_td.lidar2_track_a_fl[idx_fl].track_pos_y = float_to_bit_signed((obj.pos_y/0.1), 10);
            buffer_[idx].lidar2_td.lidar2_track_a_fl[idx_fl].track_pos_x = float_to_bit_signed((obj.pos_x/0.1), 11);
            buffer_[idx].lidar2_td.lidar2_track_a_fl[idx_fl].track_rolling_count = static_cast<uint64_t>(obj.rolling_count);
            // TD Track B FL
            buffer_[idx].lidar2_td.lidar2_track_b_fl[idx_fl].lidar_track_id = static_cast<uint64_t>(obj.lidar_track_id);
            buffer_[idx].lidar2_td.lidar2_track_b_fl[idx_fl].track_add_info_status = static_cast<uint64_t>(1);  // NTC
            buffer_[idx].lidar2_td.lidar2_track_b_fl[idx_fl].track_width = float_to_bit_signed((obj.width/0.1), 10);
            buffer_[idx].lidar2_td.lidar2_track_b_fl[idx_fl].track_length = float_to_bit_unsigned((obj.length/0.1), 11);
            buffer_[idx].lidar2_td.lidar2_track_b_fl[idx_fl].track_rolling_count = static_cast<uint64_t>(obj.rolling_count % 2);
            idx_fl++;
        }
        // lidar_td_fr
        else if (obj.pos_x >= 0 && obj.pos_y < 0 && idx_fr < LID_TRACK_NUM) {
            // TD Track A FR
            buffer_[idx].lidar2_td.lidar2_track_a_fr[idx_fr].track_status = static_cast<uint64_t>(1);  // NTC
            buffer_[idx].lidar2_td.lidar2_track_a_fr[idx_fr].track_valid = static_cast<uint64_t>(obj.lidar_track_valid);   // NTC
            buffer_[idx].lidar2_td.lidar2_track_a_fr[idx_fr].track_pos_y = float_to_bit_signed((obj.pos_y/0.1), 10);
            buffer_[idx].lidar2_td.lidar2_track_a_fr[idx_fr].track_pos_x = float_to_bit_signed((obj.pos_x/0.1), 11);
            buffer_[idx].lidar2_td.lidar2_track_a_fr[idx_fr].track_rolling_count = static_cast<uint64_t>(obj.rolling_count);
            // TD Track B FR
            buffer_[idx].lidar2_td.lidar2_track_b_fr[idx_fr].lidar_track_id = static_cast<uint64_t>(obj.lidar_track_id);
            buffer_[idx].lidar2_td.lidar2_track_b_fr[idx_fr].track_add_info_status = static_cast<uint64_t>(1);  // NTC
            buffer_[idx].lidar2_td.lidar2_track_b_fr[idx_fr].track_width = float_to_bit_signed((obj.width/0.1), 10);
            buffer_[idx].lidar2_td.lidar2_track_b_fr[idx_fr].track_length = float_to_bit_unsigned((obj.length/0.1), 11);
            buffer_[idx].lidar2_td.lidar2_track_b_fr[idx_fr].track_rolling_count = static_cast<uint64_t>(obj.rolling_count % 2);
            idx_fr++;
        }
        // lidar_td_r
        else if (obj.pos_x < 0 && idx_r < LID_TRACK_NUM) {
            // TD Track A R
            buffer_[idx].lidar2_td.lidar2_track_a_r[idx_r].track_status = static_cast<uint64_t>(1);  // NTC
            buffer_[idx].lidar2_td.lidar2_track_a_r[idx_r].track_valid = static_cast<uint64_t>(obj.lidar_track_valid);   // NTC
            buffer_[idx].lidar2_td.lidar2_track_a_r[idx_r].track_pos_y = float_to_bit_signed((obj.pos_y/0.1), 10);
            buffer_[idx].lidar2_td.lidar2_track_a_r[idx_r].track_pos_x = float_to_bit_signed((obj.pos_x/0.1), 11);
            buffer_[idx].lidar2_td.lidar2_track_a_r[idx_r].track_rolling_count = static_cast<uint64_t>(obj.rolling_count);
            // TD Track B R
            buffer_[idx].lidar2_td.lidar2_track_b_r[idx_r].lidar_track_id = static_cast<uint64_t>(obj.lidar_track_id);
            buffer_[idx].lidar2_td.lidar2_track_b_r[idx_r].track_add_info_status = static_cast<uint64_t>(1);  // NTC
            buffer_[idx].lidar2_td.lidar2_track_b_r[idx_r].track_width = float_to_bit_signed((obj.width/0.1), 10);
            buffer_[idx].lidar2_td.lidar2_track_b_r[idx_r].track_length = float_to_bit_unsigned((obj.length/0.1), 11);
            buffer_[idx].lidar2_td.lidar2_track_b_r[idx_r].track_rolling_count = static_cast<uint64_t>(obj.rolling_count % 2);
            idx_r++;
        }
    }
    // R 트랙 전체 32개 확인 (8바이트 * 32트랙)
    // for (size_t i = 0; i < LID_TRACK_NUM; i++) {
    //     auto &track_r = buffer_[idx].lidar2_td.lidar2_track_a_r[i];
    //     const uint8_t* p = reinterpret_cast<const uint8_t*>(&track_r);
    //     printf("Track R[%zu]: ", i);
    //     for (size_t j = 0; j < 8; ++j) {
    //         printf("%02X ", p[j]);
    //     }
    //     printf("\n");
    // }
}


//-----------------------------------------------------------------------------------------------
void MsgSubscriber::callback_radar_tr(const fo_msgs::msg::RadarTrArray::SharedPtr msg){
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();

    memset(buffer_[idx].radar2_tr.radar2_track_1_c, 0, sizeof(buffer_[idx].radar2_tr.radar2_track_1_c));
    memset(buffer_[idx].radar2_tr.radar2_track_1_f, 0, sizeof(buffer_[idx].radar2_tr.radar2_track_1_f));
    memset(buffer_[idx].radar2_tr.radar2_track_1_r, 0, sizeof(buffer_[idx].radar2_tr.radar2_track_1_r));

    uint64_t last_rolling_count = 0;
    for (size_t i = 0; i < msg->tracks.size(); i++){
        auto& track = msg->tracks[i];
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_id = static_cast<uint64_t>(track.track_id);                   // NTC
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_status = static_cast<uint64_t>(track.track_status);           // NTC
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_pos_y = float_to_bit_signed((track.track_pos_y/0.1), 10);     // NTC
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_pos_x = float_to_bit_signed((track.track_pos_x/0.1), 11);   // NTC
        buffer_[idx].radar2_tr.radar2_track_1_f[i].track_valid = static_cast<uint64_t>(track.track_valid);                    // NTC
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_type = static_cast<uint64_t>(track.track_type);               // NTC
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_vel_y = float_to_bit_signed((track.track_vel_y/0.05), 10);    // NTC
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_rolling_count = static_cast<uint64_t>(track.rolling_count);
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_vel_x = float_to_bit_signed((track.track_vel_x/0.01), 14);    // NTC
        last_rolling_count = static_cast<uint64_t>(track.rolling_count);
        // std::cout << "radar2_track_status: " << buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_status << std::endl;
        // std::cout << "radar2_track_pos_y: " << buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_pos_y << std::endl;
    }
}

//-----------------------------------------------------------------------------------------------
void MsgSubscriber::callback_sf2(const fo_msgs::msg::FusionobjListForSF2::SharedPtr msg){
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();

    memset(buffer_[idx].sf2_tr.sf2_track_1, 0, sizeof(buffer_[idx].sf2_tr.sf2_track_1));
    memset(buffer_[idx].sf2_tr.sf2_track_2, 0, sizeof(buffer_[idx].sf2_tr.sf2_track_2));

    size_t sf_track_count = std::min(static_cast<int>(msg->objs.size()), SF_TRACK_NUM);

    for (size_t i = 0; i < sf_track_count; i++){
        const auto& obj = msg->objs[i];

        // SF2_Track_1
        buffer_[idx].sf2_tr.sf2_track_1[i].track_valid = static_cast<uint64_t>(obj.track_valid);        // NTC
        buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_pos_y = float_to_bit_signed((obj.sf2_track_pos_y/0.1), 10);
        buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_pos_x = float_to_bit_signed((obj.sf2_track_pos_x/0.1), 11);
        buffer_[idx].sf2_tr.sf2_track_1[i].sf2_rolling_count = static_cast<uint64_t>(obj.sf2_rolling_count);
        buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_type = static_cast<uint64_t>(obj.sf2_track_type);
        buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_vel_y = float_to_bit_signed((obj.sf2_track_vel_y/0.05), 10);
        buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_vel_x = float_to_bit_signed((obj.sf2_track_vel_x/0.01), 14);
        sf2_counter = static_cast<uint64_t>(obj.sf2_rolling_count);
        // SF2_Track_2
        buffer_[idx].sf2_tr.sf2_track_2[i].sf2_track_id = static_cast<uint64_t>(obj.sf2_track_id);
    }
}

//-----------------------------------------------------------------------------------------------
void MsgSubscriber::callback_gnss_pvt(const ublox_ubx_msgs::msg::UBXNavPVT::SharedPtr msg){
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();

    buffer_[idx].gps.gps_rtk.rtk_latitude = static_cast<uint64_t>(msg->lat);  // 범위: 0~37xxxxxxx (9자리)
    buffer_[idx].gps.gps_rtk.rtk_longitude = static_cast<uint64_t>(msg->lon);  // 범위: 0~126xxxxxxx (10자리)
    buffer_[idx].gps.gps_fix_heading.rtk_rolling_count = static_cast<uint64_t>((msg->itow / 1000) % 16);

    // 0:No Fix 1:Single 2:DGPS 3:Float 4:Fixed
    int fix_flag;
    if (!msg->gnss_fix_ok) {
        fix_flag = 0;   // No Fix
    } else if (msg->carr_soln.status == 2) {
        fix_flag = 4;   // Fixed RTK
    } else if (msg->carr_soln.status == 1) {    
        fix_flag = 3;   // Float RTK
    } else if (msg->diff_soln) {
        fix_flag = 2;   // DGPS
    } else {
        fix_flag = 1;   // Single Point Positioning (보정 없음)
    }
    buffer_[idx].gps.gps_fix_heading.rtk_fix_flag = static_cast<uint64_t>(fix_flag);

    // std::cout << "rtk_latitude: " << buffer_[idx].gps.gps_rtk.rtk_latitude << std::endl;
    // std::cout << "rtk_longitude: " << buffer_[idx].gps.gps_rtk.rtk_longitude << std::endl;
    // std::cout << "fix_flag: " << buffer_[idx].gps.gps_fix_heading.rtk_fix_flag << std::endl;
    // std::cout << "rtk_rolling_count: " << buffer_[idx].gps.gps_fix_heading.rtk_rolling_count << std::endl;

    // Cam2_OD 정적 객체 로컬좌표 변환용 자차 위도/경도 [deg] (msg->lat/lon: 1e-7 deg scale)
    ego_gps_latitude_ = static_cast<double>(msg->lat);
    ego_gps_longitude_ = static_cast<double>(msg->lon);
}

void MsgSubscriber::callback_gnss_heading(const ublox_ubx_msgs::msg::UBXNavRelPosNED::SharedPtr msg){
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();

    if (msg->rel_pos_valid == 1) {
        buffer_[idx].gps.gps_fix_heading.rtk_heading = static_cast<uint64_t>((msg->rel_pos_heading * 1e-3));  // 범위: 0~36000
        // std::cout << "rtk_heading: " << buffer_[idx].gps.gps_fix_heading.rtk_heading << std::endl;
    
        // Cam2_OD 정적 객체 로컬좌표 변환용 자차 헤딩 [deg] (msg->rel_pos_heading: 1e-5 deg scale, 북쪽 0, 시계방향 증가)
        ego_gps_heading_deg_ = static_cast<double>(msg->rel_pos_heading) * 1e-5;  // 범위: 0~360
    }
    else {
        buffer_[idx].gps.gps_fix_heading.rtk_heading = static_cast<uint64_t>((calculate_heading * 1e-3));  // 범위: 0~36000
        // std::cout << "cal_heading: " << buffer_[idx].gps.gps_fix_heading.rtk_heading << std::endl;

        // Cam2_OD 정적 객체 로컬좌표 변환용 자차 헤딩 [deg] (msg->rel_pos_heading: 1e-5 deg scale, 북쪽 0, 시계방향 증가)
        ego_gps_heading_deg_ = static_cast<double>(calculate_heading) * 1e-5;  // 범위: 0~360
    }
}

void MsgSubscriber::callback_gnss_cal_heading(const std_msgs::msg::Int32::SharedPtr msg){
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();

    calculate_heading = msg->data;
    // std::cout << "[cal_buff] cal_heading: " << calculate_heading << std::endl;
    // std::cout << "" << std::endl;
}

//-----------------------------------------------------------------------------------------------

uint64_t MsgSubscriber::float_to_bit_signed(double value, int bit)
{
    // int64_t int_value = static_cast<int64_t>(value);

    // 비트 범위 계산
    double min_val = -(1LL << (bit - 1));
    double max_val =  (1LL << (bit - 1)) - 1;
    if (value < min_val) value = min_val;
    else if (value > max_val) value = max_val;

    // 음수 처리(2의 보수)
    if (value < 0) value = (1LL << bit) + value;
    int64_t int_value = static_cast<int64_t>(std::llround(value));

    // 마스킹 (비트수 초과시 쓰레기값 방지)
    uint64_t bits = int_value & ((1ULL << bit) - 1);
    return static_cast<uint64_t>(bits);
}

uint64_t MsgSubscriber::float_to_bit_unsigned(double value, int bit)
{
    // uint64_t int_value = static_cast<uint64_t>(value);

    // 비트 범위 계산
    double min_val = 0;
    double max_val = (1ULL << bit) - 1;
    if (value < min_val) value = min_val;
    else if (value > max_val) value = max_val;

    uint64_t int_value = static_cast<uint64_t>(std::llround(value));

    // 마스킹 (비트수 초과시 쓰레기값 방지)
    uint64_t bits = int_value & (1ULL << bit) - 1;
    return static_cast<uint64_t>(bits);
}

//-----------------------------------------------------------------------------------------------
// Cam2_OD 정적 객체용 GPS(EPSG:4326) 좌표 로드 및 로컬좌표 변환
void MsgSubscriber::loadStaticObjectGpsPoints(const std::string &file_path)
{
    static_object_gps_points_.clear();

    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open static object gps file: " << file_path << std::endl;
        return;
    }

    double lat, lon;
    while (file >> lat >> lon) {
        static_object_gps_points_.emplace_back(lat, lon);
    }
}

// 기준점(ref_lat, ref_lon) 대비 등거리 원통 투영(equirectangular) 근사로 북(N)/동(E) 오프셋을 구한 뒤,
// 자차 헤딩(ref_heading_deg, 북쪽 0 / 시계방향 증가)만큼 회전시켜 차량 좌표계(x=전방, y=좌측)로 변환
void MsgSubscriber::latLonToLocalXY(double lat, double lon, double ref_lat, double ref_lon, double ref_heading_deg, double &local_x, double &local_y)
{
    constexpr double kEarthRadiusM = 6378137.0;
    const double ref_lat_rad = ref_lat * M_PI / 180.0;

    const double north_offset = (lat - ref_lat) * M_PI / 180.0 * kEarthRadiusM;
    const double east_offset = (lon - ref_lon) * M_PI / 180.0 * kEarthRadiusM * std::cos(ref_lat_rad);

    const double heading_rad = ref_heading_deg * M_PI / 180.0;
    const double cos_h = std::cos(heading_rad);
    const double sin_h = std::sin(heading_rad);

    local_x = north_offset * cos_h + east_offset * sin_h;  // 전방
    local_y = north_offset * sin_h - east_offset * cos_h;  // 좌측
}

//-----------------------------------------------------------------------------------------------
// decode용임
int64_t MsgSubscriber::bit_to_signed(uint64_t value, int bit)
{
    if (bit <= 0 || bit > 63) throw std::invalid_argument("bit_to_signed: bit must be 1~63");

    const uint64_t mask = (1ULL << bit) - 1;
    const uint64_t sign_bit = 1ULL << (bit - 1);

    value &= mask;

    if (value & sign_bit) return static_cast<int64_t>(value) - static_cast<int64_t>(1ULL << bit);
    return static_cast<int64_t>(value);
}

uint64_t MsgSubscriber::bit_to_unsigned(uint64_t value, int bit)
{
    if (bit <= 0 || bit > 63) {
        throw std::invalid_argument("bit_to_unsigned: bit must be 1~63");
    }

    const uint64_t mask = (1ULL << bit) - 1;
    return value & mask;
}

//-----------------------------------------------------------------------------------------------

void MsgSubscriber::testBuff() {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int idx = write_idx_.load();
    
    /* ============================= general ============================= */
    buffer_[idx].general_sensor2_status.camera2_fail_flag = static_cast<uint8_t>(param_camera2_fail_flag_);
    buffer_[idx].general_sensor2_status.camera2_alive_counter = static_cast<uint8_t>(param_camera2_alive_counter_);
    buffer_[idx].general_sensor2_status.radar2_c_fail_flag = static_cast<uint8_t>(param_radar2_c_fail_flag_);
    buffer_[idx].general_sensor2_status.radar2_c_alive_counter = static_cast<uint8_t>(param_radar2_c_alive_counter_); 
    buffer_[idx].general_sensor2_status.radar2_f_fail_flag = static_cast<uint8_t>(param_radar2_f_fail_flag_);
    buffer_[idx].general_sensor2_status.radar2_f_alive_counter = static_cast<uint8_t>(param_radar2_f_alive_counter_);
    buffer_[idx].general_sensor2_status.radar2_r_fail_flag = static_cast<uint8_t>(param_radar2_r_fail_flag_); 
    buffer_[idx].general_sensor2_status.radar2_r_alive_counter = static_cast<uint8_t>(param_radar2_r_alive_counter_);
    buffer_[idx].general_sensor2_status.lidar2_fl_fail_flag = static_cast<uint8_t>(param_lidar2_fl_fail_flag_); 
    buffer_[idx].general_sensor2_status.lidar2_fl_alive_counter = static_cast<uint8_t>(param_lidar2_fl_alive_counter_); 
    buffer_[idx].general_sensor2_status.lidar2_fr_fail_flag = static_cast<uint8_t>(param_lidar2_fr_fail_flag_); 
    buffer_[idx].general_sensor2_status.lidar2_fr_alive_counter = static_cast<uint8_t>(param_lidar2_fr_alive_counter_);
    buffer_[idx].general_sensor2_status.lidar2_r_fail_flag = static_cast<uint8_t>(param_lidar2_r_fail_flag_); 
    buffer_[idx].general_sensor2_status.lidar2_r_alive_counter = static_cast<uint8_t>(param_lidar2_r_alive_counter_); 
    buffer_[idx].general_sensor2_status.gnss_fail_flag = static_cast<uint8_t>(param_gnss_fail_flag_);
    buffer_[idx].general_sensor2_status.gnss_alive_counter = static_cast<uint8_t>(param_gnss_alive_counter_);
    buffer_[idx].general_sensor2_status.v2x_fail_flag = static_cast<uint8_t>(param_v2x_fail_flag_); 
    buffer_[idx].general_sensor2_status.v2x_alive_counter = static_cast<uint8_t>(param_v2x_alive_counter_); 
    // std::cout << "camera2_fail_flag: " << static_cast<int>(buffer_[idx].general_sensor2_status.camera2_fail_flag) << std::endl;
    // std::cout << "camera2_alive_counter: " << static_cast<int>(buffer_[idx].general_sensor2_status.camera2_alive_counter) << std::endl;
    // std::cout << "v2x_alive_counter: " << static_cast<int>(buffer_[idx].general_sensor2_status.v2x_alive_counter) << std::endl;

    buffer_[idx].general_sensor2_status.camera2_failure_state = static_cast<uint8_t>(param_camera2_failure_state_);
    buffer_[idx].general_sensor2_status.radar2_c_failure_state = static_cast<uint8_t>(param_radar2_c_failure_state_);
    buffer_[idx].general_sensor2_status.radar2_f_failure_state = static_cast<uint8_t>(param_radar2_f_failure_state_);
    buffer_[idx].general_sensor2_status.radar2_r_failure_state = static_cast<uint8_t>(param_radar2_r_failure_state_);
    buffer_[idx].general_sensor2_status.lidar2_fl_failure_state = static_cast<uint8_t>(param_lidar2_fl_failure_state_);
    buffer_[idx].general_sensor2_status.lidar2_fr_failure_state = static_cast<uint8_t>(param_lidar2_fr_failure_state_);
    buffer_[idx].general_sensor2_status.lidar2_r_failure_state = static_cast<uint8_t>(param_lidar2_r_failure_state_);
    buffer_[idx].general_sensor2_status.gnss_failure_state = static_cast<uint8_t>(param_gnss_failure_state_);
    buffer_[idx].general_sensor2_status.v2x_failure_state = static_cast<uint8_t>(param_v2x_failure_state_);
    // std::cout << "camera2_failure_state: " << static_cast<int>(buffer_[idx].general_sensor2_status.camera2_failure_state) << std::endl;
    // std::cout << "============================================" << std::endl;

    buffer_[idx].general_sense_state.sense_staute_rolling_counter = static_cast<uint64_t>(param_sense_staute_rolling_counter_);
    buffer_[idx].general_sense_state.SW_VehiCommS = static_cast<uint64_t>(param_SW_VehiCommS_);
    buffer_[idx].general_sense_state.SW_SensProc = static_cast<uint64_t>(param_SW_SensProc_);
    buffer_[idx].general_sense_state.SW_SensFusion = static_cast<uint64_t>(param_SW_SensFusion_);
    buffer_[idx].general_sense_state.SW_ADSMonitorS = static_cast<uint64_t>(param_SW_ADSMonitorS_);
    buffer_[idx].general_sense_state.HW_ProcSense = static_cast<uint64_t>(param_HW_ProcSense_);
    buffer_[idx].general_sense_state.HW_ExtNPU = static_cast<uint64_t>(param_HW_ExtNPU_);
    buffer_[idx].general_sense_state.HW_PMICS = static_cast<uint64_t>(param_HW_PMICS_);
    buffer_[idx].general_sense_state.HW_EthPhySen = static_cast<uint64_t>(param_HW_EthPhySen_);
    buffer_[idx].general_sense_state.HW_EthPhyCam = static_cast<uint64_t>(param_HW_EthPhyCam_);
    buffer_[idx].general_sense_state.HW_CCANS = static_cast<uint64_t>(param_HW_CCANS_);
    buffer_[idx].general_sense_state.HW_IntCANS = static_cast<uint64_t>(param_HW_IntCANS_);
    buffer_[idx].general_sense_state.HW_DDR = static_cast<uint64_t>(param_HW_DDR_);
    buffer_[idx].general_sense_state.HW_eMMC = static_cast<uint64_t>(param_HW_eMMC_);
    buffer_[idx].general_sense_state.VEH_Chassis = static_cast<uint64_t>(param_VEH_Chassis_);
    buffer_[idx].general_sense_state.DNN_IMG = static_cast<uint64_t>(param_DNN_IMG_);
    buffer_[idx].general_sense_state.DNN_PCD = static_cast<uint64_t>(param_DNN_PCD_);
    // std::cout << "sense_staute_rolling_counter: " << static_cast<int>(buffer_[idx].general_sense_state.sense_staute_rolling_counter) << std::endl;
    // std::cout << "SW_VehiCommS: " << static_cast<int>(buffer_[idx].general_sense_state.SW_VehiCommS) << std::endl;
    // std::cout << "============================================" << std::endl;

    /* ============================= camera ============================= */
    // Cam2_LD / Cam2_X_Lane_A (Left)
    buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_type = static_cast<uint64_t>(param_lane_mark_type_);
    buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_quality = static_cast<uint64_t>(param_lane_mark_quality_);
    buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_position = float_to_bit_signed(((param_lane_mark_position_)*256), 16);
    buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_model_a = float_to_bit_unsigned((((param_lane_mark_model_a_)*(1ULL<<32))+0x7FFFFFFF), 32);
    // Cam2_LD / Cam2_X_Lane_A (Right)
    buffer_[idx].cam2_ld.cam2_right_lane_a.lane_mark_type = static_cast<uint64_t>(param_lane_mark_type_);
    buffer_[idx].cam2_ld.cam2_right_lane_a.lane_mark_quality = static_cast<uint64_t>(param_lane_mark_quality_);
    buffer_[idx].cam2_ld.cam2_right_lane_a.lane_mark_position = float_to_bit_signed(((param_lane_mark_position_)*256), 16);
    buffer_[idx].cam2_ld.cam2_right_lane_a.lane_mark_model_a = float_to_bit_unsigned((((param_lane_mark_model_a_)*(1ULL<<32))+0x7FFFFFFF), 32);
    // std::cout << "lane_mark_type: " << static_cast<int>(buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_type) << std::endl;
    // std::cout << "lane_mark_quality: " << static_cast<int>(buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_quality) << std::endl;
    
    // std::cout << "lane_mark_position: " << static_cast<double>(buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_position) << std::endl;
    // int64_t decode_lane_mark_position_ = bit_to_signed(buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_position, 16);
    // std::cout << "(decode)lane_mark_position: " << static_cast<double>(decode_lane_mark_position_)/256 << std::endl;
    
    // std::cout << "lane_mark_model_a: " << static_cast<double>(buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_model_a) << std::endl;
    // int64_t decode_lane_mark_model_a_ = bit_to_unsigned(buffer_[idx].cam2_ld.cam2_left_lane_a.lane_mark_model_a, 32);
    // std::cout << "(decode)lane_mark_model_a: " << (static_cast<double>(decode_lane_mark_model_a_)-0x7FFFFFFF)/(1ULL<<32) << std::endl;
    
    // std::cout << "============================================" << std::endl;


    // Cam2_LD / Cam2_X_Lane_B (Left)
    param_lane_mark_heading_angle_ = std::clamp(param_lane_mark_heading_angle_, -0.357, 0.357);
    buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_heading_angle = float_to_bit_unsigned((((param_lane_mark_heading_angle_)*(0x10000))+0x7FFF), 16); // 교수님과 프로토콜 문의 04.14
    buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_model_view_range = float_to_bit_unsigned(((param_lane_mark_model_view_range_)*256), 15); //
    buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_model_view_range_availability = static_cast<uint64_t>(param_lane_mark_model_view_range_availability_);
    buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_model_da = float_to_bit_unsigned((((param_lane_mark_model_da_)*(1ULL<<36))+0x7FFFFFFF), 32);
    // Cam2_LD / Cam2_X_Lane_B (Right)
    buffer_[idx].cam2_ld.cam2_right_lane_b.lane_mark_heading_angle = float_to_bit_unsigned((((param_lane_mark_heading_angle_)*(0x10000))+0x7FFF), 16); 
    buffer_[idx].cam2_ld.cam2_right_lane_b.lane_mark_model_view_range = float_to_bit_unsigned(((param_lane_mark_model_view_range_)*256), 15); 
    buffer_[idx].cam2_ld.cam2_right_lane_b.lane_mark_model_view_range_availability = static_cast<uint64_t>(param_lane_mark_model_view_range_availability_);
    buffer_[idx].cam2_ld.cam2_right_lane_b.lane_mark_model_da = float_to_bit_unsigned((((param_lane_mark_model_da_)*(1ULL<<36))+0x7FFFFFFF), 32);
    // std::cout << "lane_mark_heading_angle: " << static_cast<double>(buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_heading_angle) << std::endl;
    // int64_t decode_lane_mark_heading_angle_ = bit_to_unsigned(buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_heading_angle, 16);
    // std::cout << "(param)lane_mark_heading_angle: " << static_cast<double>(param_lane_mark_heading_angle_) << std::endl;
    // std::cout << "(decode)lane_mark_heading_angle: " << (static_cast<double>(decode_lane_mark_heading_angle_)-0x7FFF)/(0x10000) << std::endl;
    
    // std::cout << "lane_mark_model_view_range: " << static_cast<double>(buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_model_view_range) << std::endl;
    // int64_t decode_lane_mark_model_view_range = bit_to_unsigned(buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_model_view_range, 15);
    // std::cout << "(decode)lane_mark_model_view_range: " << static_cast<double>(decode_lane_mark_model_view_range)/256 << std::endl;
    
    // std::cout << "lane_mark_model_view_range_availability: " << static_cast<int>(buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_model_view_range_availability) << std::endl;
    
    // std::cout << "lane_mark_model_da: " << static_cast<double>(buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_model_da) << std::endl;
    // int64_t decode_lane_mark_model_da = bit_to_unsigned(buffer_[idx].cam2_ld.cam2_left_lane_b.lane_mark_model_da, 32);
    // std::cout << "(decode)lane_mark_model_da: " << (static_cast<double>(decode_lane_mark_model_da)-0x7FFFFFFF)/(1ULL<<36) << std::endl;
    // std::cout << "============================================" << std::endl;


    // Cam2_LD / Cam2_Lane_Additional_Data_1
    buffer_[idx].cam2_ld.cam2_lane_additional_data_1.alive_counter = static_cast<uint8_t>(param_alive_counter_ld_); 
    // std::cout << "alive_counter: " << static_cast<int>(buffer_[idx].cam2_ld.cam2_lane_additional_data_1.alive_counter) << std::endl;
    // std::cout << "============================================" << std::endl;

    
    // Cam2_TD
    for (size_t i = 0; i < 3; i++) {
        // Cam2_Obstacle_Data_A
        param_angle_rate_ = std::clamp(param_angle_rate_, -100.0, 100.0);
        param_angle_left_ = std::clamp(param_angle_left_, -100.0, 100.0);
        param_angle_right_ = std::clamp(param_angle_right_, -100.0, 100.0);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].alive_counter = static_cast<uint64_t>(param_alive_counter_td_a_);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].object_age = static_cast<uint64_t>(param_object_age_);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].angle_rate = float_to_bit_unsigned((((param_angle_rate_)+100)/0.05), 12); 
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].angle_left = float_to_bit_unsigned((((param_angle_left_)+100)/0.05), 12); 
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].angle_right = float_to_bit_unsigned((((param_angle_right_)+100)/0.05), 12); 
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].motion_status = static_cast<uint64_t>(param_motion_status_);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].object_lane = static_cast<uint64_t>(param_object_lane_);
        buffer_[idx].cam2_td.cam2_obstacle_data_a[i].cam2_obstacle_brake_lights = static_cast<uint64_t>(param_cam2_obstacle_brake_lights_);
        // std::cout << "alive_counter: " << static_cast<int>(buffer_[idx].cam2_td.cam2_obstacle_data_a[i].alive_counter) << std::endl;
        // std::cout << "object_age: " << static_cast<int>(buffer_[idx].cam2_td.cam2_obstacle_data_a[i].object_age) << std::endl;

        // std::cout << "angle_rate: " << static_cast<double>(buffer_[idx].cam2_td.cam2_obstacle_data_a[i].angle_rate) << std::endl;
        // int64_t decode_angle_rate_ = bit_to_unsigned(buffer_[idx].cam2_td.cam2_obstacle_data_a[i].angle_rate, 12);
        // std::cout << "(decode)angle_rate: " << (static_cast<double>(decode_angle_rate_)*0.05)-100 << std::endl;
        
        // std::cout << "angle_left: " << static_cast<double>(buffer_[idx].cam2_td.cam2_obstacle_data_a[i].angle_left) << std::endl;
        // int64_t decode_angle_left = bit_to_unsigned(buffer_[idx].cam2_td.cam2_obstacle_data_a[i].angle_left, 12);
        // std::cout << "(decode)angle_left: " << (static_cast<double>(decode_angle_left)*0.05)-100 << std::endl;

        // std::cout << "angle_right: " << static_cast<double>(buffer_[idx].cam2_td.cam2_obstacle_data_a[i].angle_right) << std::endl;
        // int64_t decode_angle_right_ = bit_to_unsigned(buffer_[idx].cam2_td.cam2_obstacle_data_a[i].angle_right, 12);
        // std::cout << "(decode)angle_right: " << (static_cast<double>(decode_angle_right_)*0.05)-100 << std::endl;

        // std::cout << "motion_status: " << static_cast<int>(buffer_[idx].cam2_td.cam2_obstacle_data_a[i].motion_status) << std::endl;
        // std::cout << "object_lane: " << static_cast<int>(buffer_[idx].cam2_td.cam2_obstacle_data_a[i].object_lane) << std::endl;
        // std::cout << "cam2_obstacle_brake_lights: " << static_cast<int>(buffer_[idx].cam2_td.cam2_obstacle_data_a[i].cam2_obstacle_brake_lights) << std::endl;
        // std::cout << "============================================" << std::endl;

        // Cam2_Obstacle_Data_B
        int64_t range_64 = float_to_bit_unsigned(param_range_/0.1, 12);
        int64_t range_rate_64 = float_to_bit_unsigned((param_range_rate_+100)/0.1, 12);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].alive_counter = static_cast<uint8_t>(param_alive_counter_td_a_);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].range_0 = static_cast<uint8_t>(range_64 & 0x3F);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].range_1 = static_cast<uint8_t>((range_64 >> 6) & 0x3F);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].object_vaildity = static_cast<uint8_t>(param_object_vaildity_);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].range_rate_0 = static_cast<uint8_t>(range_rate_64 & 0x3F);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].range_rate_1 = static_cast<uint8_t>((range_rate_64 >> 6) & 0x3F);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].cam2_obstacle_physical_width = float_to_bit_unsigned(((param_cam2_obstacle_physical_width_)/0.05), 8); 
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].cam2_track_id = static_cast<uint8_t>(param_cam2_track_id_);
        buffer_[idx].cam2_td.cam2_obstacle_data_b[i].object_type = static_cast<uint8_t>(param_object_type_);
        // std::cout << "alive_counter: " << static_cast<int>(buffer_[idx].cam2_td.cam2_obstacle_data_b[i].alive_counter) << std::endl;

        // int64_t decode_range_ = static_cast<uint16_t>(buffer_[idx].cam2_td.cam2_obstacle_data_b[i].range_0) | (static_cast<uint16_t>(buffer_[idx].cam2_td.cam2_obstacle_data_b[i].range_1) << 6);
        // std::cout << "(decode)range: " << static_cast<double>(decode_range_)*0.1 << std::endl;

        // std::cout << "object_vaildity: " << static_cast<int>(buffer_[idx].cam2_td.cam2_obstacle_data_b[i].object_vaildity) << std::endl;
        
        // int64_t decode_range_rate_ = static_cast<uint16_t>(buffer_[idx].cam2_td.cam2_obstacle_data_b[i].range_rate_0) | (static_cast<uint16_t>(buffer_[idx].cam2_td.cam2_obstacle_data_b[i].range_rate_1) << 6);
        // std::cout << "(decode)range_rate: " << (static_cast<double>(decode_range_rate_)*0.1)-100 << std::endl;

        // std::cout << "cam2_obstacle_physical_width: " << static_cast<double>(buffer_[idx].cam2_td.cam2_obstacle_data_b[i].cam2_obstacle_physical_width) << std::endl;
        // int64_t decode_cam2_obstacle_physical_width_ = bit_to_unsigned(buffer_[idx].cam2_td.cam2_obstacle_data_b[i].cam2_obstacle_physical_width, 8);
        // std::cout << "(decode)cam2_obstacle_physical_width: " << static_cast<double>(decode_cam2_obstacle_physical_width_)*0.05 << std::endl;

        // std::cout << "cam2_track_id: " << static_cast<int>(buffer_[idx].cam2_td.cam2_obstacle_data_b[i].cam2_track_id) << std::endl;
        // std::cout << "object_type: " << static_cast<int>(buffer_[idx].cam2_td.cam2_obstacle_data_b[i].object_type) << std::endl;
        // std::cout << "============================================" << std::endl;
    }
    param_alive_counter_td_a_ = (param_alive_counter_td_a_ + 1) % 4;

    // Cam2_TL_Cam2_Traffic_Light_Data
    int64_t tl_accuracy_64 = float_to_bit_unsigned(param_traffic_light_accuracy_, 16);
    param_traffic_light_accuracy_ = std::clamp(param_traffic_light_accuracy_, 0, 100);
    buffer_[idx].cam2_tl_cam2_traffic_light_data.traffic_light_data = static_cast<uint8_t>(param_traffic_light_data_);
    buffer_[idx].cam2_tl_cam2_traffic_light_data.traffic_light_valid_flag = static_cast<uint8_t>(param_traffic_light_valid_flag_);
    buffer_[idx].cam2_tl_cam2_traffic_light_data.traffic_light_accuracy_0 = static_cast<uint8_t>(tl_accuracy_64 & 0xFF);
    buffer_[idx].cam2_tl_cam2_traffic_light_data.traffic_light_accuracy_1 = static_cast<uint8_t>((tl_accuracy_64 >> 8) & 0xFF);
    // std::cout << "traffic_light_data: " << static_cast<int>(buffer_[idx].cam2_tl_cam2_traffic_light_data.traffic_light_data) << std::endl;
    // std::cout << "traffic_light_valid_flag: " << static_cast<int>(buffer_[idx].cam2_tl_cam2_traffic_light_data.traffic_light_valid_flag) << std::endl;
            
    // int64_t decode_tl_accuracy_ = static_cast<uint16_t>(buffer_[idx].cam2_tl_cam2_traffic_light_data.traffic_light_accuracy_0) | (static_cast<uint16_t>(buffer_[idx].cam2_tl_cam2_traffic_light_data.traffic_light_accuracy_1) << 6);
    // std::cout << "(decode)tl_accuracy: " << static_cast<int>(decode_tl_accuracy_) << std::endl;
    // std::cout << "============================================" << std::endl;

    // Cam2_OD / Camera2_Static_Object_Data
    for (size_t i = 0; i < 3; i++) {
        buffer_[idx].cam2_od.camera2_static_object_data[i].rolling_count_1 =  static_cast<uint8_t>(cam_od_counter);
        buffer_[idx].cam2_od.camera2_static_object_data[i].camera2_static_obj_type =  static_cast<uint8_t>(param_camera2_static_obj_type_);
        buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_status =  static_cast<uint8_t>(param_static_object_status_);
        buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos_y = float_to_bit_signed((param_static_object_pos_y_/0.1), 10); // 교수님께 문의
        buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos_x = float_to_bit_signed((param_static_object_pos_x_/0.1), 11); //
        buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos2_y = float_to_bit_signed((param_static_object_pos2_y_/0.05), 10); //
        buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos2_x = float_to_bit_signed((param_static_object_pos2_x_/0.01), 11); //
        // std::cout << "rolling_count_1: " << static_cast<int>(buffer_[idx].cam2_od.camera2_static_object_data[i].rolling_count_1) << std::endl;
        // std::cout << "camera2_static_obj_type: " << static_cast<int>(buffer_[idx].cam2_od.camera2_static_object_data[i].camera2_static_obj_type) << std::endl;
        // std::cout << "static_object_status: " << static_cast<int>(buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_status) << std::endl;

        // std::cout << "static_object_pos_y: " << static_cast<double>(buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos_y) << std::endl;
        // int64_t decode_static_object_pos_y = bit_to_signed(buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos_y, 10);
        // std::cout << "(decode)static_object_pos_y: " << static_cast<double>(decode_static_object_pos_y)*0.1 << std::endl;

        // std::cout << "static_object_pos_x: " << static_cast<double>(buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos_x) << std::endl;
        // int64_t decode_static_object_pos_x = bit_to_unsigned(buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos_x, 11);
        // std::cout << "(decode)static_object_pos_x: " << static_cast<double>(decode_static_object_pos_x)*0.1 << std::endl;

        // std::cout << "static_object_pos2_y: " << static_cast<double>(buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos2_y) << std::endl;
        // int64_t decode_static_object_pos2_y = bit_to_signed(buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos2_y, 10);
        // std::cout << "(decode)static_object_pos2_y: " << static_cast<double>(decode_static_object_pos2_y)*0.05 << std::endl;

        // std::cout << "static_object_pos2_x: " << static_cast<double>(buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos2_x) << std::endl;
        // int64_t decode_static_object_pos2_x = bit_to_signed(buffer_[idx].cam2_od.camera2_static_object_data[i].static_object_pos2_x, 11);
        // std::cout << "(decode)static_object_pos2_x: " << static_cast<double>(decode_static_object_pos2_x)*0.01 << std::endl;    
        // std::cout << "============================================" << std::endl;
    }
    cam_od_counter = (cam_od_counter + 1) % 4;

    /* ============================= lidar ============================= */
    for (size_t i = 0; i < 3; i++){
        // Lidar_TD Track A FL
        buffer_[idx].lidar2_td.lidar2_track_a_fl[i].track_status = static_cast<uint64_t>(param_track_status_);
        buffer_[idx].lidar2_td.lidar2_track_a_fl[i].track_valid = static_cast<uint64_t>(param_track_valid_lidar_);
        buffer_[idx].lidar2_td.lidar2_track_a_fl[i].track_pos_y = float_to_bit_signed((param_track_pos_y_/0.1), 10);
        buffer_[idx].lidar2_td.lidar2_track_a_fl[i].track_pos_x = float_to_bit_signed((param_track_pos_x_/0.1), 11); 
        buffer_[idx].lidar2_td.lidar2_track_a_fl[i].track_rolling_count = static_cast<uint64_t>(lidar_td_counter);
        // std::cout << "track_status: " << static_cast<int>(buffer_[idx].lidar2_td.lidar2_track_a_fl[i].track_status) << std::endl;
        // std::cout << "track_valid: " << static_cast<int>(buffer_[idx].lidar2_td.lidar2_track_a_fl[i].track_valid) << std::endl;
        
        // std::cout << "track_pos_y: " << static_cast<double>(buffer_[idx].lidar2_td.lidar2_track_a_fl[i].track_pos_y) << std::endl;
        // int64_t decode_track_pos_y = bit_to_signed(buffer_[idx].lidar2_td.lidar2_track_a_fl[i].track_pos_y, 10);
        // std::cout << "(decode)track_pos_y: " << static_cast<double>(decode_track_pos_y)*0.1 << std::endl;
        
        // std::cout << "track_pos_x: " << static_cast<double>(buffer_[idx].lidar2_td.lidar2_track_a_fl[i].track_pos_x) << std::endl;
        // int64_t decode_track_pos_x = bit_to_unsigned(buffer_[idx].lidar2_td.lidar2_track_a_fl[i].track_pos_x, 11);
        // std::cout << "(decode)track_pos_x: " << static_cast<double>(decode_track_pos_x)*0.1 << std::endl;
        
        // std::cout << "track_rolling_count: " << static_cast<int>(buffer_[idx].lidar2_td.lidar2_track_a_fl[i].track_rolling_count) << std::endl;
        // std::cout << "============================================" << std::endl;

        buffer_[idx].lidar2_td.lidar2_track_a_fr[i].track_status = static_cast<uint64_t>(param_track_status_);
        buffer_[idx].lidar2_td.lidar2_track_a_fr[i].track_valid = static_cast<uint64_t>(param_track_valid_lidar_);
        buffer_[idx].lidar2_td.lidar2_track_a_fr[i].track_pos_y = float_to_bit_signed((param_track_pos_y_/0.1), 10);
        buffer_[idx].lidar2_td.lidar2_track_a_fr[i].track_pos_x = float_to_bit_signed((param_track_pos_x_/0.1), 11); 
        buffer_[idx].lidar2_td.lidar2_track_a_fr[i].track_rolling_count = static_cast<uint64_t>(lidar_td_counter);

        buffer_[idx].lidar2_td.lidar2_track_a_r[i].track_status = static_cast<uint64_t>(param_track_status_);
        buffer_[idx].lidar2_td.lidar2_track_a_r[i].track_valid = static_cast<uint64_t>(param_track_valid_lidar_);
        buffer_[idx].lidar2_td.lidar2_track_a_r[i].track_pos_y = float_to_bit_signed((param_track_pos_y_/0.1), 10);
        buffer_[idx].lidar2_td.lidar2_track_a_r[i].track_pos_x = float_to_bit_signed((param_track_pos_x_/0.1), 11); 
        buffer_[idx].lidar2_td.lidar2_track_a_r[i].track_rolling_count = static_cast<uint64_t>(lidar_td_counter);

        // Lidar_TD Track B FL
        buffer_[idx].lidar2_td.lidar2_track_b_fl[i].lidar_track_id = static_cast<uint64_t>(param_lidar_track_id_);
        buffer_[idx].lidar2_td.lidar2_track_b_fl[i].track_add_info_status = static_cast<uint64_t>(param_track_add_info_status_);
        buffer_[idx].lidar2_td.lidar2_track_b_fl[i].track_width = float_to_bit_signed((param_track_width_/0.1), 10); 
        buffer_[idx].lidar2_td.lidar2_track_b_fl[i].track_length = float_to_bit_unsigned((param_track_length_/0.1), 11);
        buffer_[idx].lidar2_td.lidar2_track_b_fl[i].track_rolling_count = static_cast<uint64_t>(lidar_td_counter % 2); // 1bit인거 확인
        // std::cout << "lidar_track_id: " << static_cast<int>(buffer_[idx].lidar2_td.lidar2_track_b_fl[i].lidar_track_id) << std::endl;
        // std::cout << "track_add_info_status: " << static_cast<int>(buffer_[idx].lidar2_td.lidar2_track_b_fl[i].track_add_info_status) << std::endl;
        
        // std::cout << "track_width: " << static_cast<double>(buffer_[idx].lidar2_td.lidar2_track_b_fl[i].track_width) << std::endl;
        // int64_t decode_track_width = bit_to_signed(buffer_[idx].lidar2_td.lidar2_track_b_fl[i].track_width, 10);
        // std::cout << "(decode)track_width: " << static_cast<double>(decode_track_width)*0.1 << std::endl;
        
        // std::cout << "track_length: " << static_cast<double>(buffer_[idx].lidar2_td.lidar2_track_b_fl[i].track_length) << std::endl;
        // int64_t decode_track_length = bit_to_unsigned(buffer_[idx].lidar2_td.lidar2_track_b_fl[i].track_length, 11);
        // std::cout << "(decode)track_length: " << static_cast<double>(decode_track_length)*0.1 << std::endl;

        // std::cout << "track_rolling_count: " << static_cast<int>(buffer_[idx].lidar2_td.lidar2_track_b_fl[i].track_rolling_count) << std::endl;
        // std::cout << "============================================" << std::endl;

        buffer_[idx].lidar2_td.lidar2_track_b_fr[i].lidar_track_id = static_cast<uint64_t>(param_lidar_track_id_);
        buffer_[idx].lidar2_td.lidar2_track_b_fr[i].track_add_info_status = static_cast<uint64_t>(param_track_add_info_status_);
        buffer_[idx].lidar2_td.lidar2_track_b_fr[i].track_width = float_to_bit_signed((param_track_width_/0.1), 10); 
        buffer_[idx].lidar2_td.lidar2_track_b_fr[i].track_length = float_to_bit_unsigned((param_track_length_/0.1), 11);
        buffer_[idx].lidar2_td.lidar2_track_b_fr[i].track_rolling_count = static_cast<uint64_t>(lidar_td_counter % 2);

        buffer_[idx].lidar2_td.lidar2_track_b_r[i].lidar_track_id = static_cast<uint64_t>(param_lidar_track_id_);
        buffer_[idx].lidar2_td.lidar2_track_b_r[i].track_add_info_status = static_cast<uint64_t>(param_track_add_info_status_);
        buffer_[idx].lidar2_td.lidar2_track_b_r[i].track_width = float_to_bit_signed((param_track_width_/0.1), 10); 
        buffer_[idx].lidar2_td.lidar2_track_b_r[i].track_length = float_to_bit_unsigned((param_track_length_/0.1), 11);
        buffer_[idx].lidar2_td.lidar2_track_b_r[i].track_rolling_count = static_cast<uint64_t>(lidar_td_counter % 2);

        // 8바이트 확인 (i번째가 아니라 항상 0번째 기준이라면 이렇게)
        // auto &track_b_ref = buffer_[idx].lidar2_td.lidar2_track_b_fl[i];
        // if (i == 0) {
        //     const uint8_t* p = reinterpret_cast<const uint8_t*>(&track_b_ref);
        //     for (size_t j = 0; j < 8; ++j) {
        //         printf("%02X ", p[j]);
        //     }
        //     printf("\n");
        // }
    }
    lidar_td_counter = (lidar_td_counter + 1) % 4;


    /* ============================= radar ============================= */
    for (size_t i = 0; i < 3; i++){
        // Center
        buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_id = static_cast<uint64_t>(param_radar2_track_id_);
        buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_status = static_cast<uint64_t>(param_radar2_track_status_);
        buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_pos_y = float_to_bit_signed((param_radar2_track_pos_y_/0.1), 10); 
        buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_pos_x = float_to_bit_signed((param_radar2_track_pos_x_/0.1), 11); // 0.1이 줄어듦
        buffer_[idx].radar2_tr.radar2_track_1_c[i].track_valid = static_cast<uint64_t>(param_track_valid_radar_);
        buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_type = static_cast<uint64_t>(param_radar2_track_type_);
        buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_vel_y = float_to_bit_signed((param_radar2_track_vel_y_/0.05), 10);
        buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_rolling_count = static_cast<uint64_t>(radar_tr_c_counter);
        buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_vel_x = float_to_bit_signed((param_radar2_track_vel_x_/0.01), 14); /// 소수점 1번째자리부터
        // std::cout << "radar2_track_id: " << static_cast<int>(buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_id) << std::endl;
        // std::cout << "radar2_track_status: " << static_cast<int>(buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_status) << std::endl;
        
        // std::cout << "radar2_track_pos_y: " << static_cast<double>(buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_pos_y) << std::endl;
        // int64_t decode_radar2_track_pos_y = bit_to_signed(buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_pos_y, 10);
        // std::cout << "(decode)radar2_track_pos_y: " << static_cast<double>(decode_radar2_track_pos_y)*0.1 << std::endl;

        // std::cout << "radar2_track_pos_x: " << static_cast<double>(buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_pos_x) << std::endl;
        // int64_t decode_radar2_track_pos_x = bit_to_unsigned(buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_pos_x, 11);
        // std::cout << "(decode)radar2_track_pos_x: " << static_cast<double>(decode_radar2_track_pos_x)*0.1 << std::endl;

        // std::cout << "track_valid: " << static_cast<int>(buffer_[idx].radar2_tr.radar2_track_1_c[i].track_valid) << std::endl;
        // std::cout << "radar2_rolling_count: " << static_cast<int>(buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_rolling_count) << std::endl;
        // std::cout << "radar2_track_type: " << static_cast<int>(buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_type) << std::endl;

        // std::cout << "radar2_track_vel_y: " << static_cast<double>(buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_vel_y) << std::endl;
        // int64_t decode_radar2_track_vel_y = bit_to_signed(buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_vel_y, 10);
        // std::cout << "(decode)radar2_track_vel_y: " << static_cast<double>(decode_radar2_track_vel_y)*0.05 << std::endl;
        
        // std::cout << "radar2_track_vel_x: " << static_cast<double>(buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_vel_x) << std::endl;
        // int64_t decode_radar2_track_vel_x = bit_to_signed(buffer_[idx].radar2_tr.radar2_track_1_c[i].radar2_track_vel_x, 14);
        // std::cout << "(decode)radar2_track_vel_x: " << static_cast<double>(decode_radar2_track_vel_x)*0.01 << std::endl;
        // std::cout << "============================================" << std::endl;

        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_id = static_cast<uint64_t>(param_radar2_track_id_);
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_status = static_cast<uint64_t>(param_radar2_track_status_);
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_pos_y = float_to_bit_signed((param_radar2_track_pos_y_/0.1), 10); 
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_pos_x = float_to_bit_signed((param_radar2_track_pos_x_/0.1), 11); // 0.1이 줄어듦
        buffer_[idx].radar2_tr.radar2_track_1_f[i].track_valid = static_cast<uint64_t>(param_track_valid_radar_);
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_type = static_cast<uint64_t>(param_radar2_track_type_);
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_vel_y = float_to_bit_signed((param_radar2_track_vel_y_/0.05), 10);
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_rolling_count = static_cast<uint64_t>(radar_tr_c_counter);
        buffer_[idx].radar2_tr.radar2_track_1_f[i].radar2_track_vel_x = float_to_bit_signed((param_radar2_track_vel_x_/0.01), 14);

        buffer_[idx].radar2_tr.radar2_track_1_r[i].radar2_track_id = static_cast<uint64_t>(param_radar2_track_id_);
        buffer_[idx].radar2_tr.radar2_track_1_r[i].radar2_track_status = static_cast<uint64_t>(param_radar2_track_status_);
        buffer_[idx].radar2_tr.radar2_track_1_r[i].radar2_track_pos_y = float_to_bit_signed((param_radar2_track_pos_y_/0.1), 10); 
        buffer_[idx].radar2_tr.radar2_track_1_r[i].radar2_track_pos_x = float_to_bit_signed((param_radar2_track_pos_x_/0.1), 11); // 0.1이 줄어듦
        buffer_[idx].radar2_tr.radar2_track_1_r[i].track_valid = static_cast<uint64_t>(param_track_valid_radar_);
        buffer_[idx].radar2_tr.radar2_track_1_r[i].radar2_track_type = static_cast<uint64_t>(param_radar2_track_type_);
        buffer_[idx].radar2_tr.radar2_track_1_r[i].radar2_track_vel_y = float_to_bit_signed((param_radar2_track_vel_y_/0.05), 10);
        buffer_[idx].radar2_tr.radar2_track_1_r[i].radar2_rolling_count = static_cast<uint64_t>(radar_tr_c_counter);
        buffer_[idx].radar2_tr.radar2_track_1_r[i].radar2_track_vel_x = float_to_bit_signed((param_radar2_track_vel_x_/0.01), 14);
    }
    radar_tr_c_counter = (radar_tr_c_counter + 1) % 4;

    /* ============================= sf2 ============================= */
    for (size_t i = 0; i<3; i++){
        // SF2_Track_1
        buffer_[idx].sf2_tr.sf2_track_1[i].track_valid = static_cast<uint64_t>(param_track_valid_sf2_);
        buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_pos_y = float_to_bit_signed((param_sf2_track_pos_y_/0.1), 10);
        buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_pos_x = float_to_bit_signed((param_sf2_track_pos_x_/0.1), 11);
        buffer_[idx].sf2_tr.sf2_track_1[i].sf2_rolling_count = static_cast<uint64_t>(sf2_counter);
        buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_type = static_cast<uint64_t>(param_sf2_track_type_);
        buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_vel_y = float_to_bit_signed((param_sf2_track_vel_y_/0.05), 10);
        buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_vel_x = float_to_bit_signed((param_sf2_track_vel_x_/0.01), 14);
        // SF2_Track_2
        buffer_[idx].sf2_tr.sf2_track_2[i].sf2_track_id = static_cast<uint64_t>(param_sf2_track_id_);

        // std::cout << "track_valid: " << static_cast<int>(buffer_[idx].sf2_tr.sf2_track_1[i].track_valid) << std::endl;
                
        // std::cout << "sf2_track_pos_y: " << static_cast<double>(buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_pos_y) << std::endl;
        // int64_t decode_sf2_track_pos_y = bit_to_signed(buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_pos_y, 10);
        // std::cout << "(decode)sf2_track_pos_y: " << static_cast<double>(decode_sf2_track_pos_y)*0.1 << std::endl;

        // std::cout << "sf2_track_pos_x: " << static_cast<double>(buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_pos_x) << std::endl;
        // int64_t decode_sf2_track_pos_x = bit_to_unsigned(buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_pos_x, 11);
        // std::cout << "(decode)sf2_track_pos_x: " << static_cast<double>(decode_sf2_track_pos_x)*0.1 << std::endl;
        
        // std::cout << "sf2_rolling_count: " << static_cast<int>(buffer_[idx].sf2_tr.sf2_track_1[i].sf2_rolling_count) << std::endl;
        // std::cout << "sf2_track_type: " << static_cast<int>(buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_type) << std::endl;

        // std::cout << "sf2_track_vel_y: " << static_cast<double>(buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_vel_y) << std::endl;
        // int64_t decode_sf2_track_vel_y = bit_to_signed(buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_vel_y, 10);
        // std::cout << "(decode)sf2_track_vel_y: " << static_cast<double>(decode_sf2_track_vel_y)*0.05 << std::endl;

        // std::cout << "sf2_track_vel_x: " << static_cast<double>(buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_vel_x) << std::endl;
        // int64_t decode_sf2_track_vel_x = bit_to_signed(buffer_[idx].sf2_tr.sf2_track_1[i].sf2_track_vel_x, 14);
        // std::cout << "(decode)sf2_track_vel_x: " << static_cast<double>(decode_sf2_track_vel_x)*0.01 << std::endl;
        // std::cout << "============================================" << std::endl;

        // std::cout << "sf2_track_id: " << static_cast<int>(buffer_[idx].sf2_tr.sf2_track_2[i].sf2_track_id) << std::endl;
        // std::cout << "============================================" << std::endl;
    }
    sf2_counter = (sf2_counter + 1) % 4;

    /* ============================= gps ============================= */
    buffer_[idx].gps.gps_rtk.rtk_latitude = static_cast<uint64_t>(param_rtk_latitude_); 
    buffer_[idx].gps.gps_rtk.rtk_longitude = static_cast<uint64_t>(param_rtk_longitude_);
    buffer_[idx].gps.gps_fix_heading.rtk_rolling_count = static_cast<uint64_t>(gnss_rtk_counter);
    buffer_[idx].gps.gps_fix_heading.rtk_heading = static_cast<uint64_t>(param_rtk_heading_);
    buffer_[idx].gps.gps_fix_heading.rtk_fix_flag = static_cast<uint64_t>(param_rtk_fix_flag_);
    // std::cout << "rtk_latitude: " << static_cast<int>(buffer_[idx].gps.gps_rtk.rtk_latitude) << std::endl;
    // std::cout << "rtk_longitude: " << static_cast<int>(buffer_[idx].gps.gps_rtk.rtk_longitude) << std::endl;
    // std::cout << "rtk_rolling_count: " << static_cast<int>(buffer_[idx].gps.gps_fix_heading.rtk_rolling_count) << std::endl;
    // std::cout << "rtk_heading: " << static_cast<int>(buffer_[idx].gps.gps_fix_heading.rtk_heading) << std::endl;
    // std::cout << "rtk_fix_flag: " << static_cast<int>(buffer_[idx].gps.gps_fix_heading.rtk_fix_flag) << std::endl;
    // std::cout << "============================================" << std::endl;
    gnss_rtk_counter = (gnss_rtk_counter + 1) % 16;
}

//----------------------------------------------

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MsgSubscriber>();

    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize MsgSubscriber");
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
//----------------------------------------------
