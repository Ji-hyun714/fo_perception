#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>
#include <atomic>
#include <vector>
#include <utility>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fo_msgs/msg/radar_tr_array.hpp"
#include "fo_msgs/msg/radar_tr.hpp"

#include "fo_msgs/msg/cam2_data.hpp"
#include "fo_msgs/msg/cam2_ld.hpp"
#include "fo_msgs/msg/cam2_td.hpp"
#include "fo_msgs/msg/cam2_tl.hpp"
#include "fo_msgs/msg/cam2_od.hpp"
#include "fo_msgs/msg/lidarobj_lists_for_sf2.hpp"
#include "fo_msgs/msg/status_camera.hpp"

#include "fo_msgs/msg/fusionobj_list_for_sf2.hpp"

#include "ublox_ubx_msgs/msg/ubx_nav_pvt.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_rel_pos_ned.hpp"

// #define DATA_FIELD_SIZE 5346
#define DATA_FIELD_SIZE 3036
#define CAM_TRACK_NUM 8
#define LID_TRACK_NUM 32
#define RAD_TRACK_NUM 64
#define SF_TRACK_NUM 32
#define STATIC_OBJECT_RANGE_M 20.0 // Cam2_OD 정적 객체 필터링 반경 (자차 기준, m)

// #define SERVER_IP       "192.168.1.78"
// #define SERVER_IP       "192.168.0.81" // 3차 보드 IP
#define SERVER_IP       "192.168.0.202" // 3차 보드 IP
// #define SERVER_IP       "192.168.0.78"
// #define SERVER_IP       "192.168.100.60"  // hostname -I로 출력된 ip (orin에 할당된 ip)  192.168.100.141 / 192.168.55.1 / 172.17.0.1
#define SERVER_PORT     3000
// #define UDP_SEND_PORT   22222


/* FO SPI 메시지 구조체 정의*/ // TODO: Check size of every struct 
#pragma pack(push, 1)

struct General_Sensor2_Status // Be masked from the lowest bit.
{
    uint8_t camera2_fail_flag       : 1;
    uint8_t camera2_alive_counter   : 7;
    uint8_t radar2_c_fail_flag      : 1;
    uint8_t radar2_c_alive_counter  : 7;
    uint8_t radar2_f_fail_flag      : 1;
    uint8_t radar2_f_alive_counter  : 7;
    uint8_t radar2_r_fail_flag      : 1;
    uint8_t radar2_r_alive_counter  : 7;
    uint8_t lidar2_fl_fail_flag     : 1;
    uint8_t lidar2_fl_alive_counter : 7;
    uint8_t lidar2_fr_fail_flag     : 1;
    uint8_t lidar2_fr_alive_counter : 7;
    uint8_t lidar2_r_fail_flag      : 1;
    uint8_t lidar2_r_alive_counter  : 7;
    uint8_t gnss_fail_flag          : 1;
    uint8_t gnss_alive_counter      : 7;
    uint8_t v2x_fail_flag           : 1;
    uint8_t v2x_alive_counter       : 7;

    uint8_t camera2_failure_state   : 4;
    uint8_t radar2_c_failure_state  : 4;
    uint8_t radar2_f_failure_state  : 4;
    uint8_t radar2_r_failure_state  : 4;
    uint8_t lidar2_fl_failure_state : 4;
    uint8_t lidar2_fr_failure_state : 4;
    uint8_t lidar2_r_failure_state  : 4;
    uint8_t gnss_failure_state      : 4;
    uint8_t                         : 4;
    uint8_t v2x_failure_state       : 4;
}; // total 14 bytes

struct General_SenseState
{
    uint64_t sense_staute_rolling_counter : 8; // staute? state?
    uint64_t SW_VehiCommS                 : 4;
    uint64_t SW_SensProc                  : 4;
    uint64_t SW_SensFusion                : 4;
    uint64_t SW_ADSMonitorS               : 4;
    uint64_t HW_ProcSense                 : 4;
    uint64_t HW_ExtNPU                    : 4;
    uint64_t HW_PMICS                     : 4;
    uint64_t HW_EthPhySen                 : 2;
    uint64_t HW_EthPhyCam                 : 2;
    uint64_t HW_CCANS                     : 2;
    uint64_t HW_IntCANS                   : 2;
    uint64_t HW_DDR                       : 4;
    uint64_t HW_eMMC                      : 4;
    uint64_t VEH_Chassis                  : 2;
    uint64_t                              : 2;
    uint64_t DNN_IMG                      : 4;
    uint64_t DNN_PCD                      : 4;
}; //total 8 bytes


/// Cam2_LD
struct Cam2_X_Lane_A // L, R
{
    uint64_t lane_mark_type     : 4;
    uint64_t lane_mark_quality  : 2;
    uint64_t                    : 2; //pad
    uint64_t lane_mark_position : 16;
    uint64_t lane_mark_model_a  : 32;
}; // total 7 bytes

struct Cam2_X_Lane_B // L, R
{
    uint64_t lane_mark_heading_angle                 : 16;
    uint64_t lane_mark_model_view_range              : 15;
    uint64_t lane_mark_model_view_range_availability : 1;
    uint64_t lane_mark_model_da                      : 32;
}; // total 8 bytes

struct Cam2_Lane_Additional_Data_1
{
    uint8_t alive_counter                : 8;
}; // total 1 bytes

struct Cam2_LD
{
    struct Cam2_X_Lane_A cam2_left_lane_a;                          // 7
    struct Cam2_X_Lane_A cam2_right_lane_a;                         // 7  
    struct Cam2_X_Lane_B cam2_left_lane_b;                          // 8
    struct Cam2_X_Lane_B cam2_right_lane_b;                         // 8 
    struct Cam2_Lane_Additional_Data_1 cam2_lane_additional_data_1; // 1
}; // total 31 bytes


/// Cam2_TD
struct Cam2_Obstacle_Data_A // 16 tracks
{
    uint64_t alive_counter              : 2;  //rolling counter
    uint64_t                            : 6;
    uint64_t object_age                 : 8;
    uint64_t angle_rate                 : 12;
    uint64_t angle_left                 : 12;
    uint64_t angle_right                : 12;
    uint64_t motion_status              : 4;
    uint64_t object_lane                : 4;
    uint64_t cam2_obstacle_brake_lights : 1;
    uint64_t                            : 3;
}; // total 8 bytes

struct Cam2_Obstacle_Data_B // 16 tracks
{
    uint8_t alive_counter                : 2;
    uint8_t range_0                      : 6;
    uint8_t range_1                      : 6; // range : 12 bits
    uint8_t object_vaildity              : 2;
    uint8_t                              : 2;
    uint8_t range_rate_0                 : 6;
    uint8_t range_rate_1                 : 6; // range_rate : 12 bits
    uint8_t                              : 2;
    uint8_t cam2_obstacle_physical_width : 8;
    uint8_t cam2_track_id                : 8;
    uint8_t object_type                  : 4;
    uint8_t                              : 4;
}; // total 7 bytes

struct Cam2_TD
{
    struct Cam2_Obstacle_Data_A cam2_obstacle_data_a[CAM_TRACK_NUM]; // 8*8 = 64 bytes
    struct Cam2_Obstacle_Data_B cam2_obstacle_data_b[CAM_TRACK_NUM]; // 7*8 = 56 bytes
}; // total 120 bytes

/// Cam2_TL
struct Cam2_TL_Cam2_Traffic_Light_Data
{
    uint8_t traffic_light_data       : 4;
    uint8_t                          : 1;
    uint8_t traffic_light_valid_flag : 2; 
    uint8_t                          : 1;
    uint8_t traffic_light_accuracy_0 : 8;
    uint8_t traffic_light_accuracy_1 : 8; // traffic_light_accuracy : 16 bits
}; // total 3 bytes

// Cam2_OD
struct Camera2_Static_Object_Data // 16 tracks
{
    uint64_t rolling_count_1         : 2;
    uint64_t camera2_static_obj_type : 4;
    uint64_t                         : 2;
    uint64_t static_object_status    : 3;
    uint64_t static_object_pos_y     : 10;
    uint64_t static_object_pos_x     : 11;
    uint64_t                         : 6;
    uint64_t static_object_pos2_y    : 10;
    uint64_t static_object_pos2_x    : 11;
    uint64_t                         : 5;
}; // total 8 bytes

struct Cam2_OD
{
    struct Camera2_Static_Object_Data camera2_static_object_data[CAM_TRACK_NUM]; // 8*8 = 64 bytes
};

struct Cam2_Data
{
    Cam2_LD                         ld; // 31
    Cam2_TD                         td; // 120
    Cam2_TL_Cam2_Traffic_Light_Data tl; // 3
    Cam2_OD                         od; // 64
}; // total 218 bytes

/// Lidar2_TD
struct Lidar2_Track_A_X // FL, FR, R //32 tracks
{
    uint64_t track_status        : 3;
    uint64_t track_valid         : 1;
    uint64_t                     : 2;
    uint64_t track_pos_y         : 10;
    uint64_t track_pos_x         : 11;
    uint64_t                     : 5;
    uint64_t                     : 2;
    uint64_t track_rolling_count : 2;
    uint64_t                     : 4;
}; // total 5 bytes 

struct Lidar2_Track_B_X // FL, FR, R //32 tracks
{
    uint64_t lidar_track_id        : 8;
    uint64_t track_add_info_status : 3;
    uint64_t track_width           : 10;
    uint64_t track_length          : 11;
    uint64_t                       : 5;
    uint64_t track_rolling_count   : 1;
    uint64_t                       : 2;
}; // total 5 bytes

struct Lidar2_TD
{
    struct Lidar2_Track_A_X lidar2_track_a_fl[LID_TRACK_NUM]; // 5*32 = 160
    struct Lidar2_Track_A_X lidar2_track_a_fr[LID_TRACK_NUM]; // 5*32 = 160
    struct Lidar2_Track_A_X lidar2_track_a_r[LID_TRACK_NUM];  // 5*32 = 160

    struct Lidar2_Track_B_X lidar2_track_b_fl[LID_TRACK_NUM]; // 5*32 = 160
    struct Lidar2_Track_B_X lidar2_track_b_fr[LID_TRACK_NUM]; // 5*32 = 160
    struct Lidar2_Track_B_X lidar2_track_b_r[LID_TRACK_NUM];  // 5*32 = 160
}; // total 960 bytes

/// Radar2_TR
struct Radar2_Track_1 // 64 track
{
    uint64_t radar2_track_id      : 8;
    uint64_t radar2_track_status  : 3;
    uint64_t radar2_track_pos_y   : 10;
    uint64_t radar2_track_pos_x   : 11;
    uint64_t track_valid          : 2;
    uint64_t radar2_track_type    : 4;
    uint64_t radar2_track_vel_y   : 10;
    uint64_t radar2_rolling_count : 2;
    uint64_t radar2_track_vel_x   : 14;
}; // total 8 bytes

struct Radar2_TR
{
    struct Radar2_Track_1 radar2_track_1_c[RAD_TRACK_NUM]; // 8 * 64 = 512
    struct Radar2_Track_1 radar2_track_1_f[RAD_TRACK_NUM]; // 8 * 64 = 512
    struct Radar2_Track_1 radar2_track_1_r[RAD_TRACK_NUM]; // 8 * 64 = 512
}; // total 1536 bytes

/// SF2_TR
struct SF2_Track_1 // 32 track
{
    uint64_t track_valid       : 3;
    uint64_t                   : 5;
    uint64_t                   : 3;
    uint64_t sf2_track_pos_y   : 10;
    uint64_t sf2_track_pos_x   : 11;
    uint64_t sf2_rolling_count : 2;
    uint64_t sf2_track_type    : 4;
    uint64_t sf2_track_vel_y   : 10;
    uint64_t                   : 2;
    uint64_t sf2_track_vel_x   : 14;
}; // total 8 bytes

struct SF2_Track_2 // 32 track
{
    uint64_t sf2_track_id                : 8;
}; // total 1 bytes

struct SF2_TR
{
    struct SF2_Track_1 sf2_track_1[SF_TRACK_NUM]; // 8 * 32 = 256 
    struct SF2_Track_2 sf2_track_2[SF_TRACK_NUM]; // 1 * 32 = 32
}; // total 288 bytes

/// GPS
struct GPS_RTK
{
    uint64_t rtk_latitude  : 32;
    uint64_t rtk_longitude : 32;
}; // total 8 bytes

struct GPS_Fix_Heading
{
    uint16_t rtk_rolling_count : 4;
    uint16_t                   : 4;
    uint16_t rtk_heading       : 16;
    uint16_t rtk_fix_flag      : 3;
    uint16_t                   : 5;
}; // total 4 bytes

struct GPS
{
    struct GPS_RTK gps_rtk;                 // 8
    struct GPS_Fix_Heading gps_fix_heading; // 4
}; // total 12 bytes


struct UDP_DATA
{
    struct General_Sensor2_Status general_sensor2_status; // 14
    struct General_SenseState general_sense_state; // 8
    
    struct Cam2_LD cam2_ld; // (7+7)+(8+8)+1 = 31
    struct Cam2_TD cam2_td; // 8*(8+7) = 120
    struct Cam2_TL_Cam2_Traffic_Light_Data cam2_tl_cam2_traffic_light_data; // 3
    struct Cam2_OD cam2_od; // 8*8 = 64

    struct Lidar2_TD lidar2_td; // 32*(5+5)*3 = 960

    struct Radar2_TR radar2_tr; // 64*8*3 = 1536

    struct SF2_TR sf2_tr; // 32*(8+1) = 288

    struct GPS gps; // 8+4 = 12
};
// total 3036 bytes

#pragma pack(pop)
// header 12 + data 3036 + tail 4
// total 3052 bytes




class MsgSubscriber : public rclcpp::Node
{
public:
    MsgSubscriber();
    virtual ~MsgSubscriber() override;

    bool init();

    UDP_DATA tmpBuff_;

private:
    // func
    void swapBuff();
    void sendBuff();
    void testBuff();
    void loadYAML();
    void checkGeneralSenseState(); // callback_sf2의 sf2_counter 정체 여부로 SW_SensFusion 상태 갱신
    void checkSensorProcState();   // lidar2_general/radar_general/camera_general fail_flag로 SW_SensProc 상태 갱신
    
    // subscription
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr lidar_general_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr radar_general_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_delay_fl_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_delay_fr_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_delay_r_sub_;

    rclcpp::Subscription<fo_msgs::msg::LidarobjListsForSF2>::SharedPtr lidar_obj_sub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_coef_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_coef_sub_;
    
    rclcpp::Subscription<fo_msgs::msg::RadarTrArray>::SharedPtr radar_tr_sub_;
    
    rclcpp::Subscription<fo_msgs::msg::Cam2Data>::SharedPtr cam2_sub_;
    rclcpp::Subscription<fo_msgs::msg::StatusCamera>::SharedPtr camera_general_sub_;

    rclcpp::Subscription<fo_msgs::msg::FusionobjListForSF2>::SharedPtr sf2_sub_;

    rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavPVT>::SharedPtr gnss_pvt_sub_;
    rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavRelPosNED>::SharedPtr gnss_heading_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gnss_cal_heading_sub_;

    // callback func
    void callback_lidar_general(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void callback_radar_general(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void callback_camera_general(const fo_msgs::msg::StatusCamera::SharedPtr msg);

    void callback_lidar1_delay(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_lidar2_delay(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_lidar3_delay(const std_msgs::msg::Bool::SharedPtr msg);
    
    void callback_lidar_td(const fo_msgs::msg::LidarobjListsForSF2::SharedPtr msg);
    void callback_radar_tr(const fo_msgs::msg::RadarTrArray::SharedPtr msg);
    void callback_cam2(const fo_msgs::msg::Cam2Data::SharedPtr msg);
    void callback_sf2(const fo_msgs::msg::FusionobjListForSF2::SharedPtr msg);
    
    void callback_gnss_pvt(const ublox_ubx_msgs::msg::UBXNavPVT::SharedPtr msg);
    void callback_gnss_heading(const ublox_ubx_msgs::msg::UBXNavRelPosNED::SharedPtr msg);
    void callback_gnss_cal_heading(const std_msgs::msg::Int32::SharedPtr msg);

    // callback func variable
    int cam_td_counter = 0;
    int cam_od_counter = 0;

    int lidar_td_counter = 0;

    int radar_tr_c_counter = 0;
    int radar_tr_f_counter = 0;
    int radar_tr_r_counter = 0;

    int gnss_rtk_counter = 0;
    int calculate_heading = 0;
    
    int sf2_counter = 0;
    int sf2_prev_counter = -1;   // 이전 timer tick에서 관찰한 sf2_counter (-1: 최초 tick 구분용)
    int sf2_stale_tick_count = 0; // sf2_counter가 연속으로 동일했던 timer tick 수
    
    // float to -bit signed func
    uint64_t float_to_bit_signed(double value, int bit);
    uint64_t float_to_bit_unsigned(double value, int bit);

    // decode용 func
    int64_t bit_to_signed(uint64_t value, int bit);
    uint64_t bit_to_unsigned(uint64_t value, int bit);

    // Cam2_OD 정적 객체(drum.txt, EPSG:4326) - 자차 GPS 기준 로컬좌표 변환 구조
    void loadStaticObjectGpsPoints(const std::string &file_path);
    void latLonToLocalXY(double lat, double lon, double ref_lat, double ref_lon, double ref_heading_deg, double &local_x, double &local_y);

    std::vector<std::pair<double, double>> static_object_gps_points_; // drum.txt (lat, lon)
    double ego_gps_latitude_ = 0.0;   // callback_gnss_pvt에서 갱신되는 자차 위도 [deg]
    double ego_gps_longitude_ = 0.0;  // callback_gnss_pvt에서 갱신되는 자차 경도 [deg]
    double ego_gps_heading_deg_ = 0.0; // callback_gnss_heading에서 갱신되는 자차 헤딩 [deg], 북쪽 0, 시계방향 증가

    // 20ms timer
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;

    int server_socket;
    struct sockaddr_in server_addr;


    /* =============== testBuff용 파라미터 (general) =============== */
    // general
    int    param_camera2_fail_flag_;
    int    param_camera2_alive_counter_;
    int    param_radar2_c_fail_flag_;
    int    param_radar2_c_alive_counter_;
    int    param_radar2_f_fail_flag_;
    int    param_radar2_f_alive_counter_;
    int    param_radar2_r_fail_flag_;
    int    param_radar2_r_alive_counter_;
    int    param_lidar2_fl_fail_flag_;
    int    param_lidar2_fl_alive_counter_;
    int    param_lidar2_fr_fail_flag_;
    int    param_lidar2_fr_alive_counter_;
    int    param_lidar2_r_fail_flag_;
    int    param_lidar2_r_alive_counter_;
    int    param_gnss_fail_flag_;
    int    param_gnss_alive_counter_;
    int    param_v2x_fail_flag_;
    int    param_v2x_alive_counter_;

    int    param_camera2_failure_state_;
    int    param_radar2_c_failure_state_;
    int    param_radar2_f_failure_state_;
    int    param_radar2_r_failure_state_;
    int    param_lidar2_fl_failure_state_;
    int    param_lidar2_fr_failure_state_;
    int    param_lidar2_r_failure_state_;
    int    param_gnss_failure_state_;
    int    param_v2x_failure_state_;

    int    param_sense_staute_rolling_counter_;
    int    param_SW_VehiCommS_;
    int    param_SW_SensProc_;
    int    param_SW_SensFusion_;
    int    param_SW_ADSMonitorS_;
    int    param_HW_ProcSense_;
    int    param_HW_ExtNPU_;
    int    param_HW_PMICS_;
    int    param_HW_EthPhySen_;
    int    param_HW_EthPhyCam_;
    int    param_HW_CCANS_;
    int    param_HW_IntCANS_;
    int    param_HW_DDR_;
    int    param_HW_eMMC_;
    int    param_VEH_Chassis_;
    int    param_DNN_IMG_;
    int    param_DNN_PCD_;

    /* =============== testBuff용 파라미터 (camera) =============== */
    // Camera LD
    // camera - lane a
    int    param_lane_mark_type_;
    int    param_lane_mark_quality_;
    double param_lane_mark_position_;
    double param_lane_mark_model_a_;
    // camera - lane b 
    double param_lane_mark_heading_angle_;
    double param_lane_mark_model_view_range_;
    int    param_lane_mark_model_view_range_availability_;
    double param_lane_mark_model_da_;
    // camera - lane additional data
    int    param_alive_counter_ld_;

    // Camera TD
    // camera - obstacle a
    int    param_alive_counter_td_a_;
    int    param_object_age_;
    double param_angle_rate_;
    double param_angle_left_;
    double param_angle_right_;
    int    param_motion_status_;
    int    param_object_lane_;
    int    param_cam2_obstacle_brake_lights_;
    // camera - obstacle b
    int    param_alive_counter_td_b_;
    double param_range_;
    int    param_object_vaildity_;
    double param_range_rate_;
    double param_cam2_obstacle_physical_width_;
    int    param_cam2_track_id_;
    int    param_object_type_;

    // Camera TL
    // camera - traffic light
    int    param_traffic_light_data_;
    int    param_traffic_light_valid_flag_;
    int    param_traffic_light_accuracy_;

    // Camera OD
    // camera - static object
    int    param_rolling_count_od_;
    int    param_camera2_static_obj_type_;
    int    param_static_object_status_;
    double param_static_object_pos_y_;
    double param_static_object_pos_x_;
    double param_static_object_pos2_y_;
    double param_static_object_pos2_x_;

    /* =============== testBuff용 파라미터 (lidar) =============== */
    // LiDAR TD
    // lidar track a
    int    param_track_status_;
    int    param_track_valid_lidar_;
    double param_track_pos_y_;
    double param_track_pos_x_;
    int    param_track_rolling_count_a_;
    // lidar track b
    int    param_lidar_track_id_;
    int    param_track_add_info_status_;
    double param_track_width_;
    double param_track_length_;
    int    param_track_rolling_count_b_;

    /* =============== testBuff용 파라미터 (radar) =============== */
    // radar
    int    param_radar2_track_id_;
    int    param_radar2_track_status_;
    double param_radar2_track_pos_y_;
    double param_radar2_track_pos_x_;
    int    param_track_valid_radar_;
    int    param_radar2_track_type_;
    double param_radar2_track_vel_y_;
    int    param_radar2_rolling_count_;
    double param_radar2_track_vel_x_;

    /* =============== testBuff용 파라미터 (sf2) =============== */
    // sf2 track1
    int    param_track_valid_sf2_;
    double param_sf2_track_pos_y_;
    double param_sf2_track_pos_x_;
    int    param_sf2_rolling_count_;
    int    param_sf2_track_type_;
    double param_sf2_track_vel_y_;
    double param_sf2_track_vel_x_;
    // sf2 track2
    int    param_sf2_track_id_;

    /* =============== testBuff용 파라미터 (gps) =============== */
    int    param_rtk_latitude_;
    int    param_rtk_longitude_;
    int    param_rtk_rolling_count_;
    int    param_rtk_heading_;
    int    param_rtk_fix_flag_;
};
