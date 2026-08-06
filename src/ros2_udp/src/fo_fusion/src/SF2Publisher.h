#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


#include "fo_msgs/msg/lidarobj_lists_for_sf2.hpp"
#include "fo_msgs/msg/radar_tr_array.hpp"

#include "fo_msgs/msg/cam2_data_for_sf2.hpp"
#include "fo_msgs/msg/obj2_sf.hpp"

#include "fo_msgs/msg/fusion_for_sf2.hpp"
#include "fo_msgs/msg/fusionobj_list_for_sf2.hpp"
#include "fo_msgs/msg/status_camera.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <geometry_msgs/msg/pose.hpp>


#pragma pack(push, 1)

struct SFObjs 
{
    uint8_t  track_valid;
    uint8_t  sf2_track_status;
    float    sf2_track_pos_y;
    float    sf2_track_pos_x;
    uint8_t  sf2_rolling_count;
    uint8_t  sf2_track_type;
    float    sf2_track_vel_y;
    float    sf2_track_vel_x;
    uint8_t  used_track_number;
    uint8_t  sf2_add_info_status;
    uint8_t  sf2_track_id;
    uint8_t  used_cam2_track_id_first;
    uint8_t  used_cam2_track_id_second;
    uint8_t  used_lidar2_track_id_first;
    uint8_t  used_lidar2_track_id_second;
    uint8_t  used_radar2_track_id_first;
    uint8_t  used_radar2_track_id_second;

    // 추가: 앵커 박스 크기
    float    length;
    float    width;
    float    height;
    float    confidence;

}; 

#pragma pack(pop)

class SF2Publisher : public rclcpp::Node
{
public:
    SF2Publisher();
    virtual ~SF2Publisher() override;

    bool init();


private:

    
    // subscription
    rclcpp::Subscription<fo_msgs::msg::LidarobjListsForSF2>::SharedPtr lidar_obs_sub_;
    rclcpp::Subscription<fo_msgs::msg::Cam2DataForSF2>::SharedPtr camera_obs_sub_;
    rclcpp::Subscription<fo_msgs::msg::RadarTrArray>::SharedPtr radar_obs_sub_;

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr lidar_general_sub_;
    rclcpp::Subscription<fo_msgs::msg::StatusCamera>::SharedPtr camera_general_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr radar_general_c_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr radar_general_f_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr radar_general_r_sub_;


    //pub
    rclcpp::Publisher<fo_msgs::msg::FusionobjListForSF2>::SharedPtr pub_fusion_objs_;

    // pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_detector/markers", 10);
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;



    // callback func
    void callback_lidar_general(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void callback_camera_general(const fo_msgs::msg::StatusCamera::SharedPtr msg);
    void callback_radar_c_general(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    // void callback_radar_f_general(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    // void callback_radar_r_general(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

    void callback_lidar_obj(const fo_msgs::msg::LidarobjListsForSF2::SharedPtr msg);
    void callback_camera_obj(const fo_msgs::msg::Cam2DataForSF2::SharedPtr msg);
    void callback_radar_obj(const fo_msgs::msg::RadarTrArray::SharedPtr msg);
  
    void fusion();
    void confidence();
    float computeIoU(SFObjs &lidar_obj, SFObjs &camera_obj);

    std::string main_sensor = "camera";

    uint8_t lidar2_fl_fail_flag;
    uint8_t lidar2_fl_alive_counter;
    uint8_t lidar2_fr_fail_flag;
    uint8_t lidar2_fr_alive_counter;
    uint8_t lidar2_r_fail_flag;
    uint8_t lidar2_r_alive_counter;

    uint8_t camera_fail_flag;
    uint8_t camera_alive_counter;
    uint8_t camera_failure_state;

    uint8_t radar2_c_fail_flag;
    uint8_t radar2_c_alive_counter;
    uint8_t radar2_c_failure_state;
    
    float anchor_bus_l = 7.0;
    float anchor_bus_w = 2.0;
    float anchor_bus_h = 4.0;

    float anchor_bicycle_l = 2.0;
    float anchor_bicycle_w = 1.0;
    float anchor_bicycle_h = 1.6;

    float anchor_truck_l = 5.0;
    float anchor_truck_w = 2.0;
    float anchor_truck_h = 3.0;

    float anchor_car_l = 3.6;
    float anchor_car_w = 1.4;
    float anchor_car_h = 1.9;

    float anchor_person_l = 0.7;
    float anchor_person_w = 0.7;
    float anchor_person_h = 1.7;

    float radar_anchor_w = 1.0;
    float radar_anchor_l = 1.0;

    // SFObjs lidar_obj_data_;
    // SFObjs cam_obj_data_;
    
    std::vector<SFObjs> lidar_obj_data_list_;
    std::vector<SFObjs> cam_obj_data_list_;
    std::vector<SFObjs> radar_obj_data_list_;

    std::vector<SFObjs> fusion_obj_data_list_;

    int lidar_roilling = 0;
    int cam_roilling = 0;
    int radar_roilling = 0;
    
    bool cam_check = false;
    bool lidar_check = false;
    bool radar_check = false;

    float conf_base = 0.9f;
    float conf_base_origin = 0.9f;

    float gojang = 1 ;
  

};
