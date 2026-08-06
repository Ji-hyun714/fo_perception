#include <iostream>
#include <cstring>
#include <mutex>

#include "SF2Publisher.h"
#include <cstdlib> 
#include <ctime>


SF2Publisher::SF2Publisher()
: Node("sf_node")
{
    RCLCPP_INFO(this->get_logger(), "SF2Publisher Node started.");
}


SF2Publisher::~SF2Publisher(){
    // 필요한 자원 정리
}

bool SF2Publisher::init(){

    lidar_general_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/lidar2_general", 10,
        std::bind(&SF2Publisher::callback_lidar_general, this, std::placeholders::_1));

    radar_general_c_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/radar_general_c", 10,
        std::bind(&SF2Publisher::callback_radar_c_general, this, std::placeholders::_1));

    camera_general_sub_ = this->create_subscription<fo_msgs::msg::StatusCamera>(
            "/camera/general", 10,
            std::bind(&SF2Publisher::callback_camera_general, this, std::placeholders::_1));

    // radar_general_f_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
    //     "/radar_general_f", 10,
    //     std::bind(&SF2Publisher::callback_radar_f_general, this, std::placeholders::_1));

    // radar_general_r_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
    //     "/radar_general_r", 10,
    //     std::bind(&SF2Publisher::callback_radar_r_general, this, std::placeholders::_1));

    lidar_obs_sub_ = this->create_subscription<fo_msgs::msg::LidarobjListsForSF2>(
        "/lidar/sf_objs", 10,
        std::bind(&SF2Publisher::callback_lidar_obj, this, std::placeholders::_1));

    camera_obs_sub_ = this->create_subscription<fo_msgs::msg::Cam2DataForSF2>(
        "/camera/sf_objs", 10,
        std::bind(&SF2Publisher::callback_camera_obj, this, std::placeholders::_1));

    //radar_obs_sub_ = this->create_subscription<fo_msgs::msg::LidarobjListsForSF2>(
    //        "/radar/sf_objs", 10,
    //        std::bind(&SF2Publisher::callback_radar_obj, this, std::placeholders::_1));


    pub_fusion_objs_= this->create_publisher<fo_msgs::msg::FusionobjListForSF2>("/fusion/sf_objs", 10);

    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("fusion/markers", 10);

    return true;
}

void SF2Publisher::callback_lidar_general(const std_msgs::msg::String::SharedPtr msg){
    const std::string& gen_data = msg->data;
    lidar2_fl_fail_flag = static_cast<uint8_t>(std::stoi(gen_data.substr(0, 3)));
    lidar2_fl_alive_counter = static_cast<uint8_t>(std::stoi(gen_data.substr(3, 3)));
    lidar2_fr_fail_flag = static_cast<uint8_t>(std::stoi(gen_data.substr(6, 3)));
    lidar2_fr_alive_counter = static_cast<uint8_t>(std::stoi(gen_data.substr(9, 3)));
    lidar2_r_fail_flag = static_cast<uint8_t>(std::stoi(gen_data.substr(12, 3)));
    lidar2_r_alive_counter = static_cast<uint8_t>(std::stoi(gen_data.substr(15, 3)));


    //  conf_base = conf_base_origin - (0.5 *lidar2_fl_fail_flag) -(0.5 * lidar2_fr_fail_flag) -(0.5 * lidar2_r_fail_flag);
        
}

void SF2Publisher::callback_camera_general(const fo_msgs::msg::StatusCamera::SharedPtr msg) {
    // std::lock_guard<std::mutex> lock(buffer_mutex);
    // int idx = write_idx_.load();

    // buffer_[idx].general_sensor2_status.camera2_fail_flag = msg->fail_flag;
    // buffer_[idx].general_sensor2_status.camera2_alive_counter = msg->rolling_counter;
    // buffer_[idx].general_sensor2_status.camera2_failure_state = msg->failure_state;

    camera_fail_flag = static_cast<uint8_t>(msg->fail_flag);
    camera_alive_counter = static_cast<uint8_t>(msg->rolling_counter);
    camera_failure_state = static_cast<uint8_t>(msg->failure_state);


    // std::cout << "camera2_fail_flag: " << buffer_[idx].general_sensor2_status.camera2_fail_flag << std::endl;
    // std::cout << "camera2_alive_counter: " << buffer_[idx].general_sensor2_status.camera2_alive_counter << std::endl;
    // std::cout << "camera2_failure_state: " << buffer_[idx].general_sensor2_status.camera2_failure_state << std::endl;

}

// callback func (radar)
void SF2Publisher::callback_radar_c_general(const std_msgs::msg::UInt8MultiArray::SharedPtr msg){
    radar2_c_fail_flag = static_cast<uint8_t>(msg->data[0]);
    radar2_c_alive_counter = static_cast<uint8_t>(msg->data[1]);
    radar2_c_failure_state = static_cast<uint8_t>(msg->data[2]);
}

// void SF2Publisher::callback_radar_f_general(const std_msgs::msg::UInt8MultiArray::SharedPtr msg){
//     radar2_f_fail_flag = static_cast<uint8_t>(msg->data[0]);
//     radar2_f_alive_counter = static_cast<uint8_t>(msg->data[1]);
//     radar2_f_failure_state = static_cast<uint8_t>(msg->data[2]);
// }

// void SF2Publisher::callback_radar_r_general(const std_msgs::msg::UInt8MultiArray::SharedPtr msg){
//     radar2_r_fail_flag = static_cast<uint8_t>(msg->data[0]);
//     radar2_r_alive_counter = static_cast<uint8_t>(msg->data[1]);
//     radar2_r_failure_state = static_cast<uint8_t>(msg->data[2]);
// }


void SF2Publisher::callback_lidar_obj(const fo_msgs::msg::LidarobjListsForSF2::SharedPtr msg){
    // std::cout << " callback_lidar_obj  " <<  std::endl;
    lidar_obj_data_list_ = {};
    int li = 0;
    for (auto &lidar_obj : msg->objs) {
        // if(lidar_obj.pos_y == 0 && lidar_obj.pos_x == 0){
        //     continue;
        // }
        SFObjs lidar_obj_data_  ;
        
        lidar_obj_data_.track_valid = 1;
        lidar_obj_data_.sf2_track_status = 1;
        lidar_obj_data_.sf2_track_pos_y = lidar_obj.pos_y;
        lidar_obj_data_.sf2_track_pos_x = lidar_obj.pos_x;
        lidar_obj_data_.sf2_rolling_count = lidar_roilling;
        lidar_obj_data_.sf2_track_type = lidar_obj.object_type;
        lidar_obj_data_.sf2_track_vel_y = lidar_obj.velocity_y;
        lidar_obj_data_.sf2_track_vel_x = lidar_obj.velocity_x;
        lidar_obj_data_.sf2_add_info_status = lidar_obj.velocity_x;
        lidar_obj_data_.sf2_track_id = li;

        lidar_obj_data_.used_lidar2_track_id_first = li;
        lidar_obj_data_.used_lidar2_track_id_second = li;

        lidar_obj_data_.length = lidar_obj.length;
        lidar_obj_data_.width = lidar_obj.width;
        lidar_obj_data_.height = 1.0;
        lidar_obj_data_.confidence = conf_base + static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 0.1f;

        lidar_obj_data_list_.push_back(lidar_obj_data_);
        li +=1;
    }

    lidar_roilling += 1;
    if(lidar_roilling == 4){
        lidar_roilling = 0;
    }

    lidar_check = true;
    if (main_sensor == "lidar" && cam_check){
        SF2Publisher::fusion();
    }


}

void SF2Publisher::callback_camera_obj(const fo_msgs::msg::Cam2DataForSF2::SharedPtr msg){
    // std::cout << " callback_camera_obj  " <<  std::endl;
    cam_obj_data_list_ = {};
    
    for (auto &camera_obj : msg->objs_td) {
        // if(camera_obj.pos_y == 0 && camera_obj.pos_x == 0){
        //     continue;
        // }
        SFObjs cam_obj_data_;

        //

        // if(camera_obj.object_type == 10 ){
        //     cam_obj_data_.length = anchor_bus_l;
        //     cam_obj_data_.width = anchor_bus_w;

        // }

        // else if(camera_obj.object_type == 11 ){
        //     cam_obj_data_.length = anchor_bicycle_l;
        //     cam_obj_data_.width = anchor_bicycle_w;
 
        // }

        // else if(camera_obj.object_type == 12 ){
        //     cam_obj_data_.length = anchor_truck_l;
        //     cam_obj_data_.width = anchor_truck_w;

        // }

        // else if(camera_obj.object_type == 13 ){
        //     cam_obj_data_.length = anchor_car_l;
        //     cam_obj_data_.width = anchor_car_w;

        // }

        // else if(camera_obj.object_type == 14 ){
        //     cam_obj_data_.length = anchor_person_l;
        //     cam_obj_data_.width = anchor_person_w;

        // }

        if(camera_obj.object_type == 1 ){
            cam_obj_data_.length = anchor_car_l;
            cam_obj_data_.width = anchor_car_w;

        }


        if(camera_obj.object_type == 2 ){
            cam_obj_data_.length = anchor_bus_l;
            cam_obj_data_.width = anchor_bus_w;

        }

        if(camera_obj.object_type == 3 ){
            cam_obj_data_.length = anchor_bicycle_l;
            cam_obj_data_.width = anchor_bicycle_w;

        }
        
        if(camera_obj.object_type == 4 ){
            cam_obj_data_.length = anchor_person_l;
            cam_obj_data_.width = anchor_person_w;

        }


        // if(camera_obj.object_type == 7 ){
        //     cam_obj_data_.length = anchor_truck_l;
        //     cam_obj_data_.width = anchor_truck_w;

        // }

        cam_obj_data_.height = 1.0;

        cam_obj_data_.track_valid = 1;
        cam_obj_data_.sf2_track_status = 1;
        cam_obj_data_.sf2_track_pos_y = camera_obj.pos_y;
        cam_obj_data_.sf2_track_pos_x = camera_obj.pos_x + cam_obj_data_.length/2;
        cam_obj_data_.sf2_rolling_count = cam_roilling;
        cam_obj_data_.sf2_track_type = camera_obj.object_type;
        cam_obj_data_.sf2_track_vel_y = 0.0;
        cam_obj_data_.sf2_track_vel_x = 0.0;
        cam_obj_data_.used_track_number = 0;
        cam_obj_data_.sf2_add_info_status = 1;
        cam_obj_data_.sf2_track_id = camera_obj.cam2_track_id;
        cam_obj_data_.used_cam2_track_id_first = camera_obj.cam2_track_id;
        cam_obj_data_.used_cam2_track_id_second = camera_obj.cam2_track_id;


        // std::cout << " camera_obj.pos_y : " <<  camera_obj.pos_y << std::endl;
        // std::cout << " camera_obj.pos_x : " <<  camera_obj.pos_x << std::endl;
        // std::cout << " cam_roilling : " <<  static_cast<int>(cam_roilling) << std::endl;
        // std::cout << " camera_obj.object_type : " <<  static_cast<int>(camera_obj.object_type) << std::endl;
        // std::cout << " camera_obj.cam2_track_id : " <<  static_cast<int>(camera_obj.cam2_track_id) << std::endl;
        // std::cout << " camera_obj.cam2_track_id : " <<  static_cast<int>(camera_obj.cam2_track_id) << std::endl;

        // std::cout << " cam_obj_data_.track_valid : " <<  static_cast<int>(cam_obj_data_.track_valid) << std::endl;
        // std::cout << " cam_obj_data_.sf2_track_status : " <<  static_cast<int>(cam_obj_data_.sf2_track_status) << std::endl;
        // std::cout << " cam_obj_data_.sf2_track_pos_y : " <<  static_cast<int>(cam_obj_data_.sf2_track_pos_y) << std::endl;
        // std::cout << " cam_obj_data_.sf2_track_pos_x : " <<  static_cast<int>(cam_obj_data_.sf2_track_pos_x) << std::endl;
        // std::cout << " cam_obj_data_.sf2_rolling_count : " <<  static_cast<int>(cam_obj_data_.sf2_rolling_count) << std::endl;
        // std::cout << " cam_obj_data_.sf2_track_id : " <<  static_cast<int>(cam_obj_data_.sf2_track_id) << std::endl;
        // std::cout << " cam_obj_data_.used_cam2_track_id_first : " <<  static_cast<int>(cam_obj_data_.used_cam2_track_id_first) << std::endl;
        // std::cout << " cam_obj_data_.used_cam2_track_id_second : " <<  static_cast<int>(cam_obj_data_.used_cam2_track_id_second) << std::endl;


        cam_obj_data_.confidence = conf_base + static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 0.1f;
        
        cam_obj_data_list_.push_back(cam_obj_data_);

    }
    cam_roilling +=1;
    if(cam_roilling == 4){
        cam_roilling = 0;
    }
    cam_check = true;

    if (main_sensor == "camera" && lidar_check){
        SF2Publisher::fusion();
    }
}

//void SF2Publisher::callback_radar_obj(const fo_msgs::msg::LidarobjListsForSF2::SharedPtr msg){
//
//}



float SF2Publisher::computeIoU(SFObjs &lidar_obj, SFObjs &camera_obj){
    // 2D IoU 계산 (x, y, w, h 기반)
    float x1 = std::max(lidar_obj.sf2_track_pos_x - lidar_obj.width/2,  camera_obj.sf2_track_pos_x - camera_obj.width/2);
    float y1 = std::max(lidar_obj.sf2_track_pos_y - lidar_obj.length/2, camera_obj.sf2_track_pos_y - camera_obj.length/2);
    float x2 = std::min(lidar_obj.sf2_track_pos_x + lidar_obj.width/2,  camera_obj.sf2_track_pos_x + camera_obj.width/2);
    float y2 = std::min(lidar_obj.sf2_track_pos_y + lidar_obj.length/2, camera_obj.sf2_track_pos_y + camera_obj.length/2);

    float interArea = std::max(0.0f, x2-x1) * std::max(0.0f, y2-y1);
    float unionArea = lidar_obj.width*lidar_obj.length + camera_obj.width*camera_obj.length - interArea;
    return (unionArea > 0) ? interArea/unionArea : 0.0f;
}


void SF2Publisher::fusion(){
    // std::cout << " fusion  " <<  std::endl;
    fusion_obj_data_list_ ={};

    if (main_sensor == "camera"){
        
        for (auto &cam_obj : cam_obj_data_list_) {
        //for (auto &lidar_obj : lidar_obj_data_list_) {
            for (auto &lidar_obj : lidar_obj_data_list_) {
            //for (auto &cam_obj : cam_obj_data_list_) {

                if(lidar_obj.confidence == 0.0){
                    continue;
                }
                float iou = SF2Publisher::computeIoU(lidar_obj,cam_obj);
                if(iou >=0.1){
                    lidar_obj.confidence = 0.0;
                    cam_obj.used_lidar2_track_id_first = lidar_obj.used_lidar2_track_id_first;
                    cam_obj.used_lidar2_track_id_second = lidar_obj.used_lidar2_track_id_second;
                    break;
                }
            }
            cam_obj.confidence = conf_base + static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 0.1f;

            fusion_obj_data_list_.push_back(cam_obj);
        }
        // for (auto &lidar_obj : lidar_obj_data_list_) {
        //     if(lidar_obj.confidence > 0.0){
        //         lidar_obj.confidence = conf_base - 0.1f + static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 0.1f;

        //         fusion_obj_data_list_.push_back(lidar_obj);
        //     }
        // }
    }
    else if (main_sensor == "lidar"){
        for (auto &lidar_obj : lidar_obj_data_list_) {
            for (auto &cam_obj : cam_obj_data_list_) {

                if(lidar_obj.confidence == 0.0){
                    continue;
                }
                float iou = SF2Publisher::computeIoU(lidar_obj,cam_obj);
                if(iou >=0.5){
                    cam_obj.confidence = 0.0;
                    lidar_obj.used_cam2_track_id_first = cam_obj.used_cam2_track_id_first;
                    lidar_obj.used_cam2_track_id_second = cam_obj.used_cam2_track_id_second;
                    lidar_obj.sf2_track_type = cam_obj.sf2_track_type;
                    break;
                }
            }
            lidar_obj.confidence = conf_base + static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 0.1f;

            fusion_obj_data_list_.push_back(lidar_obj);
        }
        for (auto &cam_obj : cam_obj_data_list_) {
            if(cam_obj.confidence > 0.0){
                cam_obj.confidence = conf_base - 0.1f + static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 0.1f;

                fusion_obj_data_list_.push_back(cam_obj);
            }
        }
    }
    // pub_fusion_objs_->publish(fusion_obj_data_list_);

    fo_msgs::msg::FusionobjListForSF2 fusion_msg;
    fusion_msg.header.stamp = this->get_clock()->now();
    fusion_msg.header.frame_id = "vehicle";

     //marker start

    visualization_msgs::msg::MarkerArray markers;

    // marker delete
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);
    

    
    int cluster_id = 0;

    // size_t idx = 0;
    for (auto &obj : fusion_obj_data_list_) {

        if(obj.sf2_track_pos_x == 0 && obj.sf2_track_pos_y == 0){
            continue;
        }


        fo_msgs::msg::FusionForSF2 msg_obj;
        msg_obj.track_valid              = obj.track_valid;
        msg_obj.sf2_track_status         = obj.sf2_track_status;
        msg_obj.sf2_track_pos_x          = obj.sf2_track_pos_x;
        msg_obj.sf2_track_pos_y          = obj.sf2_track_pos_y;
        msg_obj.sf2_track_vel_x          = obj.sf2_track_vel_x;
        msg_obj.sf2_track_vel_y          = obj.sf2_track_vel_y;
        msg_obj.sf2_track_type           = obj.sf2_track_type;
        msg_obj.sf2_track_id             = obj.sf2_track_id;
        msg_obj.sf2_rolling_count        = obj.sf2_rolling_count;
        msg_obj.confidence               = obj.confidence;

        // msg_obj.length                   = obj.length;
        // msg_obj.width                    = obj.width;
        // msg_obj.height                   = obj.height;

        msg_obj.used_lidar2_track_id_first = obj.used_lidar2_track_id_first;
        msg_obj.used_lidar2_track_id_second = obj.used_lidar2_track_id_second;
        msg_obj.used_cam2_track_id_first   = obj.used_cam2_track_id_first;
        msg_obj.used_cam2_track_id_second  = obj.used_cam2_track_id_second;

        fusion_msg.objs.push_back(msg_obj);
        // fusion_msg.objs[idx++] = msg_obj;

        visualization_msgs::msg::Marker marker;
        marker.header = fusion_msg.header;
        marker.ns = "obstacles";
        marker.id = obj.sf2_track_id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.pose.position.x = obj.sf2_track_pos_x;   // 중심 x
        marker.pose.position.y = obj.sf2_track_pos_y;   // 중심 y
        marker.pose.position.z = 0;   // 중심 z
        marker.pose.orientation.w = 1.0;
        marker.scale.x = obj.length;          // length
        marker.scale.y = obj.width;          // width
        marker.scale.z = obj.height;          // height
        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markers.markers.push_back(marker);

    }

    // ROS2 Publisher로 메시지 publish
    pub_fusion_objs_->publish(fusion_msg);

    cam_check = false;
    lidar_check = false;
    
    // gojang = (1.0f +1.0f +1.0f +static_cast<float>(camera_fail_flag) + static_cast<float>(radar2_c_fail_flag)+2.0f) / 7.0f * 100.0f;
   
    pub_markers_->publish(markers);

}


//----------------------------------------------

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SF2Publisher>();
    std::srand(std::time(nullptr));

    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize SF2Publisher");
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
//----------------------------------------------