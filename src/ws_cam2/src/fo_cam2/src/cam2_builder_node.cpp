/* CPP */
#include <cstdio>
#include <chrono>
#include <thread>

/* ROS2 */
#include "rclcpp/rclcpp.hpp"
#include "yolo_msgs/msg/detection_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "fo_msgs/msg/cam2_ld.hpp"      // for subscriber - lane_detector
#include "fo_msgs/msg/cam2_tdtlod.hpp"  // for subscriber - detection_postprocessor
#include "fo_msgs/msg/cam2_data.hpp"    // for publisher

/* Utils */
// #include "opencv2/opencv.hpp"
#include "fo_cam2/fo_struct.hpp"


/* Variables */
// Cam2_Data buff_write = {};
// Cam2_Data buff_send = {};


class Cam2DataBuilder : public rclcpp::Node
{
public: 
  Cam2DataBuilder()
  : Node("Cam2DataBuilder")
  {
    RCLCPP_INFO(this->get_logger(), "Cam2DataBuilder init");

    this->declare_parameter<std::string>("input_topic_ld", "/cam2_ld");
    this->declare_parameter<std::string>("input_topic_tdtlod", "/cam2_tdtlod");
    this->declare_parameter<std::string>("output_topic", "/cam2_data");
    this->declare_parameter<int>("timer_period_ms", 20);

    std::string input_topic_tdtlod = this->get_parameter("input_topic_tdtlod").as_string(); 
    std::string input_topic_ld     = this->get_parameter("input_topic_ld").as_string(); 
    std::string output_topic       = this->get_parameter("output_topic").as_string(); 
    int timer_period_ms    = this->get_parameter("timer_period_ms").as_int(); 

    buff_ld = std::make_shared<fo_msgs::msg::Cam2LD>();
    buff_td = std::make_shared<fo_msgs::msg::Cam2TDTLOD>();
    buff_write = std::make_shared<fo_msgs::msg::Cam2Data>();
    buff_send = std::make_shared<fo_msgs::msg::Cam2Data>();
    
    sub_ld_     = this->create_subscription<fo_msgs::msg::Cam2LD>(
                  input_topic_ld, 
                  10, 
                  std::bind(&Cam2DataBuilder::callbackCam2LD, this, std::placeholders::_1)
                  );
    sub_tdtlod_ = this->create_subscription<fo_msgs::msg::Cam2TDTLOD>(
                  input_topic_tdtlod, 
                  10, 
                  std::bind(&Cam2DataBuilder::callbackCam2ODTDTL, this, std::placeholders::_1)
                  );
    pub_cam2_   = this->create_publisher<fo_msgs::msg::Cam2Data>(output_topic, 10); //TODO: fo_cam2 data msg 타입 정의 후 publisher 만들기
    timer_      = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms), 
                                          std::bind(&Cam2DataBuilder::callbackCam2DataPub, this)); // car data
                                          // std::bind(&Cam2DataBuilder::testPub, this));                // lab test
  }

  ~Cam2DataBuilder()
  {

  }

private:
  rclcpp::Subscription<fo_msgs::msg::Cam2LD>::SharedPtr sub_ld_;
  rclcpp::Subscription<fo_msgs::msg::Cam2TDTLOD>::SharedPtr sub_tdtlod_;
  rclcpp::Publisher<fo_msgs::msg::Cam2Data>::SharedPtr pub_cam2_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<fo_msgs::msg::Cam2LD> buff_ld;
  std::shared_ptr<fo_msgs::msg::Cam2TDTLOD> buff_td;
  std::shared_ptr<fo_msgs::msg::Cam2Data> buff_write, buff_send;
  
  std::mutex mtx_ld;
  std::mutex mtx_td;
  std::mutex mtx_;


  void callbackCam2LD(const fo_msgs::msg::Cam2LD::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "callbackCam2LD starts");
    
    {std::lock_guard<std::mutex> l_ld(mtx_ld);
      *buff_ld = *msg;  
    }
  }
  
  void callbackCam2ODTDTL(const fo_msgs::msg::Cam2TDTLOD::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "callbackCam2ODTDTL starts.");

    {std::lock_guard<std::mutex> l_ld(mtx_td);
      *buff_td = *msg;  
    }
  }
  
  void callbackCam2DataPub()
  {
    // RCLCPP_INFO(this->get_logger(), "callbackCam2DataPub starts.");
    
    { std::lock_guard<std::mutex> lock(mtx_);
      // *buff_write.ld = *buff_ld;
      // *buff_write.td = *buff_td->td;
      // *buff_write.tl = *buff_td->tl;
      // *buff_write.od = *buff_td->od;
      buff_write->ld = *buff_ld;  
      buff_write->td = buff_td->td;
      buff_write->tl = buff_td->tl;
      buff_write->od = buff_td->od;
    }

    { std::lock_guard<std::mutex> lock(mtx_);
        buff_write.swap(buff_send);
    }

    pub_cam2_->publish(*buff_send);
  }

  void testPub()
  {
      buff_write->ld = *buff_ld;  

      for (int i=0; i<CAM_TRACK_NUM; i++) {
        if (i%2==0) {
          buff_write->td.tda[i].rolling_counter = 0xff;
          buff_write->td.tda[i].object_age = 0xff;
          buff_write->td.tda[i].angle_rate = 0xff;
          buff_write->td.tda[i].angle_left = 0xff;
          buff_write->td.tda[i].angle_right = 0xff;
          buff_write->td.tda[i].motion_status = 0xff;
          buff_write->td.tda[i].object_lane = 0xff;
          buff_write->td.tda[i].cam2_obstacle_brake_lights = 0xff;
  
          buff_write->td.tdb[i].rolling_counter = 0xff;
          buff_write->td.tdb[i].range = 0xff;
          buff_write->td.tdb[i].object_vaildity = 0xff;
          buff_write->td.tdb[i].range_rate = 0xff;
          buff_write->td.tdb[i].cam2_obstacle_physical_width = 0xff;
          buff_write->td.tdb[i].cam2_track_id = 0xff;
          buff_write->td.tdb[i].object_type = 0xff;

          buff_write->od.sod[i].rolling_count_1         = 0xff;
          buff_write->od.sod[i].camera2_static_obj_type = 0xff;
          buff_write->od.sod[i].static_object_status    = 0xff;
          buff_write->od.sod[i].static_object_pos_y     = 0xff;
          buff_write->od.sod[i].static_object_pos_x     = 0xff;
          buff_write->od.sod[i].static_object_pos2_y    = 0xff;
          buff_write->od.sod[i].static_object_pos2_x    = 0xff;
        } else {
          buff_write->td.tda[i].rolling_counter = 0x55;
          buff_write->td.tda[i].object_age = 0x55;
          buff_write->td.tda[i].angle_rate = 0x55;
          buff_write->td.tda[i].angle_left = 0x55;
          buff_write->td.tda[i].angle_right = 0x55;
          buff_write->td.tda[i].motion_status = 0x55;
          buff_write->td.tda[i].object_lane = 0x55;
          buff_write->td.tda[i].cam2_obstacle_brake_lights = 0x55;
  
          buff_write->td.tdb[i].rolling_counter = 0x55;
          buff_write->td.tdb[i].range = 0x55;
          buff_write->td.tdb[i].object_vaildity = 0x55;
          buff_write->td.tdb[i].range_rate = 0x55;
          buff_write->td.tdb[i].cam2_obstacle_physical_width = 0x55;
          buff_write->td.tdb[i].cam2_track_id = 0x55;
          buff_write->td.tdb[i].object_type = 0x55;

          buff_write->od.sod[i].rolling_count_1         = 0x55;
          buff_write->od.sod[i].camera2_static_obj_type = 0x55;
          buff_write->od.sod[i].static_object_status    = 0x55;
          buff_write->od.sod[i].static_object_pos_y     = 0x55;
          buff_write->od.sod[i].static_object_pos_x     = 0x55;
          buff_write->od.sod[i].static_object_pos2_y    = 0x55;
          buff_write->od.sod[i].static_object_pos2_x    = 0x55;
        }
      }

      buff_write->tl.traffic_light_data = 3;
      buff_write->tl.traffic_light_valid_flag = 2;
      buff_write->tl.traffic_light_accuracy = 1;

    { std::lock_guard<std::mutex> lock(mtx_);
        buff_write.swap(buff_send);
    }
    pub_cam2_->publish(*buff_send);
  }

};



int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Cam2DataBuilder>());
  rclcpp::shutdown();

  return 0;
}
