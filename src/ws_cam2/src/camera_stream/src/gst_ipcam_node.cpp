#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cstdlib> // For std::system
#include <sensor_msgs/msg/image.hpp>
#include <fo_msgs/msg/status_camera.hpp>
#include "camera_stream/GstAppsink.hpp"
// #include <yaml-cpp/yaml.h>
#include "common.hpp"

using namespace std::chrono_literals;


// std::string getPipelineFromYaml(const std::string& yaml_path)
// {
//     // std::cout << "received yaml_path:" << yaml_path << std::endl;
//     YAML::Node root = YAML::LoadFile(yaml_path);
//     const auto stream = root["camera_stream"];
//     if (!stream) throw std::runtime_error("missing 'stream' section");

//     std::string pipeline, url;
//     if (stream["gst_pipeline"]) {
//         pipeline = stream["gst_pipeline"].as<std::string>();
//     } else {
//         throw std::runtime_error("missing 'gst_pipeline'");
//     }
//     if (stream["rtsp_url"]) {
//         url = stream["rtsp_url"].as<std::string>();
//     } else {
//         throw std::runtime_error("missing 'rtsp_url'");
//     }

//     const std::string placeholder = "{url}";
//     size_t pos = pipeline.find(placeholder);

//     if (pos == std::string::npos) {
//         throw std::runtime_error("The 'gst_pipeline' string must contain the '{url}' placeholder.");
//     }
//     pipeline.replace(pos, placeholder.length(), url);

//     return pipeline;
// }


class gstImagePublisher : public rclcpp::Node
{
public:
    gstImagePublisher()
    : Node("gstreamer_image_publihser")
    {
        std::string yp = "/home/wise/adsp-test3/ws_cam2/src/camera_stream/yaml/cam_parameters_220.yaml";
        std::string yaml_path = this->declare_parameter<std::string>("yaml_path",yp);
        // pipeline_ = getPipelineFromYaml(yaml_path);
        calib_ = getCameraConfigFromYaml(yaml_path);
        RCLCPP_INFO(this->get_logger(), "Camera configration parsed");

        // std::cout << "Parsed pipeline:" << pipeline_ << std::endl;
        status_topic_ = this->declare_parameter<std::string>("status_topic", "/camera_general");  
        image_topic_ = this->declare_parameter<std::string>("image_topic", "/image_raw");
        frame_id_    = this->declare_parameter<std::string>("frame_id", "camera");
        const std::string configured_host = this->declare_parameter<std::string>("host", "");
        host_ = !calib_.host.empty() ? calib_.host : configured_host;
        timeout_ms_          = this->declare_parameter<int>("timeout_ms", 200);         // appsink pull timeout
        double expected_fps = this->declare_parameter<double>("expected_fps", 25.0);

        if (!calib_.host.empty()) {
            RCLCPP_INFO(this->get_logger(), "Using host '%s' parsed from YAML rtsp_url.", host_.c_str());
        } else if (!configured_host.empty()) {
            RCLCPP_INFO(this->get_logger(), "Using fallback host '%s' from ROS parameter.", host_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "No host could be derived from YAML rtsp_url or ROS parameter. Ping-based status checks are disabled.");
        }

        // rclcpp::QoS qos(rclcpp::SensorDataQoS().keep_last(1));
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1))
                        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        pub_img_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
        pub_status_ = create_publisher<fo_msgs::msg::StatusCamera>(status_topic_, 10);

        // if (!gst_.open(pipeline_)) {
        if (!gst_.open(calib_.gst_pipeline)) {
            RCLCPP_FATAL(get_logger(), "Failed to open GStreamer pipeline.");
            throw std::runtime_error("gst open failed.");
        }

        // timer_ = create_wall_timer(0ms, std::bind(&gstImagePublisher::framePub, this));
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / expected_fps));
        timer_ = create_wall_timer(timer_period, std::bind(&gstImagePublisher::framePub, this));

        // RCLCPP_INFO(get_logger(), "gstreamer_image_publihser starts. \npipeline:\n %s", pipeline_.c_str());
    }

    ~gstImagePublisher() override {
        gst_.close();
    }

private:
    CameraConfig calib_;
    GstAppsink gst_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
    rclcpp::Publisher<fo_msgs::msg::StatusCamera>::SharedPtr pub_status_;  // StatusCamera publisher

    rclcpp::TimerBase::SharedPtr timer_;
    std::string pipeline_, host_; 
    std::string image_topic_, frame_id_, status_topic_;
    int timeout_ms_;

    int fail_flag = 0;
    int alive_cnt = 0;
    int failure_state = 0x0;

    void framePub() {
        cv::Mat frame;
        

        bool ok = gst_.read(frame, timeout_ms_);
        bool ping = true;
        if (!host_.empty()) {
            ping = isHostReachable(host_);
        }

        // if (!ok) { // camera input failed
        //     if (gst_.is_disconnected()) { // network disconnection
        //         // std::cerr << "[ERROR] Camera disconnected/bus error!" << std::endl;
        //         fail_flag = 1;
        //         failure_state = 0xf; //nosig

        //         std::cout << "fail_flag: " << static_cast<int>(fail_flag) << " alive_cnt: " << static_cast<int>(alive_cnt) << " failure_state: " << static_cast<int>(failure_state) << std::endl;
                

        //     } else { // delayed input
        //         // std::cerr << "[ERROR] Frame read failed (timeout or empty)" << std::endl;
        //         fail_flag = 1;
        //         failure_state = 0x1; //delay
            
        //         std::cout << "fail_flag: " << static_cast<int>(fail_flag) << " alive_cnt: " << static_cast<int>(alive_cnt) << " failure_state: " << static_cast<int>(failure_state) << std::endl;
            
        //     }

        if (ping==false) { // network disconnection
            // std::cerr << "[ERROR] Camera disconnected/bus error!" << std::endl;
            fail_flag = 1;
            failure_state = 0xf; //nosig

            std::cout << "fail_flag: " << static_cast<int>(fail_flag) << " alive_cnt: " << static_cast<int>(alive_cnt) << " failure_state: " << static_cast<int>(failure_state) << std::endl;
        
        } else { // camera input successed
            fail_flag = 0;
            alive_cnt = (alive_cnt + 1) & 0xFF;
            failure_state = 0x0;
            // std::cout << "fail_flag: " << static_cast<int>(fail_flag) << " alive_cnt: " << static_cast<int>(alive_cnt) << " failure_state: " << static_cast<int>(failure_state) << std::endl;
        }


        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), 
                                        sensor_msgs::image_encodings::BGR8, 
                                        frame).toImageMsg();
        img_msg->header.stamp = this->get_clock()->now();
        img_msg->header.frame_id = frame_id_;
        pub_img_->publish(std::move(*img_msg));

        // StatusCamera 메시지 생성exit_code 및 publish
        auto status_msg = fo_msgs::msg::StatusCamera();
        status_msg.fail_flag = fail_flag;
        status_msg.rolling_counter = alive_cnt;
        status_msg.failure_state = failure_state;
        pub_status_->publish(status_msg);
    }

    bool isHostReachable(const std::string& host) {
    // Construct the ping command
    // -c 1: Send only one packet
    // -W 1: Wait 1 second for a response
    std::string command = "ping -c 1 -W 1 " + host + " > /dev/null 2>&1";
    int exit_code = std::system(command.c_str());

    // The command returns 0 on success (host is reachable)
    return (exit_code == 0);
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gstImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
