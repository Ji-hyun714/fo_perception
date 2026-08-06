#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <gst/gst.h>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>

class GStreamerRTSPStreamer : public rclcpp::Node
{
public:
    GStreamerRTSPStreamer() : Node("gstreamer_rtsp_streamer"), is_running_(true)
    {
        RCLCPP_INFO(this->get_logger(), "Node init");


        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw2", 10);

        // GStreamer 파이프라인 설정
        // std::string rtsp_url = "rtsp://admin:1234@192.168.0.223:554/stream protocols=udp latency=0";        // lab test
        std::string rtsp_url = "rtsp://admin:tlJwpbo6@192.168.0.25:554/stream protocols=udp latency=0";  // car test
        // std::string pipeline_str = "rtspsrc location=" + rtsp_url + " ! rtph264depay ! avdec_h264 ! videoconvert ! appsink";
        std::string pipeline_str = "rtspsrc location=" + rtsp_url + " ! rtph264depay ! avdec_h264 ! videoconvert ! appsink";

        cap_ = cv::VideoCapture(pipeline_str, cv::CAP_GSTREAMER);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer pipeline");
            return;
        }

        capture_thread_ = std::thread(&GStreamerRTSPStreamer::capture_frames, this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&GStreamerRTSPStreamer::timer_callback, this));
    }

    ~GStreamerRTSPStreamer()
    {
        is_running_ = false;
        if (capture_thread_.joinable())
            capture_thread_.join();
    }

private:
    void capture_frames()
    {
        while (is_running_)
        {
            cv::Mat frame;
            if (cap_.read(frame))
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = frame.clone(); // 최신 프레임 저장
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to read frame from GStreamer pipeline.");
            }
        }
    }

    void timer_callback()
    {
        auto start_time = std::chrono::steady_clock::now(); // 프레임 처리 시작 시간

        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (latest_frame_.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No frame available for publishing.");
                return;
            }
            frame = latest_frame_.clone();
        }

        cv::resize(frame, frame, cv::Size(1280, 720));

        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        msg->header.stamp = this->get_clock()->now();

        // 메시지 발행
        publisher_->publish(*msg);
        // RCLCPP_INFO(this->get_logger(), "Publishing video frame");

        frame_times_.push_back(start_time);
        if (frame_times_.size() > 30)
        {
            frame_times_.erase(frame_times_.begin()); // 가장 오래된 프레임 제거

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                                frame_times_.back() - frame_times_.front()).count();
            if (duration > 0)
            {
                double fps = 30000.0 / duration; // 30프레임 기준 FPS 계산
                RCLCPP_INFO(this->get_logger(), "FPS: %.2f", fps);
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    std::vector<std::chrono::steady_clock::time_point> frame_times_;

    std::thread capture_thread_;    
    std::mutex frame_mutex_;        
    cv::Mat latest_frame_;          
    bool is_running_;               
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GStreamerRTSPStreamer>());
    rclcpp::shutdown();
    return 0;
}