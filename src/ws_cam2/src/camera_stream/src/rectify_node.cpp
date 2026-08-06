#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include <chrono>
#include <vector>
#include <string>

#include "common.hpp"

// #include <yaml-cpp/yaml.h>

// struct CameraConfig {
//     uint8_t model;       // camera model, 0: pinhole, 1: fisheye
//     cv::Mat K;           // 3x3 camera matrix
//     cv::Mat D;           // 1x5(pinhole)|1x4(fisheye) distortion coefficients
//     cv::Mat R;           // 3x3 rotation matrix
//     cv::Mat t;           // 3x1 translation vector
//     cv::Size image_size;
//     std::string rtsp_url;
//     std::string gst_pipeline;
// };

// CameraConfig getPipelineFromYaml(const std::string& yaml_path)
// {
//     CameraConfig calib;

//     YAML::Node root = YAML::LoadFile(yaml_path);

//     /* 
//         calib 
//     */    
//     if (!root["calib"]) {
//         throw std::runtime_error("Missing 'calib' section in YAML");
//     }
//     auto calib_node = root["calib"];

//     uint8_t camera_model = calib_node["camera_model"].as<int>();
//     calib.model = camera_model;
    
//     // --- Camera Matrix K ---
//     if (calib_node["camera_matrix"]) {
//         auto K_node = calib_node["camera_matrix"];
//         int rows = K_node["rows"].as<int>();
//         int cols = K_node["cols"].as<int>();
//         auto data = K_node["data"].as<std::vector<double>>();
        
//         if (data.size() != rows * cols) {
//             throw std::runtime_error("camera_matrix data size mismatch");
//         }
        
//         calib.K = cv::Mat(rows, cols, CV_64F);
//         for (int i = 0; i < rows; ++i) {
//             for (int j = 0; j < cols; ++j) {
//                 calib.K.at<double>(i, j) = data[i * cols + j];
//             }
//         }
//     }
    
//     // --- Distortion Coefficients D ---
//     if (calib_node["distortion_coefficients"]) {
//         auto D_node = calib_node["distortion_coefficients"];
//         int rows = D_node["rows"].as<int>();
//         int cols = D_node["cols"].as<int>();
//         auto data = D_node["data"].as<std::vector<double>>();
        
//         if (data.size() != rows * cols) {
//             throw std::runtime_error("distortion_coefficients data size mismatch");
//         }
        
//         calib.D = cv::Mat(rows, cols, CV_64F, data.data()).clone();
//     }

//     /* 
//         camera_stream
//     */    
//     if (!root["camera_stream"]) {
//         throw std::runtime_error("Missing 'camera_stream' section in YAML");
//     }
//     auto stream = root["camera_stream"];

//     std::string pipeline, url;
//     if (stream["rtsp_url"]) {
//         url = stream["rtsp_url"].as<std::string>();
//     } else {
//         throw std::runtime_error("missing 'rtsp_url'");
//     }
//     if (stream["gst_pipeline"]) {
//         pipeline = stream["gst_pipeline"].as<std::string>();
//     } else {
//         throw std::runtime_error("missing 'gst_pipeline'");
//     }

//     const std::string placeholder = "{url}";
//     size_t pos = pipeline.find(placeholder);

//     if (pos == std::string::npos) {
//         throw std::runtime_error("The 'gst_pipeline' string must contain the '{url}' placeholder.");
//     }
//     pipeline.replace(pos, placeholder.length(), url);
    
//     calib.rtsp_url = url;
//     calib.gst_pipeline = pipeline;

//     /*
//         size
//     */
//     if (root["size"]) {
//         auto size = root["size"];
//         int width = size["image_width"].as<int>();
//         int height = size["image_height"].as<int>();
//         calib.image_size = cv::Size(width, height);
//     }

//     return calib;
// }

class rectifyImageRaw : public rclcpp::Node
{
public:
    rectifyImageRaw() : Node("image_rectifier")
    // explicit rectifyImageRaw(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
    // : Node("image_rectifier", options)
    {
        // RCLCPP_INFO(this->get_logger(), "Node init");
        
        // CameraConfig calib_;
        std::string yp = "/home/wise/adsp-test3/ws_cam2/src/camera_stream/yaml/cam_101.yaml";
        std::string yaml_path = this->declare_parameter<std::string>("yaml_path",yp);
        calib_ = getCameraConfigFromYaml(yaml_path);
        RCLCPP_INFO(this->get_logger(), "Camera configration parsed");

        this->declare_parameter<std::string>("input_topic", "/cam2/image_raw");
        this->declare_parameter<std::string>("output_topic", "/image_rect");
        this->declare_parameter<bool>("debug_mode", true);
        
        this->declare_parameter<int>("image_width", 1280);
        this->declare_parameter<int>("image_height", 720);
        // this->declare_parameter<std::string>("distortion_model", "equidistant"); // fisheye?
        // this->declare_parameter<std::vector<double>>("camera_matrix.data", {});
        // this->declare_parameter<std::vector<double>>("distortion_coefficients.data", {});
        // this->declare_parameter<std::vector<double>>("rectification_matrix.data", {});
        // this->declare_parameter<double>("balance", 0.5);
        // this->declare_parameter<double>("alpha", 0.0);

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        debug_mode = this->get_parameter("debug_mode").as_bool();

        image_size_ = cv::Size(
            this->get_parameter("image_width").as_int(),
            this->get_parameter("image_height").as_int()
        );

        cv::initUndistortRectifyMap(
            calib_.K,                    // Camera matrix
            calib_.D,                    // Distortion coefficients
            cv::Mat(),                   // Rectification (identity for monocular)
            calib_.K,                    // New camera matrix (same as K)
            calib_.image_size,
            CV_16SC2,
            map1_, map2_
        );
        
        K_ = calib_.K;
        D_ = calib_.D;
        K_rect_ = calib_.K;

        if (calib_.model == 0) {  // Pinhole
            RCLCPP_INFO(this->get_logger(), "Using PINHOLE camera model");
            
            K_rect_ = K_.clone();  // 또는 getOptimalNewCameraMatrix() 사용
            
            cv::initUndistortRectifyMap(
                K_,                      // Camera matrix
                D_,                      // Distortion coefficients (5 params)
                cv::Mat(),               // Rectification matrix (identity)
                K_rect_,                 // New camera matrix
                image_size_,
                CV_16SC2,
                map1_, map2_
            );
            
        } else if (calib_.model == 1) {  // Fisheye
            RCLCPP_INFO(this->get_logger(), "Using FISHEYE camera model");
            
            double balance = 0.0;
            cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
                K_, 
                D_, 
                image_size_, 
                cv::Mat::eye(3, 3, CV_64F),  // 회전 없음
                K_rect_, 
                balance
            );
            
            cv::fisheye::initUndistortRectifyMap(
                K_,                          // Camera matrix
                D_,                          // Distortion coefficients (4 params)
                cv::Mat::eye(3, 3, CV_64F),  // Rectification matrix (identity)
                K_rect_,                     // New camera matrix
                image_size_,
                CV_16SC2,
                map1_, map2_
            );
            
        } else {
            std::string err_msg = "Invalid camera model: " + std::to_string(calib_.model);
            RCLCPP_FATAL(this->get_logger(), "%s", err_msg.c_str());
            throw std::runtime_error(err_msg);
        }

        // auto K_vec = this->get_parameter("camera_matrix.data").as_double_array();
        // auto D_vec = this->get_parameter("distortion_coefficients.data").as_double_array();
        // auto K_rect_vec = this->get_parameter("rectification_matrix.data").as_double_array();
        
        // if (K_vec.size() != 9 || D_vec.size() != 4) {
        //     std::string err_msg = "Invalid calibration data size from parameters!";
        //     RCLCPP_FATAL(this->get_logger(), err_msg.c_str());
        //     throw std::runtime_error(err_msg);
        // }

        // auto qos = rclcpp::SensorDataQoS().keep_last(1);
        rclcpp::QoS qos(rclcpp::SensorDataQoS().keep_last(1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic,
            qos,
            std::bind(&rectifyImageRaw::callbackUndistortion, this, std::placeholders::_1)
        );

        /* TODO: fisheye 또는 pinhole 모델 구분해서 값 받기 */
        // K_ = cv::Mat(3, 3, CV_64F, K_vec.data()).clone();
        // D_ = cv::Mat(1, 4, CV_64F, D_vec.data()).clone(); 
        // K_rect_ = cv::Mat(3, 3, CV_64F, K_rect_vec.data()).clone();

        /* 250912 */
        // fisheye
        // K_ = (cv::Mat_<double>(3, 3) << 
        //             557.286481458488, 0, 645.434763202952,
        //             0, 560.2662472618434, 422.5041216026116,
        //             0.0, 0.0, 1.0
        //         );
        // D_ = (cv::Mat_<double>(1, 4) << 
        //             -0.04717653045611134, 0.08466535058959987, -0.1122427643930151, 0.02860302134167773
        //         );
        // K_rect_ = (cv::Mat_<double>(3, 3) << 
        //             532.9377761804633, 0.0, 1387.398374688271,
        //             0.0, 535.7873514233846, 679.053415832703,
        //             0.0, 0.0, 1.0
        //         );
        
        //fisheye2 //TODO: 이 값들을 yaml 파일로 받아서 값 넣도록
        // K_ = (cv::Mat_<double>(3, 3) << 
        //             596.242804413754, 0, 640,
        //             0, 606.212049838454, 360,
        //             0.0, 0.0, 1.0
        //         );
        // D_ = (cv::Mat_<double>(1, 4) << 
        //             0.145233418368253, -0.448981613840834, 0.348017752004174, -0.0460532929321028
        //         );
        // K_rect_ = (cv::Mat_<double>(3, 3) << 
        //             532.9377761804633, 0.0, 1387.398374688271,
        //             0.0, 535.7873514233846, 679.053415832703,
        //             0.0, 0.0, 1.0
        //         );

        // balance_ = 0.0;
        // alpha_   = 0.0;
        // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        //     K_, D_, image_size_, cv::Mat::eye(3,3,CV_64F), K_rect_, balance_
        // );
        // cv::fisheye::initUndistortRectifyMap(
        //     K_, D_, cv::Mat::eye(3, 3, CV_64F), K_rect_, image_size_, CV_16SC2, map1_, map2_
        // );
        
    }

    ~rectifyImageRaw() = default;

private:
    bool debug_mode;
    double balance_, alpha_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    cv::Mat frame_raw, frame_rect, frame_prev;

    cv::Mat K_, D_, K_rect_;
    cv::Size image_size_;

    cv::Mat map1_, map2_;
    CameraConfig calib_;


    // void callbackUndistortion(const sensor_msgs::msg::Image::SharedPtr msg)
    void callbackUndistortion(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        // try {
        //     auto cvp = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        //     frame_raw = cvp->image;
        // } catch (cv_bridge::Exception &e) {
        //     RCLCPP_ERROR(this->get_logger(), "callbackUndistortion: cv_bridge exception | %s", e.what());
        //     return;
        // }

        // cv::remap(frame_raw, frame_rect, map1_, map2_, cv::INTER_LINEAR);

        // auto msg_pub = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame_rect).toImageMsg();
        // pub_->publish(*msg_pub);

        /**** */ 
        if (!msg || msg->data.empty() || msg->height==0 || msg->width==0) {
            RCLCPP_WARN(this->get_logger(),
            "Empty ima  ge received. size=%zu, w=%u, h=%u",
            msg ? msg->data.size() : 0,
            msg ? msg->width : 0,
            msg ? msg->height : 0);
            // return;

            frame_rect = frame_prev;

            std_msgs::msg::Header hd;
            if (msg) hd = msg->header;
            else hd.stamp = this->now();
            
            auto msg_pub = cv_bridge::CvImage(hd, sensor_msgs::image_encodings::BGR8, frame_rect).toImageMsg();
            pub_->publish(*msg_pub);

        } else {
            try {
                auto cvp = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
                frame_raw = cvp->image;
            } catch (cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "callbackUndistortion: cv_bridge exception | %s", e.what());
                return;
            }
            cv::remap(frame_raw, frame_rect, map1_, map2_, cv::INTER_LINEAR);
            
            auto msg_pub = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame_rect).toImageMsg();
            pub_->publish(*msg_pub);

            frame_prev = frame_rect;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rectifyImageRaw>());
    rclcpp::shutdown();
    return 0;
}