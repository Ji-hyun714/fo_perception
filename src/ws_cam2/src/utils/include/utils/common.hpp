/**
 include
 */

#include <stdexcept>
#include <cstdio>
#include <chrono>
#include <utility>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>


/*
 utils
 */

struct CameraConfig {
    uint8_t model;       // 0: pinhole, 1: fisheye
    cv::Mat K;           // 3x3 camera matrix (CV_64F)
    cv::Mat D;           // 1x5(pinhole) or 1x4(fisheye) distortion
    cv::Mat R;           // 3x3 rotation matrix (CV_64F)
    cv::Mat t;           // 3x1 translation vector (CV_64F)
    cv::Size image_size;
    std::string rtsp_url;
    std::string gst_pipeline;
};

inline CameraConfig getCameraConfigFromYaml(const std::string& yaml_path)
{
    CameraConfig calib;
    YAML::Node root = YAML::LoadFile(yaml_path);

    // ========== calib 섹션 ==========
    if (!root["calib"]) {
        throw std::runtime_error("Missing 'calib' section in YAML");
    }
    auto calib_node = root["calib"];

    // Camera model
    calib.model = calib_node["camera_model"].as<uint8_t>();
    
    // Camera Matrix K
    if (!calib_node["camera_matrix"]) {
        throw std::runtime_error("Missing 'camera_matrix'");
    }
    {
        auto K_node = calib_node["camera_matrix"];
        int rows = K_node["rows"].as<int>();
        int cols = K_node["cols"].as<int>();
        auto data = K_node["data"].as<std::vector<double>>();
        
        if (static_cast<int>(data.size()) != rows * cols) {
            throw std::runtime_error("camera_matrix data size mismatch");
        }
        
        calib.K = cv::Mat(rows, cols, CV_64F);
        std::memcpy(calib.K.data, data.data(), data.size() * sizeof(double));
    }
    
    // Distortion Coefficients D
    if (!calib_node["distortion_coefficients"]) {
        throw std::runtime_error("Missing 'distortion_coefficients'");
    }
    {
        auto D_node = calib_node["distortion_coefficients"];
        int rows = D_node["rows"].as<int>();
        int cols = D_node["cols"].as<int>();
        auto data = D_node["data"].as<std::vector<double>>();
        
        if (static_cast<int>(data.size()) != rows * cols) {
            throw std::runtime_error("distortion_coefficients data size mismatch");
        }
        
        calib.D = cv::Mat(rows, cols, CV_64F);
        std::memcpy(calib.D.data, data.data(), data.size() * sizeof(double));
    }

    // Rotation Matrix R (추가!)
    if (!calib_node["rotation_matrix"]) {
        throw std::runtime_error("Missing 'rotation_matrix'");
    }
    {
        auto R_node = calib_node["rotation_matrix"];
        int rows = R_node["rows"].as<int>();
        int cols = R_node["cols"].as<int>();
        auto data = R_node["data"].as<std::vector<double>>();
        
        if (static_cast<int>(data.size()) != rows * cols) {
            throw std::runtime_error("rotation_matrix data size mismatch");
        }
        
        calib.R = cv::Mat(rows, cols, CV_64F);
        std::memcpy(calib.R.data, data.data(), data.size() * sizeof(double));
    }

    // Translation Vector t (추가!)
    if (!calib_node["translation_vector"]) {
        throw std::runtime_error("Missing 'translation_vector'");
    }
    {
        auto t_node = calib_node["translation_vector"];
        int rows = t_node["rows"].as<int>();
        int cols = t_node["cols"].as<int>();
        auto data = t_node["data"].as<std::vector<double>>();
        
        if (static_cast<int>(data.size()) != rows * cols) {
            throw std::runtime_error("translation_vector data size mismatch");
        }
        
        calib.t = cv::Mat(rows, cols, CV_64F);
        std::memcpy(calib.t.data, data.data(), data.size() * sizeof(double));
    }

    // ========== camera_stream 섹션 ==========
    if (!root["camera_stream"]) {
        throw std::runtime_error("Missing 'camera_stream' section in YAML");
    }
    auto stream = root["camera_stream"];

    // RTSP URL
    if (!stream["rtsp_url"]) {
        throw std::runtime_error("Missing 'rtsp_url'");
    }
    std::string url = stream["rtsp_url"].as<std::string>();
    calib.rtsp_url = url;

    // GStreamer Pipeline
    if (!stream["gst_pipeline"]) {
        throw std::runtime_error("Missing 'gst_pipeline'");
    }
    std::string pipeline = stream["gst_pipeline"].as<std::string>();

    // Replace {url} placeholder if present (optional for v4l2)
    const std::string placeholder = "{url}";
    size_t pos = pipeline.find(placeholder);
    if (pos != std::string::npos) {
        pipeline.replace(pos, placeholder.length(), url);
    }
    calib.gst_pipeline = pipeline;

    // ========== size 섹션 ==========
    if (!root["size"]) {
        throw std::runtime_error("Missing 'size' section in YAML");
    }
    {
        auto size = root["size"];
        int width = size["image_width"].as<int>();
        int height = size["image_height"].as<int>();
        calib.image_size = cv::Size(width, height);
    }

    return calib;
}

inline void printCameraConfig(const CameraConfig& config)
{
    std::printf("========== Camera Configuration from YAML ==========\n");

    std::printf("Camera Model: %s\n\n", config.model == 0 ? "Pinhole" : "Fisheye");

    std::printf("Image Size: %d x %d\n\n", config.image_size.width, config.image_size.height);

    std::printf("Camera Matrix K (3x3):\n");
    for (int i=0; i<3; i++) {
        std::printf("  [");
        for (int j=0; j<3; j++) {
            std::printf("%12.6f", config.K.at<double>(i,j));
            if(j<2) std::printf(", ");
        }
        std::printf("]\n\n");
    }

    std::printf("Rotation Matrix R (3x3):\n");
    for (int i = 0; i < 3; i++) {
        std::printf("  [");
        for (int j = 0; j < 3; j++) {
            std::printf("%12.6f", config.R.at<double>(i, j));
            if (j < 2) std::printf(", ");
        }
        std::printf("]\n\n");
    }

    std::printf("Translation Vector t (3x1):\n  [");
    for (int i = 0; i < 3; i++) {
        std::printf("%12.6f", config.t.at<double>(i));
        if (i < 2) std::printf(", ");
    }
    std::printf("]\n\n");

    // RTSP URL
    std::printf("RTSP URL: %s\n\n", config.rtsp_url.c_str());

    // GStreamer Pipeline
    std::printf("GStreamer Pipeline:\n%s\n\n", config.gst_pipeline.c_str());

    std::printf("===================================================\n\n");


}

template <typename Fn>
inline double measureBlockMs(Fn&& fn)
{
    const auto t0 = std::chrono::steady_clock::now();
    std::forward<Fn>(fn)();
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}
