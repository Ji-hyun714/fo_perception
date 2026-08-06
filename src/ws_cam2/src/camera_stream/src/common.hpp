#include <string>
#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>


struct CameraConfig {
    uint8_t model;       // camera model, 0: pinhole, 1: fisheye
    cv::Mat K;           // 3x3 camera matrix
    cv::Mat D;           // 1x5(pinhole)|1x4(fisheye) distortion coefficients
    cv::Mat R;           // 3x3 rotation matrix
    cv::Mat t;           // 3x1 translation vector
    cv::Size image_size;
    std::string rtsp_url;
    std::string host;
    std::string gst_pipeline;
};

inline std::string trimWhitespace(const std::string& input)
{
    const size_t first = input.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }

    const size_t last = input.find_last_not_of(" \t\r\n");
    return input.substr(first, last - first + 1);
}

inline std::string extractHostFromRtspUrl(const std::string& raw_url)
{
    const std::string url = trimWhitespace(raw_url);
    if (url.empty()) {
        return "";
    }

    const size_t scheme_pos = url.find("://");
    const size_t authority_start = (scheme_pos == std::string::npos) ? 0 : scheme_pos + 3;
    if (authority_start >= url.size()) {
        return "";
    }

    const size_t authority_end = url.find_first_of("/?#", authority_start);
    std::string authority = trimWhitespace(
        url.substr(authority_start, authority_end == std::string::npos ? std::string::npos : authority_end - authority_start)
    );
    if (authority.empty()) {
        return "";
    }

    const size_t at_pos = authority.rfind('@');
    if (at_pos != std::string::npos) {
        authority = trimWhitespace(authority.substr(at_pos + 1));
        if (authority.empty()) {
            return "";
        }
    }

    if (authority.front() == '[') {
        const size_t closing_bracket = authority.find(']');
        if (closing_bracket == std::string::npos || closing_bracket == 1) {
            return "";
        }
        return authority.substr(1, closing_bracket - 1);
    }

    return trimWhitespace(authority.substr(0, authority.find(':')));
}

CameraConfig getCameraConfigFromYaml(const std::string& yaml_path)
{
    CameraConfig calib;

    YAML::Node root = YAML::LoadFile(yaml_path);

    /* 
        calib 
    */    
    if (!root["calib"]) {
        throw std::runtime_error("Missing 'calib' section in YAML");
    }
    auto calib_node = root["calib"];

    uint8_t camera_model = calib_node["camera_model"].as<int>();
    calib.model = camera_model;
    
    // --- Camera Matrix K ---
    if (calib_node["camera_matrix"]) {
        auto K_node = calib_node["camera_matrix"];
        int rows = K_node["rows"].as<int>();
        int cols = K_node["cols"].as<int>();
        auto data = K_node["data"].as<std::vector<double>>();
        
        if (data.size() != rows * cols) {
            throw std::runtime_error("camera_matrix data size mismatch");
        }
        
        calib.K = cv::Mat(rows, cols, CV_64F);
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                calib.K.at<double>(i, j) = data[i * cols + j];
            }
        }
    }
    
    // --- Distortion Coefficients D ---
    if (calib_node["distortion_coefficients"]) {
        auto D_node = calib_node["distortion_coefficients"];
        int rows = D_node["rows"].as<int>();
        int cols = D_node["cols"].as<int>();
        auto data = D_node["data"].as<std::vector<double>>();
        
        if (data.size() != rows * cols) {
            throw std::runtime_error("distortion_coefficients data size mismatch");
        }
        
        calib.D = cv::Mat(rows, cols, CV_64F, data.data()).clone();
    }

    /* 
        camera_stream
    */    
    if (!root["camera_stream"]) {
        throw std::runtime_error("Missing 'camera_stream' section in YAML");
    }
    auto stream = root["camera_stream"];

    std::string pipeline, url;
    if (stream["rtsp_url"]) {
        url = stream["rtsp_url"].as<std::string>();
    } else {
        throw std::runtime_error("missing 'rtsp_url'");
    }
    if (stream["gst_pipeline"]) {
        pipeline = stream["gst_pipeline"].as<std::string>();
    } else {
        throw std::runtime_error("missing 'gst_pipeline'");
    }

    const std::string placeholder = "{url}";
    size_t pos = pipeline.find(placeholder);

    if (pos == std::string::npos) {
        throw std::runtime_error("The 'gst_pipeline' string must contain the '{url}' placeholder.");
    }
    pipeline.replace(pos, placeholder.length(), url);
    
    calib.rtsp_url = url;
    calib.host = extractHostFromRtspUrl(url);
    calib.gst_pipeline = pipeline;

    /*
        size
    */
    if (root["size"]) {
        auto size = root["size"];
        int width = size["image_width"].as<int>();
        int height = size["image_height"].as<int>();
        calib.image_size = cv::Size(width, height);
    }

    return calib;
}
