#pragma once 

#include <string>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <iostream>

struct _GstElement;
struct _GstBus;

class GstAppsink {
public:
    // 생성자와 소멸자 추가
    GstAppsink();
    ~GstAppsink();

    bool open(const std::string &pipeline_str);
    bool read(cv::Mat &frame, int timeout_ms=500);
    void close();
    bool is_disconnected() const;

private:
    _GstElement *pipe_ = nullptr;
    _GstElement *sink_ = nullptr;
    _GstBus *bus_ = nullptr;
    std::atomic<bool> disconnected_{false};
};
