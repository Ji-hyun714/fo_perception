
#include "camera_stream/GstAppsink.hpp"

/********************************** gstreamer ************************************/

GstAppsink::GstAppsink() {

}

GstAppsink::~GstAppsink() {
    close();
}

bool GstAppsink::open(const std::string &input_path)
{
    gst_init(nullptr, nullptr);

    GError *err = nullptr;
    pipe_ = gst_parse_launch(input_path.c_str(), &err);
    if (!pipe_ || err) {if (err) g_error_free(err); return false;}
    
    sink_ = gst_bin_get_by_name(GST_BIN(pipe_), "appsink");
    if (!sink_) return false;

    bus_ = gst_element_get_bus(pipe_);
    gst_bus_add_watch(bus_, [](GstBus*, GstMessage* m, gpointer user_data) -> gboolean {
        auto *self = static_cast<GstAppsink*>(user_data);
        if (GST_MESSAGE_TYPE(m)==GST_MESSAGE_ERROR || GST_MESSAGE_TYPE(m)==GST_MESSAGE_EOS)
            self->disconnected_ = true;
        return TRUE;
    }, this);
    
    gst_element_set_state(pipe_, GST_STATE_PLAYING);
    return true;
}

// bool isHostReachable(const std::string& host) {
//     // Construct the ping command
//     // -c 1: Send only one packet
//     // -W 1: Wait 1 second for a response
//     std::string command = "ping -c 1 -W 1 " + host + " > /dev/null 2>&1";
//     int exit_code = std::system(command.c_str());

//     // The command returns 0 on success (host is reachable)
//     return (exit_code == 0);
//     }

bool GstAppsink::read(cv::Mat &out, int timeout_ms)
{
    if (!pipe_) return false;
    // bool isHostReachable(const std::string& host) {
    // // Construct the ping command
    // // -c 1: Send only one packet
    // // -W 1: Wait 1 second for a response
    // std::string command = "ping -c 1 -W 1 " + host + " > /dev/null 2>&1";
    // int exit_code = std::system(command.c_str());

    // // The command returns 0 on success (host is reachable)
    // return (exit_code == 0);
    // }

//};
    if (disconnected_) return false;

    GstSample *s = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), timeout_ms*GST_MSECOND);
    if (!s) return false;

    GstBuffer *buf = gst_sample_get_buffer(s);
    GstCaps   *caps = gst_sample_get_caps(s);
    GstStructure *st = gst_caps_get_structure(caps, 0);
    
    int w,h; 
    gst_structure_get_int(st, "width", &w);
    gst_structure_get_int(st, "height", &h);
    const gchar *fmt = gst_structure_get_string(st, "format");
    // printf("[DEBUG] appsink received format: %s (%dx%d)\n", fmt, w, h);

    GstMapInfo map;
    gst_buffer_map(buf, &map, GST_MAP_READ);
    if (strcmp(fmt, "BGR")==0) {
        out = cv::Mat(h, w, CV_8UC3, (void*)map.data).clone();
        // printf("[DEBUG] appsink received BGR format\n");
    } else if (strcmp(fmt, "BGRx")==0 || strcmp(fmt, "BGRA")==0) {
        cv::Mat tmp(h, w, CV_8UC4, (void*)map.data);
        cv::cvtColor(tmp, out, cv::COLOR_BGRA2BGR);
        // printf("[DEBUG] appsink received RGB format\n");
    } else {
        out.release();
    }
    
    gst_buffer_unmap(buf, &map);
    gst_sample_unref(s);

    return !out.empty();
}

void GstAppsink::close()
{
    if (pipe_) {gst_element_set_state(pipe_, GST_STATE_NULL); gst_object_unref(pipe_); }
    if (sink_) gst_object_unref(sink_);
    if (bus_)  gst_object_unref(bus_);
    pipe_ = nullptr;
    sink_ = nullptr;
    bus_  = nullptr;
}

bool GstAppsink::is_disconnected() const { return disconnected_; }

/********************************** gstreamer ************************************/
