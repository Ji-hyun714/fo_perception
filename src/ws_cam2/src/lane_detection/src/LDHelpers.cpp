#include "lane_detection/LDHelpers.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

namespace ld_helpers {

const char* tf(bool v)
{
    return v ? "T" : "F";
}

const char* tfOpt(bool known, bool v)
{
    if (!known) {
        return "-";
    }
    return tf(v);
}

void printGateTraceFrameHeader(bool enabled, std::size_t frame_idx,
                               GateTraceProfile profile)
{
    if (!enabled) {
        return;
    }
    if (profile == GateTraceProfile::MaskLite) {
        std::cout << "[track][f=" << frame_idx << "][mask]" << std::endl;
        return;
    }
    std::cout << "[gate][f=" << frame_idx << "]" << std::endl;
}

void printGateTraceSide(bool enabled, const GateTraceSideData& data,
                        GateTraceProfile profile)
{
    if (!enabled) {
        return;
    }

    if (profile == GateTraceProfile::MaskLite) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        oss << data.side << "{"
            << "det=" << tf(data.det)
            << ",kf=" << data.kf
            << ",st=" << data.st;
        if (data.rmse_known) {
            oss << ",rmse=" << data.rmse_value;
        }
        if (data.span_g_known) {
            oss << ",span=" << data.span_value;
        }
        oss << ",vr=";
        if (data.vr_known) {
            oss << data.vr_value;
        } else {
            oss << "-";
        }
        oss << ",av=" << tf(data.av) << "}";
        std::cout << oss.str() << std::endl;
        return;
    }

    std::cout
        << data.side << "{"
        << "det=" << tf(data.det)
        << ",exp=" << tfOpt(data.exp_known, data.exp_pass)
        << ",width=" << tfOpt(data.width_known, data.width_pass)
        << ",rmse_h=" << tfOpt(data.rmse_h_known, data.rmse_h_pass)
        << ",dy=" << tfOpt(data.dy_known, data.dy_pass)
        << ",dk=" << tfOpt(data.dk_known, data.dk_pass)
        << ",span_g=" << tfOpt(data.span_g_known, data.span_g_pass)
        << ",qw=" << tfOpt(data.qw_known, data.qw_pass)
        << ",qg=" << tfOpt(data.qg_known, data.qg_pass)
        << ",hf=" << tf(data.hf)
        << ",st=" << data.st
        << ",kf=" << data.kf
        << ",vr=" << tfOpt(data.vr_known, data.vr_pass)
        << ",av=" << tf(data.av)
        << "}"
        << std::endl;
}

} // namespace ld_helpers
