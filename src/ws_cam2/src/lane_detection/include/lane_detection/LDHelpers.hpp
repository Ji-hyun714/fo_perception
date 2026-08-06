#pragma once

#include <cstddef>
#include <string>

namespace ld_helpers {

enum class GateTraceProfile {
    Full,
    MaskLite
};

const char* tf(bool v);
const char* tfOpt(bool known, bool v);

struct GateTraceSideData {
    char side = 'L';
    bool det = false;
    bool rmse_known = false;

    bool exp_known = false;
    bool exp_pass = false;

    bool width_known = false;
    bool width_pass = false;

    bool rmse_h_known = false;
    bool rmse_h_pass = false;

    bool dy_known = false;
    bool dy_pass = false;

    bool dk_known = false;
    bool dk_pass = false;

    bool span_g_known = false;
    bool span_g_pass = false;

    bool qw_known = false;
    bool qw_pass = false;

    bool qg_known = false;
    bool qg_pass = false;

    bool hf = false;
    std::string st;
    std::string kf;

    bool vr_known = false;
    bool vr_pass = false;

    bool av = false;

    // Optional numeric values for trace readability
    double rmse_value = 0.0;
    double dy_value = 0.0;
    double dk_value = 0.0;
    double span_value = 0.0;
    double span_good_threshold = 0.0;
    double qw_pixel_count = 0.0;
    double qw_rmse = 0.0;
    double qg_pixel_count = 0.0;
    double qg_rmse = 0.0;
    double vr_value = 0.0;
    double vr_threshold = 0.0;
};

void printGateTraceFrameHeader(bool enabled, std::size_t frame_idx,
                               GateTraceProfile profile = GateTraceProfile::Full);
void printGateTraceSide(bool enabled, const GateTraceSideData& data,
                        GateTraceProfile profile = GateTraceProfile::Full);

} // namespace ld_helpers
