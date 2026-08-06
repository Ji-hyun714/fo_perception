// #include "LaneDetection.hpp"
#include "lane_detection/LaneDetection.hpp"
#include "lane_detection/LDHelpers.hpp"
#include "utils/common.hpp"
#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>

namespace {
constexpr double kRearOriginShiftM = 3.8;

struct RowRun {
    int x_min = 0;
    int x_max = 0;
};

std::vector<RowRun> extractWhiteRunsInRow(const cv::Mat& lane_mask, const cv::Rect& win_rect, int y, int min_run_px)
{
    std::vector<RowRun> runs;
    if (lane_mask.empty() || y < win_rect.y || y >= win_rect.y + win_rect.height) return runs;
    if (win_rect.width <= 0 || win_rect.height <= 0) return runs;

    const uchar* row = lane_mask.ptr<uchar>(y);
    const int x_begin = std::max(0, win_rect.x);
    const int x_end = std::min(lane_mask.cols, win_rect.x + win_rect.width);
    int run_start = -1;

    for (int x = x_begin; x < x_end; ++x) {
        const bool on = (row[x] != 0);
        if (on) {
            if (run_start < 0) run_start = x;
            continue;
        }
        if (run_start >= 0) {
            const int run_len = x - run_start;
            if (run_len >= min_run_px) runs.push_back({run_start, x - 1});
            run_start = -1;
        }
    }
    if (run_start >= 0) {
        const int run_len = x_end - run_start;
        if (run_len >= min_run_px) runs.push_back({run_start, x_end - 1});
    }
    return runs;
}

int pickBestRunIndexNearCentroid(const std::vector<RowRun>& runs, int cx)
{
    if (runs.empty()) return -1;

    int best_idx = -1;
    int best_dist = std::numeric_limits<int>::max();
    for (int i = 0; i < (int)runs.size(); ++i) {
        const auto& run = runs[i];
        int dist = 0;
        if (cx < run.x_min) dist = run.x_min - cx;
        else if (cx > run.x_max) dist = cx - run.x_max;
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = i;
            if (dist == 0) break;
        }
    }
    return best_idx;
}

int refineRunIndexByDirection(const std::vector<RowRun>& runs,
                              int base_idx,
                              LaneSideMask side,
                              bool enable_run_hop,
                              int max_hop_gap_px,
                              int min_hop_run_width_px,
                              int max_hops_per_row)
{
    if (!enable_run_hop) return base_idx;
    if (base_idx < 0 || base_idx >= (int)runs.size()) return base_idx;

    int idx = base_idx;
    const int hop_limit = std::max(0, max_hops_per_row);
    for (int hop = 0; hop < hop_limit; ++hop) {
        const int next_idx = (side == LaneSideMask::Left) ? idx + 1 : idx - 1;
        if (next_idx < 0 || next_idx >= (int)runs.size()) break;

        const RowRun& cur = runs[(size_t)idx];
        const RowRun& next = runs[(size_t)next_idx];
        const int gap_px = (side == LaneSideMask::Left)
            ? (next.x_min - cur.x_max - 1)
            : (cur.x_min - next.x_max - 1);
        const int next_width = next.x_max - next.x_min + 1;

        if (gap_px < 0) break;
        if (gap_px > max_hop_gap_px) break;
        if (next_width < min_hop_run_width_px) break;
        idx = next_idx;
    }

    return idx;
}
} // namespace

/* 생성자 */
LaneLineDetector::LaneLineDetector(const CalibData& calib, std::string& cfg) 
    : calib_(calib)
{ 
    // bool success = config_.loadConfig("config.yaml");
    bool success = config_.loadConfig(cfg);
        if (!success) {
            std::cerr << "Warning: Failed to load config, using default values." << std::endl;
        }

    initIPM();
    initKalmanFilter();
}

std::string LaneLineDetector::stateToString(LaneState s) const
{
    switch (s) {
        case LaneState::GOOD: return "GOOD";
        case LaneState::WEAK: return "WEAK";
        case LaneState::BAD:  return "BAD";
        case LaneState::NODET:
        default:              return "NODET";
    }
}

std::string LaneLineDetector::makeCsvName() const
{
    auto t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    char buf[32];
    std::strftime(buf, sizeof(buf), "output_%y%m%d-%H%M%S.csv", &tm);
    return std::string(buf);
}

LaneResult LaneLineDetector::Detect(const cv::Mat& frame) {
    cur_result_ = LaneResult();
    debug_.clear();
    if (config_.show_org) pushDebugImage(frame);

    if (config_.enable_csv_log && !csv_initialized_) {
        std::string name = makeCsvName();
        csv_file_.open(name);
        if (csv_file_.is_open()) {
            csv_file_ << "frame,side,state,det,px,span,rmse,dy,dk,hf\n";
            csv_initialized_ = true;
        } else {
            std::cerr << "ERROR: Failed to open CSV log file: " << name << std::endl;
        }
    }

    // remap(frame, undist); // raw 이미지 사용할 때
    undist = frame.clone(); // ros2에서는 왜곡보정된 상태로 들어온다고 가정

    // 1) 전처리
    preprocess(undist, bev_bin);

    // 2) 픽셀 수집 (상태 기반 전략 분기 포함)
    findPixels(bev_bin);
    // 3) 다항식 피팅
    fitPolynomial();
    // 4) 검증 + 스무딩(KF)
    validateAndSmooth();

    if (config_.show_sld_win) {
        drawPolyBev(sld_win, cv::Scalar(0, 255, 0),
                    cur_result_.coeffs_L, cur_result_.coeffs_R,
                    cur_result_.is_detected_L, cur_result_.is_detected_R);
        cv::putText(sld_win, "yellow:raw fit / green:KF",
                    cv::Point(10, sld_win.rows - 45),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9,
                    cv::Scalar(200, 200, 200), 2);
        pushDebugImage(sld_win);
    }

    drawResults(undist);

    frame_idx_++;

    return cur_result_;
}

LaneResult LaneLineDetector::DetectFromMask(const cv::Mat& lane_mask, const cv::Mat& frame_org)
{
    const auto t_detect_start = std::chrono::steady_clock::now();
    double ms_mask_prepare = 0.0;
    double ms_find_pixels = 0.0;
    double ms_fit = 0.0;
    double ms_tracking = 0.0;
    auto printDetectFromMaskTiming = [&]() {
        if (!config_.log_process_time) {
            return;
        }
        const double total_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_detect_start).count();
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "[debug] DetectFromMask timing(ms): "
            << "total=" << total_ms
            << "| mask_prepare=" << ms_mask_prepare
            << ", find_pixels=" << ms_find_pixels
            << ", fitting=" << ms_fit
            << ", tracking=" << ms_tracking;
        std::cout << oss.str() << std::endl;
    };

    cur_result_ = LaneResult();
    debug_.clear();

    if (lane_mask.empty()) {
        std::cerr << "Warning: lane_mask is empty." << std::endl;
        return cur_result_;
    }

    cv::Mat mask_u8;
    ms_mask_prepare = measureBlockMs([&]() {
        if (lane_mask.channels() == 1) {
            mask_u8 = lane_mask;
        } else {
            cv::cvtColor(lane_mask, mask_u8, cv::COLOR_BGR2GRAY);
        }
        if (mask_u8.type() != CV_8UC1) {
            mask_u8.convertTo(mask_u8, CV_8UC1);
        }
        if (config_.show_lane_mask) pushDebugImageExec(mask_u8, "lane_mask");

        cv::threshold(mask_u8, mask_u8, 0, 255, cv::THRESH_BINARY);
        cv::resize(mask_u8, mask_u8, cv::Size(640, 360), 0.0, 0.0, cv::INTER_NEAREST);

        // SW 픽셀 수집 단계에서 m->pixel 제한선을 계산할 수 있도록 선행 구성
        cv::Size rect_size = frame_org.empty() ? calib_.imageSize : frame_org.size();
        if (rect_size.width <= 0 || rect_size.height <= 0) {
            rect_size = mask_u8.size();
        }
        buildMaskToBevHomography(mask_u8.size(), rect_size);
        updateSwForwardLimitCache(mask_u8.size(), rect_size);

        if (config_.show_preprocessed) pushDebugImageExec(mask_u8, "preprocessed");
    });
    bool lane_lost = false;
    ms_find_pixels = measureBlockMs([&]() {
        lane_lost = findPixelsFromMask(mask_u8);
    });

    const bool sample_kf_ready = !config_.use_geometry_kf &&
                                 config_.use_sample_kf_masklite &&
                                 sampleCount() >= 4 &&
                                 hasValidSampleKfNoiseConfig();
    if (!config_.use_geometry_kf && config_.use_sample_kf_masklite && !sample_kf_ready) {
        warnSampleKfNoiseConfigFallback("DetectFromMask");
    }

    if (lane_lost && sample_kf_ready) {
        printDetectFromMaskTiming();
        frame_idx_++;
        return cur_result_;
    }

    ms_fit = measureBlockMs([&]() {
        // DetectFromMask 경로: sliding-window centroid(mask px) -> BEV px 정렬 후 피팅

        // /* 임시로, 실측 검증을 위한 수동 수집 픽셀 좌표 */
        // pixels_L_ = {
        //     {153,295}, {154,293}, {157,291}, {159,289}, {161,287}, {163,284},
        //     {165,282}, {167,281}, {168,279}, {170,277}, {172,275}, {174,273},
        //     {175,271}, {178,269}, {180,266}, {183,263}, {186,260}, {189,257},
        //     {192,254}, {195,251}, {196,249}, {198,247}, {200,245}, {203,243},
        //     {204,241}, {207,238}, {210,235}, {214,231}, {216,229}, {218,227},
        //     {220,225}, {223,221}, {224,220}, {225,218}
        // };

        // pixels_R_ = {
        //     {454,294}, {451,291}, {448,288}, {446,285}, {442,283}, {441,280},
        //     {438,278}, {436,277}, {433,274}, {432,272}, {430,271}, {427,268},
        //     {425,266}, {423,264}, {422,262}, {418,260}, {417,258}, {414,255},
        //     {412,253}, {409,250}, {405,246}, {403,244}, {401,241}, {398,239},
        //     {395,236}, {393,233}, {391,231}, {388,229}, {385,227}, {384,226},
        //     {383,223}, {382,221}, {380,219}
        // };

        transformMaskCentroidsToBev(pixels_L_);
        transformMaskCentroidsToBev(pixels_R_);
        normalizeMaskLaneSlotsByTopEndpoint(pixels_L_, pixels_R_);

        fitPolynomial();
        pushBevVehicleDebugImage(pixels_L_, pixels_R_);
    });

    ms_tracking = measureBlockMs([&]() {
        if (config_.use_geometry_kf) {
            trackLaneGeometryKF();
        } else if (sample_kf_ready) {
            trackLaneSamplesKF();
        } else {
            resultsFromRawFit();
        }
    });

    if (!frame_org.empty()) {
        cv::Mat frame_draw = frame_org.clone();
        drawResults(frame_draw);
    }

    printDetectFromMaskTiming();

    frame_idx_++;

    return cur_result_;
}

void LaneLineDetector::initIPM() {

    roi_x_dist = config_.roi_xmax - config_.roi_xmin;
    roi_y_dist = config_.roi_ymax - config_.roi_ymin;
    bev_mpp_x_ = roi_x_dist / config_.bev_size.height; 
    bev_mpp_y_ = roi_y_dist / config_.bev_size.width;

    /* ROI 지면 좌표 설정 4개 : TL-TR-BR-BL */
    double y_left = config_.roi_ymax;
    double y_right = config_.roi_ymin;

    std::vector<cv::Point3f> roi_gnd_pts;
    roi_gnd_pts.push_back(cv::Point3f(config_.roi_xmax, y_left, 0 )); // Far-Left
    roi_gnd_pts.push_back(cv::Point3f(config_.roi_xmax, y_right, 0 )); // Far-Right
    roi_gnd_pts.push_back(cv::Point3f(config_.roi_xmin, y_right, 0 )); // Near-Right
    roi_gnd_pts.push_back(cv::Point3f(config_.roi_xmin, y_left, 0 )); // Near-Left

    /* 지면 좌표 투영 -> 픽셀 좌표 */
    std::vector<cv::Point2f> src_pts;
    cv::Mat rvec, tvec;

    cv::Rodrigues(calib_.R_w2c, rvec);
    tvec = calib_.t_w2c;

    cv::projectPoints(roi_gnd_pts, rvec, tvec, calib_.Krect, cv::noArray(), src_pts);
    this->roi_pixel_pts_ = src_pts; // 

    /* BEV 대응점 : TL-TR-BR-BL */
    std::vector<cv::Point2f> dst_pts;
    float w = config_.bev_size.width;
    float h = config_.bev_size.height;

    dst_pts.push_back(cv::Point2f(0, 0)); // TL
    dst_pts.push_back(cv::Point2f(w, 0)); // TR
    dst_pts.push_back(cv::Point2f(w, h)); // BR
    dst_pts.push_back(cv::Point2f(0, h)); // BL

    /* 변환 행렬 계산 */
    H_bev = cv::getPerspectiveTransform(src_pts, dst_pts);
    H_bev_inv = H_bev.inv();

    std::cout << "[initIPM] IPM Matrix Calculated" << std::endl;

}

void LaneLineDetector::buildMaskToBevHomography(const cv::Size& mask_size, const cv::Size& rect_size)
{
    if (mask_size.width <= 0 || mask_size.height <= 0 ||
        rect_size.width <= 0 || rect_size.height <= 0 ||
        calib_.H_i2w_rect.empty()) {
        H_mask2bev_.release();
        return;
    }

    if (!H_mask2bev_.empty() && mask_ref_size_ == mask_size && rect_ref_size_ == rect_size) {
        return;
    }

    mask_ref_size_ = mask_size;
    rect_ref_size_ = rect_size;

    const double sx = static_cast<double>(rect_size.width) / static_cast<double>(mask_size.width);
    const double sy = static_cast<double>(rect_size.height) / static_cast<double>(mask_size.height);

    // mask(px) -> rectified image(px)
    const cv::Mat S_mask2rect = (cv::Mat_<double>(3, 3) <<
        sx, 0.0, 0.0,
        0.0, sy, 0.0,
        0.0, 0.0, 1.0);

    // world(m) -> BEV(px), X+: 전방(위), Y+: 좌측(왼쪽)
    const cv::Mat T_w2bev = (cv::Mat_<double>(3, 3) <<
        0.0,            -1.0 / bev_mpp_y_,  config_.roi_ymax / bev_mpp_y_,
       -1.0 / bev_mpp_x_, 0.0,              config_.roi_xmax / bev_mpp_x_,
        0.0,             0.0,               1.0);

    cv::Mat H_i2w;
    calib_.H_i2w_rect.convertTo(H_i2w, CV_64F);
    H_mask2bev_ = T_w2bev * H_i2w * S_mask2rect;

    const double w = H_mask2bev_.at<double>(2, 2);
    if (std::abs(w) > 1e-12) {
        H_mask2bev_ /= w;
    }
}

void LaneLineDetector::transformMaskCentroidsToBev(std::vector<cv::Point>& pts) const
{
    if (pts.empty()) return;
    if (H_mask2bev_.empty()) {
        pts.clear();
        return;
    }

    std::vector<cv::Point2f> src;
    src.reserve(pts.size());
    for (const auto& p : pts) {
        src.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
    }

    std::vector<cv::Point2f> dst;
    cv::perspectiveTransform(src, dst, H_mask2bev_);

    std::vector<cv::Point> bev_pts;
    bev_pts.reserve(dst.size());
    const int w_bev = config_.bev_size.width;
    const int h_bev = config_.bev_size.height;
    for (const auto& p : dst) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
        const int u = cvRound(p.x);
        const int v = cvRound(p.y);
        if (u < 0 || u >= w_bev || v < 0 || v >= h_bev) continue;
        bev_pts.emplace_back(u, v);
    }

    pts.swap(bev_pts);
}

cv::Point2d LaneLineDetector::bevPixToVehicle(const cv::Point2d& p_bev) const
{
    const double x_old = config_.roi_xmax - (p_bev.y * bev_mpp_x_);
    const double x_veh = x_old + kRearOriginShiftM; // 기존 원점(번호판 중심)에서 x 방향 오프셋 추가
    const double y_veh = config_.roi_ymax - (p_bev.x * bev_mpp_y_);
    return cv::Point2d(x_veh, y_veh);
}

cv::Point2d LaneLineDetector::vehicleToBevPix(const cv::Point2d& p_veh) const
{
    const double x_old = p_veh.x - kRearOriginShiftM; // x 방향 오프셋 역변환
    const double u_bev = (config_.roi_ymax - p_veh.y) / bev_mpp_y_;
    const double v_bev = (config_.roi_xmax - x_old) / bev_mpp_x_;
    return cv::Point2d(u_bev, v_bev);
}

std::vector<cv::Point2f> LaneLineDetector::bevToVehicleLocal(const std::vector<cv::Point>& bev_pts) const
{
    std::vector<cv::Point2f> veh_pts;
    veh_pts.reserve(bev_pts.size());
    for (const auto& p : bev_pts) {
        const cv::Point2d veh = bevPixToVehicle(cv::Point2d(static_cast<double>(p.x), static_cast<double>(p.y)));
        veh_pts.emplace_back(static_cast<float>(veh.x), static_cast<float>(veh.y));
    }
    return veh_pts;
}

void LaneLineDetector::pushBevVehicleDebugImage(const std::vector<cv::Point>& bev_L, const std::vector<cv::Point>& bev_R)
{
    if (!config_.show_fitpoly) return;

    cv::Mat vis_bev = cv::Mat::zeros(config_.bev_size, CV_8UC3);
    const cv::Scalar kGreen(0, 255, 0);
    const cv::Scalar kBlue(255, 0, 0);
    const cv::Scalar kRed(0, 0, 255);
    const cv::Scalar kGray(96, 96, 96);

    for (double guide_x_m = 5.0; guide_x_m <= config_.roi_xmax + 1e-6; guide_x_m += 5.0) {
        const cv::Point2d p_bev = vehicleToBevPix(cv::Point2d(guide_x_m, 0.0));
        if (!std::isfinite(p_bev.y)) continue;
        const int v = cvRound(p_bev.y);
        if (v < 0 || v >= vis_bev.rows) continue;
        cv::line(vis_bev, cv::Point(0, v), cv::Point(vis_bev.cols - 1, v), kGray, 1, cv::LINE_AA);

        std::ostringstream label_ss;
        label_ss << static_cast<int>(std::round(guide_x_m)) << "m";
        const std::string label = label_ss.str();
        int baseline = 0;
        const cv::Size text_size = cv::getTextSize(
            label, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, &baseline);
        const int text_x = std::max(0, vis_bev.cols - text_size.width - 6);
        const int text_y = std::min(vis_bev.rows - baseline - 2, v + text_size.height + 3);
        if (text_y > v) {
            cv::putText(
                vis_bev,
                label,
                cv::Point(text_x, text_y),
                cv::FONT_HERSHEY_SIMPLEX,
                0.35,
                kGray,
                1,
                cv::LINE_AA);
        }
    }

    auto drawPts = [&](const std::vector<cv::Point>& bev_pts) {
        for (const auto& p : bev_pts) {
            const int u = p.x;
            const int v = p.y;
            if (u < 0 || u >= vis_bev.cols || v < 0 || v >= vis_bev.rows) continue;
            cv::circle(vis_bev, cv::Point(u, v), 3, kGreen, cv::FILLED);
        }
    };

    auto drawFitCurve = [&](const cv::Vec4d& coeffs, bool detected, const cv::Scalar& color) {
        if (!detected) return;

        std::vector<cv::Point> curve_pts;
        constexpr double kDxM = 0.2;
        const double x_min = config_.roi_xmin + kRearOriginShiftM;
        const double x_max = config_.roi_xmax + kRearOriginShiftM;
        for (double x_veh = x_min; x_veh <= x_max; x_veh += kDxM) {
            const double y_veh = coeffs[0] * x_veh * x_veh * x_veh
                               + coeffs[1] * x_veh * x_veh
                               + coeffs[2] * x_veh
                               + coeffs[3];
            const cv::Point2d p_bev = vehicleToBevPix(cv::Point2d(x_veh, y_veh));
            if (!std::isfinite(p_bev.x) || !std::isfinite(p_bev.y)) continue;

            const int u = cvRound(p_bev.x);
            const int v = cvRound(p_bev.y);
            if (u < 0 || u >= vis_bev.cols || v < 0 || v >= vis_bev.rows) continue;
            curve_pts.emplace_back(u, v);
        }

        if (curve_pts.size() >= 2) {
            cv::polylines(vis_bev, curve_pts, false, color, 2);
        }
    };

    drawPts(bev_L);
    drawPts(bev_R);
    drawFitCurve(cur_result_.coeffs_fit_L, cur_result_.detected_fit_L, kBlue); // Left: blue
    drawFitCurve(cur_result_.coeffs_fit_R, cur_result_.detected_fit_R, kRed);  // Right: red
    pushDebugImageExec(vis_bev, "bev_vehicle_points");
}

void LaneLineDetector::remap(const cv::Mat& src, cv::Mat& dst) {
    
    if (!calib_.map1.empty() && !calib_.map2.empty()) {
        cv::remap(src, dst, calib_.map1, calib_.map2, cv::INTER_LINEAR);
    } else {
        src.copyTo(dst); // LUT 없으면 원본 복사
    }
}

void LaneLineDetector::initKalmanFilter() {

    kf_L_.init(8, 4, 0);
    kf_R_.init(8, 4, 0);

    /* 전이행렬(F): 다음 값 = 현재 값 + 속도 */
    kf_L_.transitionMatrix = (cv::Mat_<float>(8, 8) << 
        1,0,0,0, 1,0,0,0, 
        0,1,0,0, 0,1,0,0, 
        0,0,1,0, 0,0,1,0, 
        0,0,0,1, 0,0,0,1,
        0,0,0,0, 1,0,0,0, // 변화율은 등속 모델 가정 (da' = da)
        0,0,0,0, 0,1,0,0,
        0,0,0,0, 0,0,1,0,
        0,0,0,0, 0,0,0,1
    );
    kf_R_.transitionMatrix = kf_L_.transitionMatrix.clone();

    /* 측정 행렬(H) */
    kf_L_.measurementMatrix = cv::Mat(cv::Mat::eye(8, 8, CV_32F)).rowRange(0,4);
    kf_R_.measurementMatrix = cv::Mat(cv::Mat::eye(8, 8, CV_32F)).rowRange(0,4);

    /* 공분산 행렬: 튜닝 포인트 */
    cv::setIdentity(kf_L_.processNoiseCov, cv::Scalar::all(1e-4)); // Q: 예측 믿는 정도(작으면 관성 유지, 크면 변화에 민감)
    cv::setIdentity(kf_R_.processNoiseCov, cv::Scalar::all(1e-4));

    cv::setIdentity(kf_L_.measurementNoiseCov, cv::Scalar::all(1e-1)); // R: 카메라 검출 결과 믿는 정도(작으면 검출 튀어도 무시, 크면 검출 결과 바로 반응)
    cv::setIdentity(kf_R_.measurementNoiseCov, cv::Scalar::all(1e-1));
    
    cv::setIdentity(kf_L_.errorCovPost, cv::Scalar::all(1)); // R: Error Covariance 초기 신뢰도(처음엔 모르니까 크게)
    cv::setIdentity(kf_R_.errorCovPost, cv::Scalar::all(1));
    kf_L_.statePost = cv::Mat::zeros(8, 1, CV_32F);
    kf_R_.statePost = cv::Mat::zeros(8, 1, CV_32F);
    kf_L_.statePre = cv::Mat::zeros(8, 1, CV_32F);
    kf_R_.statePre = cv::Mat::zeros(8, 1, CV_32F);
    kf_initialized_L_ = false;
    kf_initialized_R_ = false;

    meas_L_ = cv::Mat::zeros(4, 1, CV_32F);
    meas_R_ = cv::Mat::zeros(4, 1, CV_32F);
    initGeometryKalmanFilter();
    initGeometryKalmanFilter();
    initSampleKalmanFilter();
}

void LaneLineDetector::initGeometryKalmanFilter()
{
    auto initOne = [&](cv::KalmanFilter& kf) {
        kf.init(4, 4, 0);
        kf.transitionMatrix = cv::Mat::eye(4, 4, CV_32F);
        kf.measurementMatrix = cv::Mat::eye(4, 4, CV_32F);
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(config_.geom_kf_process_noise));
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(config_.geom_kf_measurement_noise));
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1.0));
        kf.statePost = cv::Mat::zeros(4, 1, CV_32F);
        kf.statePre = cv::Mat::zeros(4, 1, CV_32F);
    };

    initOne(kf_geom_L_);
    initOne(kf_geom_R_);
    kf_geom_initialized_L_ = false;
    kf_geom_initialized_R_ = false;
    geom_miss_L_ = 0;
    geom_miss_R_ = 0;
    geom_hold_valid_L_ = false;
    geom_hold_valid_R_ = false;
    geom_hold_lane_start_m_L_ = 0.0;
    geom_hold_lane_end_m_L_ = 0.0;
    geom_hold_span_m_L_ = 0.0;
    geom_hold_view_range_m_L_ = 0.0;
    geom_hold_lane_start_m_R_ = 0.0;
    geom_hold_lane_end_m_R_ = 0.0;
    geom_hold_span_m_R_ = 0.0;
    geom_hold_view_range_m_R_ = 0.0;
}

void LaneLineDetector::initSampleKalmanFilter()
{
    const int n = sampleCount();
    if (n <= 0) {
        kf_samples_initialized_L_ = false;
        kf_samples_initialized_R_ = false;
        meas_samples_L_.release();
        meas_samples_R_.release();
        pred_samples_L_.release();
        pred_samples_R_.release();
        estimated_samples_L_.release();
        estimated_samples_R_.release();
        return;
    }

    if (!hasValidSampleKfNoiseConfig()) {
        kf_samples_initialized_L_ = false;
        kf_samples_initialized_R_ = false;
        meas_samples_L_ = cv::Mat::zeros(n, 1, CV_32F);
        meas_samples_R_ = cv::Mat::zeros(n, 1, CV_32F);
        pred_samples_L_ = cv::Mat::zeros(2 * n, 1, CV_32F);
        pred_samples_R_ = cv::Mat::zeros(2 * n, 1, CV_32F);
        estimated_samples_L_ = cv::Mat::zeros(2 * n, 1, CV_32F);
        estimated_samples_R_ = cv::Mat::zeros(2 * n, 1, CV_32F);
        sample_miss_L_ = 0;
        sample_miss_R_ = 0;
        warnSampleKfNoiseConfigFallback("initSampleKalmanFilter");
        return;
    }

    if (!hasValidSampleKfNoiseConfig()) {
        kf_samples_initialized_L_ = false;
        kf_samples_initialized_R_ = false;
        meas_samples_L_ = cv::Mat::zeros(n, 1, CV_32F);
        meas_samples_R_ = cv::Mat::zeros(n, 1, CV_32F);
        pred_samples_L_ = cv::Mat::zeros(2 * n, 1, CV_32F);
        pred_samples_R_ = cv::Mat::zeros(2 * n, 1, CV_32F);
        estimated_samples_L_ = cv::Mat::zeros(2 * n, 1, CV_32F);
        estimated_samples_R_ = cv::Mat::zeros(2 * n, 1, CV_32F);
        sample_miss_L_ = 0;
        sample_miss_R_ = 0;
        warnSampleKfNoiseConfigFallback("initSampleKalmanFilter");
        return;
    }

    auto initOne = [&](cv::KalmanFilter& kf) {
        kf.init(2 * n, n, 0);

        kf.transitionMatrix = cv::Mat::eye(2 * n, 2 * n, CV_32F);
        for (int i = 0; i < n; ++i) {
            kf.transitionMatrix.at<float>(i, n + i) = 1.0f;
        }

        kf.measurementMatrix = cv::Mat::zeros(n, 2 * n, CV_32F);
        for (int i = 0; i < n; ++i) {
            kf.measurementMatrix.at<float>(i, i) = 1.0f;
        }

        kf.processNoiseCov = cv::Mat::zeros(2 * n, 2 * n, CV_32F);
        for (int i = 0; i < n; ++i) {
            kf.processNoiseCov.at<float>(i, i) =
                static_cast<float>(config_.kf_sample_Q_pos_list[(size_t)i]);
            kf.processNoiseCov.at<float>(n + i, n + i) =
                static_cast<float>(config_.kf_sample_Q_vel_list[(size_t)i]);
            kf.processNoiseCov.at<float>(i, i) =
                static_cast<float>(config_.kf_sample_Q_pos_list[(size_t)i]);
            kf.processNoiseCov.at<float>(n + i, n + i) =
                static_cast<float>(config_.kf_sample_Q_vel_list[(size_t)i]);
        }

        kf.measurementNoiseCov = cv::Mat::eye(n, n, CV_32F);
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));
        kf.statePost = cv::Mat::zeros(2 * n, 1, CV_32F);
        kf.statePre = cv::Mat::zeros(2 * n, 1, CV_32F);
    };

    initOne(kf_L_samples_);
    initOne(kf_R_samples_);

    meas_samples_L_ = cv::Mat::zeros(n, 1, CV_32F);
    meas_samples_R_ = cv::Mat::zeros(n, 1, CV_32F);
    pred_samples_L_ = cv::Mat::zeros(2 * n, 1, CV_32F);
    pred_samples_R_ = cv::Mat::zeros(2 * n, 1, CV_32F);
    estimated_samples_L_ = cv::Mat::zeros(2 * n, 1, CV_32F);
    estimated_samples_R_ = cv::Mat::zeros(2 * n, 1, CV_32F);

    kf_samples_initialized_L_ = false;
    kf_samples_initialized_R_ = false;
    sample_miss_L_ = 0;
    sample_miss_R_ = 0;
}

bool LaneLineDetector::hasValidSampleKfNoiseConfig() const
{
    const size_t n = config_.kf_sample_x_m.size();
    return n >= 4 &&
           config_.kf_sample_Q_pos_list.size() == n &&
           config_.kf_sample_Q_vel_list.size() == n &&
           config_.kf_sample_R_diag_list.size() == n;
}

void LaneLineDetector::warnSampleKfNoiseConfigFallback(const char* context)
{
    if (sample_kf_noise_config_warned_) return;
    sample_kf_noise_config_warned_ = true;
    std::cout << "[warn] sample KF disabled in " << context
              << ": Q/R list size mismatch with kf_sample_x_m"
              << " (x=" << config_.kf_sample_x_m.size()
              << ", Qpos=" << config_.kf_sample_Q_pos_list.size()
              << ", Qvel=" << config_.kf_sample_Q_vel_list.size()
              << ", R=" << config_.kf_sample_R_diag_list.size()
              << "), fallback to raw fit" << std::endl;
}

void LaneLineDetector::preprocess(cv::Mat& src, cv::Mat& dst_bev_bin) 
{
    /* IPM */
    cv::warpPerspective(src, bev, H_bev, config_.bev_size, cv::INTER_LINEAR);
    if (config_.show_bev) pushDebugImage(bev);

    /* HSL 색상 필터링 : white, yellow 찾기 */
    cv::Mat hls, mask_yellow, mask_white, mask_color;
    cv::cvtColor(bev, hls, cv::COLOR_BGR2HLS); 

    cv::inRange(hls, 
                cv::Scalar(config_.hls_white_hmin, config_.hls_white_lmin, config_.hls_white_smin), 
                cv::Scalar(config_.hls_white_hmax, config_.hls_white_lmax, config_.hls_white_smax), 
                mask_white); // 흰색
    cv::inRange(hls, 
                cv::Scalar(config_.hls_yellow_hmin, config_.hls_yellow_lmin, config_.hls_yellow_smin), 
                cv::Scalar(config_.hls_yellow_hmax, config_.hls_yellow_lmax, config_.hls_yellow_smax), 
                mask_yellow); // 노란색

    cv::bitwise_or(mask_white, mask_yellow, mask_color);
    
    if (config_.show_hls) {
        // pushDebugImage(hls);
        std::vector<cv::Mat> hls_channels;
        cv::split(hls, hls_channels);
        pushDebugImage(hls_channels[1]); // L채널(밝기)만 흑백으로 보기
        pushDebugImage(mask_white);  
        pushDebugImage(mask_yellow); 
        pushDebugImage(mask_color); 
    }

    /* sobel 엣지: color mask 영역에서만 수직 성분 강조 */
    cv::Mat sobel_x, sobel_abs, mask_edge, gray_masked;
    cv::cvtColor(bev, gray, cv::COLOR_BGR2GRAY);
    cv::bitwise_and(gray, gray, gray_masked, mask_color); // color mask 내부만 유지
    cv::GaussianBlur(gray_masked, gray_masked, cv::Size(5,5), 0);

    cv::Sobel(gray_masked, sobel_x, CV_16S, 1, 0, 3);
    cv::convertScaleAbs(sobel_x, sobel_abs);
    if (config_.show_sobel) pushDebugImage(sobel_abs); 
    
    cv::threshold(sobel_abs, mask_edge, config_.sobel_thres, 255, cv::THRESH_BINARY);

    /* [옵션] adaptive threshold: sobel_abs에 적용 후 OR (기본 OFF) */
    if (config_.preprocess_adaptive_th) {
        cv::Mat mask_edge_adapt;
        int adaptive_block_size = std::max(3, config_.adaptive_block_size);
        if (adaptive_block_size % 2 == 0) adaptive_block_size += 1;
        cv::adaptiveThreshold(sobel_abs, mask_edge_adapt, 255,
            cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,
            adaptive_block_size, config_.adaptive_C);
        cv::bitwise_or(mask_edge, mask_edge_adapt, mask_edge);
    }

    if (config_.show_sobel) pushDebugImage(mask_edge);

    cv::dilate(mask_edge, mask_edge, kernel_vert);

    /* 결합: 현재는 edge만 사용 */
    mask_edge.copyTo(dst_bev_bin);

    /* 사다리꼴 ROI 마스크 적용 */
    applyRoiMask(dst_bev_bin);

    /* 후처리: 작은 노이즈 제거 */
    cv::morphologyEx(dst_bev_bin, dst_bev_bin, cv::MORPH_OPEN, kernel_noise);

    /* [옵션] morphology close: 끊긴 차선 연결 (기본 OFF) */
    if (config_.preprocess_morph_close) {
        cv::morphologyEx(dst_bev_bin, dst_bev_bin, cv::MORPH_CLOSE, kernel_close);
    }

    /* [옵션] contour filter: 소형 블롭 제거 (기본 OFF) */
    if (config_.preprocess_contour_filter) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dst_bev_bin.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::Mat filtered = cv::Mat::zeros(dst_bev_bin.size(), CV_8U);
        for (const auto& cnt : contours) {
            double area = cv::contourArea(cnt);
            if (area < config_.contour_min_area) continue;
            cv::Rect br = cv::boundingRect(cnt);
            double ratio = (br.height > 0) ? (double)br.height / br.width : 0;
            if (ratio < config_.contour_min_ratio) continue;
            cv::drawContours(filtered, std::vector<std::vector<cv::Point>>{cnt}, -1, 255, cv::FILLED);
        }
        dst_bev_bin = filtered;
    }

    if (config_.show_bev_bin) {
        // 표시용: SW base histogram 기준 영역(하단 1/N) 시작 위치 라인
        cv::Mat dst_bev_bin_vis = dst_bev_bin.clone();
        int hist_divisor = std::max(1, config_.sw_hist_bottom_divisor);
        int hist_h = std::max(1, dst_bev_bin.rows / hist_divisor);
        int hist_start_row = dst_bev_bin.rows - hist_h;
        cv::line(dst_bev_bin_vis,
                 cv::Point(0, hist_start_row),
                 cv::Point(dst_bev_bin_vis.cols - 1, hist_start_row),
                 cv::Scalar(70), 2);
        pushDebugImageExec(dst_bev_bin_vis, "dst_bev_bin");
    }

}

void LaneLineDetector::findPixels(const cv::Mat& bev_bin)
{
    pixels_L_.clear();
    pixels_R_.clear();

    // 공통 입력 추출: non-zero 픽셀은 한 번만 계산
    std::vector<cv::Point> nonzero;
    cv::findNonZero(bev_bin, nonzero);

    // [상태 전이] INIT 판정: 이전 계수 없으면 전체 탐색(SW)
    bool first_frame = (last_coeffs_L_ == cv::Vec4d(0,0,0,0)) ||
                       (last_coeffs_R_ == cv::Vec4d(0,0,0,0));

    // [상태 전이] RECOVERY 판정: 품질 저하/미검출 누적이면 SW 복구 탐색
    bool need_recovery =
        (prev_state_L_ == LaneState::BAD || prev_state_L_ == LaneState::NODET ||
         prev_state_R_ == LaneState::BAD || prev_state_R_ == LaneState::NODET ||
         bad_streak_L_ >= config_.bad_to_sw || bad_streak_R_ >= config_.bad_to_sw ||
         weak_streak_L_ >= config_.weak_to_sw || weak_streak_R_ >= config_.weak_to_sw);

    const bool use_sw = (first_frame || need_recovery);

    if (config_.debug_mode_select && config_.log_frame_debug) {
        printf("[Mode] use=%s  weakStreak(L/R)=%d/%d  badStreak(L/R)=%d/%d  prevState(L/R)=%s/%s  prevReal(L/R)=%d/%d\n",
            use_sw ? "SW" : "SA",
            weak_streak_L_, weak_streak_R_,
            bad_streak_L_, bad_streak_R_,
            stateToString(prev_state_L_).c_str(), stateToString(prev_state_R_).c_str(),
            prev_real_L_, prev_real_R_);
    }

    // 상태 기반 전략 분기
    if (use_sw) {
        runStripHistogramGuidedSearch(bev_bin, nonzero);
    } else {
        runSearchAround(bev_bin, nonzero);
    }
}


bool LaneLineDetector::findPixelsFromMask(const cv::Mat& lane_mask)
{
    pixels_L_.clear();
    pixels_R_.clear();
    resetAllSampleSlotAccumulators();

    const LaneTrackMode mode_L = decideLaneTrackMode(LaneSideMask::Left);
    const LaneTrackMode mode_R = decideLaneTrackMode(LaneSideMask::Right);
    if (config_.debug_mode_select && config_.log_frame_debug) {
        printf("[MaskMode] mode(L/R)=%s/%s weakStreak(L/R)=%d/%d badStreak(L/R)=%d/%d prevState(L/R)=%s/%s\n",
               mode_L == LaneTrackMode::BLOB_SEEDED ? "BLOB_SEEDED" : "PREV_SEEDED",
               mode_R == LaneTrackMode::BLOB_SEEDED ? "BLOB_SEEDED" : "PREV_SEEDED",
               weak_streak_L_, weak_streak_R_,
               bad_streak_L_, bad_streak_R_,
               stateToString(prev_state_L_).c_str(), stateToString(prev_state_R_).c_str());
    }

    cv::Mat H_bev2mask;
    if (!H_mask2bev_.empty()) {
        H_bev2mask = H_mask2bev_.inv();
    }

    StartBlob blob_L, blob_R;
    selectStartBlobsForTracking(lane_mask, blob_L, blob_R, false);
    LaneTrackStart start_L, start_R;
    LaneTrackMode actual_mode_L = mode_L;
    LaneTrackMode actual_mode_R = mode_R;
    const bool built_L = buildLaneTrackStartState(lane_mask, LaneSideMask::Left, mode_L, H_bev2mask, blob_L, blob_R, start_L);
    const bool built_R = buildLaneTrackStartState(lane_mask, LaneSideMask::Right, mode_R, H_bev2mask, blob_L, blob_R, start_R);

    if (!built_L && mode_L == LaneTrackMode::PREV_SEEDED) {
        actual_mode_L = LaneTrackMode::BLOB_SEEDED;
        buildLaneTrackStartState(lane_mask, LaneSideMask::Left, actual_mode_L, H_bev2mask, blob_L, blob_R, start_L);
    }
    if (!built_R && mode_R == LaneTrackMode::PREV_SEEDED) {
        actual_mode_R = LaneTrackMode::BLOB_SEEDED;
        buildLaneTrackStartState(lane_mask, LaneSideMask::Right, actual_mode_R, H_bev2mask, blob_L, blob_R, start_R);
    }

    const int stop_line_y = sw_forward_limit_y_cache_;
    std::vector<WindowTrackDebug> debug_L, debug_R;
    LaneTrackStart first_valid_L, first_valid_R;
    if (start_L.valid) {
        runLaneTrackContinuous(lane_mask, LaneSideMask::Left, actual_mode_L, start_L, pixels_L_, stop_line_y,
                               (config_.show_sw || config_.show_sa_anchor) ? &debug_L : nullptr,
                               &first_valid_L);
    }
    if (start_R.valid) {
        runLaneTrackContinuous(lane_mask, LaneSideMask::Right, actual_mode_R, start_R, pixels_R_, stop_line_y,
                               (config_.show_sw || config_.show_sa_anchor) ? &debug_R : nullptr,
                               &first_valid_R);
    }

    const int min_valid = std::max(1, config_.sa_anchor_min_valid_points);
    bool side_ok_L = start_L.valid && (int)pixels_L_.size() >= min_valid;
    bool side_ok_R = start_R.valid && (int)pixels_R_.size() >= min_valid;

    if (!side_ok_L && actual_mode_L == LaneTrackMode::PREV_SEEDED && config_.sa_anchor_fallback_to_sw) {
        LaneTrackStart blob_start_L;
        if (buildLaneTrackStartState(lane_mask, LaneSideMask::Left, LaneTrackMode::BLOB_SEEDED, H_bev2mask, blob_L, blob_R, blob_start_L)) {
            pixels_L_.clear();
            resetSampleSlotAccumulator(LaneSideMask::Left);
            debug_L.clear();
            first_valid_L = LaneTrackStart();
            runLaneTrackContinuous(lane_mask, LaneSideMask::Left, LaneTrackMode::BLOB_SEEDED, blob_start_L, pixels_L_, stop_line_y,
                                   (config_.show_sw || config_.show_sa_anchor) ? &debug_L : nullptr,
                                   &first_valid_L);
            side_ok_L = (int)pixels_L_.size() >= min_valid;
            actual_mode_L = LaneTrackMode::BLOB_SEEDED;
        }
    }
    if (!side_ok_R && actual_mode_R == LaneTrackMode::PREV_SEEDED && config_.sa_anchor_fallback_to_sw) {
        LaneTrackStart blob_start_R;
        if (buildLaneTrackStartState(lane_mask, LaneSideMask::Right, LaneTrackMode::BLOB_SEEDED, H_bev2mask, blob_L, blob_R, blob_start_R)) {
            pixels_R_.clear();
            resetSampleSlotAccumulator(LaneSideMask::Right);
            debug_R.clear();
            first_valid_R = LaneTrackStart();
            runLaneTrackContinuous(lane_mask, LaneSideMask::Right, LaneTrackMode::BLOB_SEEDED, blob_start_R, pixels_R_, stop_line_y,
                                   (config_.show_sw || config_.show_sa_anchor) ? &debug_R : nullptr,
                                   &first_valid_R);
            side_ok_R = (int)pixels_R_.size() >= min_valid;
            actual_mode_R = LaneTrackMode::BLOB_SEEDED;
        }
    }

    if (!side_ok_L) pixels_L_.clear();
    if (!side_ok_R) pixels_R_.clear();

    if (side_ok_L && first_valid_L.valid) {
        prev_start_L_ = first_valid_L.center;
        prev_dir_L_ = first_valid_L.dir;
        has_prev_start_L_ = true;
    }
    if (side_ok_R && first_valid_R.valid) {
        prev_start_R_ = first_valid_R.center;
        prev_dir_R_ = first_valid_R.dir;
        has_prev_start_R_ = true;
    }

    if (config_.show_sw || config_.show_sa_anchor) {
        cv::Mat vis_track;
        cv::cvtColor(lane_mask, vis_track, cv::COLOR_GRAY2BGR);
        drawTrackingWindowDebug(vis_track, debug_L, LaneSideMask::Left, actual_mode_L);
        drawTrackingWindowDebug(vis_track, debug_R, LaneSideMask::Right, actual_mode_R);
        if (start_L.valid) cv::circle(vis_track, cv::Point(cvRound(start_L.center.x), cvRound(start_L.center.y)), 3, cv::Scalar(90, 180, 90), cv::FILLED);
        if (start_R.valid) cv::circle(vis_track, cv::Point(cvRound(start_R.center.x), cvRound(start_R.center.y)), 3, cv::Scalar(90, 180, 90), cv::FILLED);
        for (const auto& p : pixels_L_) cv::circle(vis_track, p, 1, cv::Scalar(255, 0, 0), cv::FILLED);
        for (const auto& p : pixels_R_) cv::circle(vis_track, p, 1, cv::Scalar(0, 0, 255), cv::FILLED);
        const std::string mode_text =
            std::string("mode L/R: ") +
            (actual_mode_L == LaneTrackMode::BLOB_SEEDED ? "BLOB" : "PREV") + "/" +
            (actual_mode_R == LaneTrackMode::BLOB_SEEDED ? "BLOB" : "PREV");
        cv::putText(vis_track, mode_text, cv::Point(10, 24),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(180, 255, 180), 2);
        pushDebugImageExec(vis_track, "search_around_mask");
    }

    return !(side_ok_L || side_ok_R);
}

void LaneLineDetector::sampleAnchorPointsVehicle(const cv::Vec4d& coeffs,
                                                 std::vector<cv::Point2d>& out_veh) const
{
    out_veh.clear();
    if (coeffs == cv::Vec4d(0, 0, 0, 0)) return;

    const double x_min_rear = config_.roi_xmin + kRearOriginShiftM;
    const double x_max_rear = config_.roi_xmax + kRearOriginShiftM;
    const double x_start = std::max(x_min_rear, config_.sa_anchor_x_start_m + kRearOriginShiftM);
    double x_end = std::min(x_max_rear, config_.sa_anchor_x_end_m + kRearOriginShiftM);
    if (config_.sw_forward_limit_m > 0.0) {
        x_end = std::min(x_end, config_.sw_forward_limit_m + kRearOriginShiftM);
    }
    const double step = std::max(0.2, config_.sa_anchor_x_step_m);
    if (x_end <= x_start + 1e-6) return;

    out_veh.reserve((size_t)((x_end - x_start) / step) + 4);
    for (double x = x_start; x <= x_end + 1e-9; x += step) {
        const double y = coeffs[0] * x * x * x + coeffs[1] * x * x + coeffs[2] * x + coeffs[3];
        if (!std::isfinite(y)) continue;
        out_veh.emplace_back(x, y);
    }
}

void LaneLineDetector::projectVehicleAnchorsToMask(const std::vector<cv::Point2d>& in_veh,
                                                   const cv::Mat& H_bev2mask,
                                                   const cv::Size& mask_size,
                                                   std::vector<cv::Point2f>& out_mask) const
{
    out_mask.clear();
    if (in_veh.empty() || H_bev2mask.empty()) return;

    std::vector<cv::Point2f> bev_pts;
    bev_pts.reserve(in_veh.size());
    for (const auto& p : in_veh) {
        const cv::Point2d bev = vehicleToBevPix(p);
        if (!std::isfinite(bev.x) || !std::isfinite(bev.y)) continue;
        bev_pts.emplace_back((float)bev.x, (float)bev.y);
    }
    if (bev_pts.empty()) return;

    std::vector<cv::Point2f> mask_pts;
    cv::perspectiveTransform(bev_pts, mask_pts, H_bev2mask);

    out_mask.reserve(mask_pts.size());
    for (const auto& p : mask_pts) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
        if (p.x < 0.0f || p.x >= (float)mask_size.width || p.y < 0.0f || p.y >= (float)mask_size.height) continue;
        out_mask.push_back(p);
    }
}

void LaneLineDetector::collectCentroidsFromAnchorWindows(const cv::Mat& lane_mask,
                                                         const std::vector<cv::Point2f>& anchors,
                                                         std::vector<cv::Point>& out_pts,
                                                         std::vector<cv::Rect>* debug_wins) const
{
    const int win_w = std::max(3, config_.sa_anchor_win_w_px);
    const int win_h = std::max(3, config_.sa_anchor_win_h_px);
    const int strips = std::max(1, config_.sa_anchor_strip_count);
    const int strip_minpix = std::max(1, config_.sa_anchor_strip_minpix);

    out_pts.clear();
    out_pts.reserve(anchors.size() * (size_t)strips);

    for (const auto& a : anchors) {
        const int cx = cvRound(a.x);
        const int cy = cvRound(a.y);

        const int x1 = std::max(0, cx - win_w / 2);
        const int x2 = std::min(lane_mask.cols, x1 + win_w);
        const int y1 = std::max(0, cy - win_h / 2);
        const int y2 = std::min(lane_mask.rows, y1 + win_h);
        if (x1 >= x2 || y1 >= y2) continue;

        if (debug_wins) debug_wins->emplace_back(x1, y1, x2 - x1, y2 - y1);

        const int h = y2 - y1;
        const int strip_h = std::max(1, h / strips);

        for (int s = 0; s < strips; ++s) {
            const int sy1 = y1 + s * strip_h;
            const int sy2 = (s == strips - 1) ? y2 : std::min(y2, sy1 + strip_h);
            if (sy1 >= sy2) continue;

            long long cnt = 0, sx = 0, sy = 0;
            for (int y = sy1; y < sy2; ++y) {
                const uchar* row = lane_mask.ptr<uchar>(y);
                for (int x = x1; x < x2; ++x) {
                    if (!row[x]) continue;
                    ++cnt;
                    sx += x;
                    sy += y;
                }
            }

            if (cnt < strip_minpix) continue;
            out_pts.emplace_back((int)(sx / cnt), (int)(sy / cnt));
        }
    }
}

void LaneLineDetector::buildMaskPredictionLutFromCoeffs(const cv::Vec4d& coeffs,
                                                        const cv::Mat& H_bev2mask,
                                                        int mask_h, int mask_w,
                                                        std::vector<int>& out_u) const
{
    out_u.assign(mask_h, -1);
    if (mask_h <= 0 || mask_w <= 0 || H_bev2mask.empty()) return;
    if (coeffs == cv::Vec4d(0, 0, 0, 0)) return;

    const double x_min = config_.roi_xmin + kRearOriginShiftM;
    double x_max = config_.roi_xmax + kRearOriginShiftM;
    if (config_.sw_forward_limit_m > 0.0) {
        x_max = std::min(x_max, config_.sw_forward_limit_m + kRearOriginShiftM);
    }
    if (x_max <= x_min + 1e-6) return;

    constexpr double kDxM = 0.25;
    std::vector<cv::Point2f> bev_pts;
    bev_pts.reserve((size_t)((x_max - x_min) / kDxM) + 8);
    for (double x_veh = x_min; x_veh <= x_max; x_veh += kDxM) {
        const double y_veh = coeffs[0] * x_veh * x_veh * x_veh
                           + coeffs[1] * x_veh * x_veh
                           + coeffs[2] * x_veh
                           + coeffs[3];
        const cv::Point2d bev = vehicleToBevPix(cv::Point2d(x_veh, y_veh));
        if (!std::isfinite(bev.x) || !std::isfinite(bev.y)) continue;
        bev_pts.emplace_back((float)bev.x, (float)bev.y);
    }
    if (bev_pts.size() < 2) return;

    std::vector<cv::Point2f> mask_pts;
    cv::perspectiveTransform(bev_pts, mask_pts, H_bev2mask);
    if (mask_pts.size() < 2) return;

    for (size_t i = 1; i < mask_pts.size(); ++i) {
        const cv::Point2f p0 = mask_pts[i - 1];
        const cv::Point2f p1 = mask_pts[i];
        if (!std::isfinite(p0.x) || !std::isfinite(p0.y) ||
            !std::isfinite(p1.x) || !std::isfinite(p1.y)) {
            continue;
        }

        const double dx = (double)p1.x - (double)p0.x;
        const double dy = (double)p1.y - (double)p0.y;
        const int steps = std::max(1, (int)std::max(std::abs(dx), std::abs(dy)));
        for (int s = 0; s <= steps; ++s) {
            const double t = (double)s / (double)steps;
            const int x = cvRound((double)p0.x + dx * t);
            const int y = cvRound((double)p0.y + dy * t);
            if (x < 0 || x >= mask_w || y < 0 || y >= mask_h) continue;
            if (out_u[y] < 0) out_u[y] = x;
            else out_u[y] = (out_u[y] + x) / 2;
        }
    }

    int last = -1;
    for (int y = 0; y < mask_h; ++y) {
        if (out_u[y] >= 0) last = out_u[y];
        else if (last >= 0) out_u[y] = last;
    }
    last = -1;
    for (int y = mask_h - 1; y >= 0; --y) {
        if (out_u[y] >= 0) last = out_u[y];
        else if (last >= 0) out_u[y] = last;
    }
}

LaneLineDetector::WindowPixelStats LaneLineDetector::computeWindowPixelStats(const cv::Mat& lane_mask,
                                                                             const cv::Rect& win_rect,
                                                                             int min_required_pixels) const
{
    WindowPixelStats stats;
    if (lane_mask.empty() || win_rect.width <= 0 || win_rect.height <= 0) return stats;

    const int x1 = std::max(0, win_rect.x);
    const int x2 = std::min(lane_mask.cols, win_rect.x + win_rect.width);
    const int y1 = std::max(0, win_rect.y);
    const int y2 = std::min(lane_mask.rows, win_rect.y + win_rect.height);
    if (x1 >= x2 || y1 >= y2) return stats;

    long long cnt = 0, sx = 0, sy = 0;
    double m20 = 0.0, m11 = 0.0, m02 = 0.0;
    for (int y = y1; y < y2; ++y) {
        const uchar* row = lane_mask.ptr<uchar>(y);
        for (int x = x1; x < x2; ++x) {
            if (!row[x]) continue;
            ++cnt;
            sx += x;
            sy += y;
            m20 += (double)x * x;
            m11 += (double)x * y;
            m02 += (double)y * y;
        }
    }

    stats.pixel_count = (int)cnt;
    if (cnt < min_required_pixels) return stats;

    const float cx = (float)sx / (float)cnt;
    const float cy = (float)sy / (float)cnt;
    stats.centroid = {cx, cy};

    const double inv_n = 1.0 / (double)cnt;
    const double cxx = m20 * inv_n - (double)cx * cx;
    const double cxy = m11 * inv_n - (double)cx * cy;
    const double cyy = m02 * inv_n - (double)cy * cy;
    const double tr = cxx + cyy;
    const double det = cxx * cyy - cxy * cxy;
    double disc = tr * tr * 0.25 - det;
    if (disc < 0.0) disc = 0.0;
    const double lam1 = tr * 0.5 + std::sqrt(disc);
    const double ddx = lam1 - cyy;
    const double ddy = cxy;
    const double len = std::sqrt(ddx * ddx + ddy * ddy);
    if (len > 1e-6) {
        stats.pca_dir = {(float)(ddx / len), (float)(ddy / len)};
        if (stats.pca_dir.y > 0.0f) {
            stats.pca_dir.x = -stats.pca_dir.x;
            stats.pca_dir.y = -stats.pca_dir.y;
        }
    }

    stats.has_enough_pixels = true;
    return stats;
}

LaneLineDetector::HorizontalEdgeSample LaneLineDetector::findHorizontalInnerEdgeFromCentroid(
    const cv::Mat& lane_mask,
    const cv::Rect& win_rect,
    const WindowPixelStats& stats,
    LaneSideMask side) const
{
    HorizontalEdgeSample sample;
    if (!stats.has_enough_pixels || lane_mask.empty()) return sample;

    const int cx = cvRound(stats.centroid.x);
    const int cy = cvRound(stats.centroid.y);
    const int x1 = std::max(0, win_rect.x);
    const int x2 = std::min(lane_mask.cols, win_rect.x + win_rect.width);
    const int y1 = std::max(0, win_rect.y);
    const int y2 = std::min(lane_mask.rows, win_rect.y + win_rect.height);
    if (x1 >= x2 || y1 >= y2) return sample;

    std::vector<int> candidate_rows;
    candidate_rows.reserve((size_t)(y2 - y1));
    candidate_rows.push_back(std::clamp(cy, y1, y2 - 1));
    for (int d = 1; d < (y2 - y1); ++d) {
        const int ya = cy - d;
        const int yb = cy + d;
        if (ya >= y1) candidate_rows.push_back(ya);
        if (yb < y2) candidate_rows.push_back(yb);
    }

    const int min_run_px = std::max(1, config_.lane_edge_min_run_px);
    const int wall_margin = std::max(0, config_.lane_edge_wall_margin_px);
    for (int row_y : candidate_rows) {
        const std::vector<RowRun> runs = extractWhiteRunsInRow(lane_mask, win_rect, row_y, min_run_px);
        const int base_idx = pickBestRunIndexNearCentroid(runs, cx);
        if (base_idx < 0) continue;
        const int run_idx = refineRunIndexByDirection(
            runs,
            base_idx,
            side,
            config_.lane_edge_enable_run_hop,
            std::max(0, config_.lane_edge_max_hop_gap_px),
            std::max(1, config_.lane_edge_min_hop_run_width_px),
            std::max(0, config_.lane_edge_max_hops_per_row));

        const RowRun& run = runs[(size_t)run_idx];
        sample.valid = true;
        sample.centroid_px = cv::Point(cx, row_y);
        sample.horizontal_span_px = run.x_max - run.x_min + 1;
        if (side == LaneSideMask::Left) {
            sample.edge_px = cv::Point(run.x_max, row_y);
            sample.touched_window_wall = (run.x_max >= x2 - 1 - wall_margin);
        } else {
            sample.edge_px = cv::Point(run.x_min, row_y);
            sample.touched_window_wall = (run.x_min <= x1 + wall_margin);
        }
        return sample;
    }

    return sample;
}

bool LaneLineDetector::processTrackingWindow(const cv::Mat& lane_mask,
                                             LaneSideMask side,
                                             const cv::Rect& win_rect,
                                             int min_required_pixels,
                                             WindowPixelStats& out_stats,
                                             HorizontalEdgeSample& out_sample) const
{
    out_stats = computeWindowPixelStats(lane_mask, win_rect, min_required_pixels);
    if (!out_stats.has_enough_pixels) {
        out_sample = HorizontalEdgeSample();
        return false;
    }
    out_sample = findHorizontalInnerEdgeFromCentroid(lane_mask, win_rect, out_stats, side);
    return out_sample.valid;
}

cv::Size LaneLineDetector::computeSaWindowSizeForStep(int step_idx) const
{
    const int init_w = std::max(config_.sa_min_win_w_px, config_.sa_init_win_w_px);
    const int init_h = std::max(config_.sa_min_win_h_px, config_.sa_init_win_h_px);
    const int w = std::max(config_.sa_min_win_w_px, init_w - step_idx * config_.sa_win_shrink_w_px_per_step);
    const int h = std::max(config_.sa_min_win_h_px, init_h - step_idx * config_.sa_win_shrink_h_px_per_step);
    return cv::Size(w, h);
}

cv::Point2f LaneLineDetector::computeNextWindowBottomCenterForSA(const cv::Rect&,
                                                                 const HorizontalEdgeSample& sample,
                                                                 int) const
{
    return cv::Point2f((float)sample.edge_px.x, (float)sample.edge_px.y);
}

bool LaneLineDetector::projectSaNearStartFromCoeff(const cv::Vec4d& coeffs,
                                                   const cv::Mat& H_bev2mask,
                                                   const cv::Size& mask_size,
                                                   cv::Point2f& out_center,
                                                   cv::Point2f& out_dir) const
{
    out_center = cv::Point2f();
    out_dir = cv::Point2f(0.0f, -1.0f);
    if (coeffs == cv::Vec4d(0, 0, 0, 0) || H_bev2mask.empty()) return false;

    const double x_start = config_.roi_xmin + kRearOriginShiftM;
    double x_end = x_start + std::max(0.0, config_.sa_seed_track_length_m);
    if (config_.sw_forward_limit_m > 0.0) {
        x_end = std::min(x_end, config_.sw_forward_limit_m + kRearOriginShiftM);
    }
    if (x_end <= x_start + 1e-6) return false;

    constexpr double kSearchStepM = 0.25;
    std::vector<cv::Point2f> mask_pts;
    mask_pts.reserve((size_t)((x_end - x_start) / kSearchStepM) + 4);

    for (double x = x_start; x <= x_end + 1e-9; x += kSearchStepM) {
        const double y = coeffs[0] * x * x * x + coeffs[1] * x * x + coeffs[2] * x + coeffs[3];
        if (!std::isfinite(y)) continue;
        const cv::Point2d bev = vehicleToBevPix(cv::Point2d(x, y));
        if (!std::isfinite(bev.x) || !std::isfinite(bev.y)) continue;
        std::vector<cv::Point2f> bev_pts = {cv::Point2f((float)bev.x, (float)bev.y)};
        std::vector<cv::Point2f> proj_pts;
        cv::perspectiveTransform(bev_pts, proj_pts, H_bev2mask);
        if (proj_pts.empty()) continue;
        const cv::Point2f& p = proj_pts.front();
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
        if (p.x < 0.0f || p.x >= (float)mask_size.width || p.y < 0.0f || p.y >= (float)mask_size.height) continue;
        mask_pts.push_back(p);
    }
    if (mask_pts.empty()) return false;

    out_center = mask_pts.front();
    if (mask_pts.size() >= 2) {
        cv::Point2f dir = mask_pts[1] - mask_pts[0];
        const float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (len > 1e-6f) {
            dir.x /= len;
            dir.y /= len;
            if (dir.y > 0.0f) {
                dir.x = -dir.x;
                dir.y = -dir.y;
            }
            out_dir = dir;
        }
    }
    return true;
}

bool LaneLineDetector::buildSaStartState(const cv::Mat& lane_mask,
                                         LaneSideMask side,
                                         const cv::Mat& H_bev2mask,
                                         cv::Point2f& out_center,
                                         cv::Point2f& out_dir) const
{
    out_center = cv::Point2f();
    out_dir = cv::Point2f(0.0f, -1.0f);

    const bool has_prev_start = (side == LaneSideMask::Left) ? has_prev_start_L_ : has_prev_start_R_;
    const cv::Point2f prev_start = (side == LaneSideMask::Left) ? prev_start_L_ : prev_start_R_;
    const cv::Point2f prev_dir = (side == LaneSideMask::Left) ? prev_dir_L_ : prev_dir_R_;
    const cv::Vec4d coeffs = (side == LaneSideMask::Left) ? last_coeffs_L_ : last_coeffs_R_;

    if (config_.sa_seed_use_prev_start &&
        has_prev_start &&
        std::isfinite(prev_start.x) && std::isfinite(prev_start.y) &&
        prev_start.x >= 0.0f && prev_start.x < (float)lane_mask.cols &&
        prev_start.y >= 0.0f && prev_start.y < (float)lane_mask.rows) {
        out_center = prev_start;
        out_dir = prev_dir;
        const float len = std::sqrt(out_dir.x * out_dir.x + out_dir.y * out_dir.y);
        if (len > 1e-6f) {
            out_dir.x /= len;
            out_dir.y /= len;
            if (out_dir.y > 0.0f) {
                out_dir.x = -out_dir.x;
                out_dir.y = -out_dir.y;
            }
        } else {
            out_dir = cv::Point2f(0.0f, -1.0f);
        }
        return true;
    }

    return projectSaNearStartFromCoeff(coeffs, H_bev2mask, lane_mask.size(), out_center, out_dir);
}

LaneTrackMode LaneLineDetector::decideLaneTrackMode(LaneSideMask side) const
{
    const cv::Vec4d coeffs = (side == LaneSideMask::Left) ? last_coeffs_L_ : last_coeffs_R_;
    const LaneState prev_state = (side == LaneSideMask::Left) ? prev_state_L_ : prev_state_R_;
    const int bad_streak = (side == LaneSideMask::Left) ? bad_streak_L_ : bad_streak_R_;
    const int weak_streak = (side == LaneSideMask::Left) ? weak_streak_L_ : weak_streak_R_;
    const bool no_coeff = (coeffs == cv::Vec4d(0, 0, 0, 0));

    if (no_coeff) return LaneTrackMode::BLOB_SEEDED;
    if (prev_state == LaneState::BAD || prev_state == LaneState::NODET) return LaneTrackMode::BLOB_SEEDED;
    if (bad_streak >= config_.bad_to_sw) return LaneTrackMode::BLOB_SEEDED;
    if (weak_streak >= config_.weak_to_sw) return LaneTrackMode::BLOB_SEEDED;
    return LaneTrackMode::PREV_SEEDED;
}

bool LaneLineDetector::buildLaneTrackStartState(const cv::Mat& lane_mask,
                                                LaneSideMask side,
                                                LaneTrackMode mode,
                                                const cv::Mat& H_bev2mask,
                                                const StartBlob& blob_L,
                                                const StartBlob& blob_R,
                                                LaneTrackStart& out_start) const
{
    out_start = LaneTrackStart();
    if (mode == LaneTrackMode::BLOB_SEEDED) {
        const StartBlob& blob = (side == LaneSideMask::Left) ? blob_L : blob_R;
        if (!blob.valid) return false;
        out_start.center = blob.end_pt;
        out_start.dir = blob.dir_vec;
        out_start.valid = true;
        return true;
    }

    const bool has_prev_start = (side == LaneSideMask::Left) ? has_prev_start_L_ : has_prev_start_R_;
    const cv::Point2f prev_start = (side == LaneSideMask::Left) ? prev_start_L_ : prev_start_R_;
    if (has_prev_start) {
        const int img_h = lane_mask.rows;
        const int img_w = lane_mask.cols;
        const float half = (float)img_w * 0.5f;
        cv::Rect roi_bottom(0, img_h * 3 / 4, img_w, img_h / 4);
        std::vector<StartBlob> cands = findStartBlobsInRoi(lane_mask, roi_bottom);
        if (cands.empty()) {
            cv::Rect roi_fallback(0, img_h / 2, img_w, img_h / 4);
            cands = findStartBlobsInRoi(lane_mask, roi_fallback);
        }

        const float prev_x = prev_start.x;
        const float prior_half = (float)std::max(0, config_.prev_seed_start_prior_half_width_px);
        const float x_min = prev_x - prior_half;
        const float x_max = prev_x + prior_half;
        int best_idx = -1;
        float best_inward_x = (side == LaneSideMask::Left)
            ? std::numeric_limits<float>::lowest()
            : std::numeric_limits<float>::max();

        auto considerCandidate = [&](int idx, bool require_in_prior) {
            const StartBlob& cand = cands[(size_t)idx];
            const bool side_ok = (side == LaneSideMask::Left) ? (cand.end_pt.x < half) : (cand.end_pt.x >= half);
            if (!side_ok) return;
            const bool in_prior = (cand.end_pt.x >= x_min && cand.end_pt.x <= x_max);
            if (require_in_prior && !in_prior) return;

            const float inward_x = cand.end_pt.x;
            const bool better_inward = (side == LaneSideMask::Left)
                ? (inward_x > best_inward_x + 1e-3f)
                : (inward_x < best_inward_x - 1e-3f);
            if (better_inward) {
                best_idx = idx;
                best_inward_x = inward_x;
            }
        };

        for (int pass = 0; pass < 2 && best_idx < 0; ++pass) {
            const bool require_in_prior = (pass == 0);
            for (int i = 0; i < (int)cands.size(); ++i) {
                considerCandidate(i, require_in_prior);
            }
        }

        if (best_idx >= 0) {
            const StartBlob& cand = cands[(size_t)best_idx];
            out_start.center = cand.end_pt;
            out_start.dir = cand.dir_vec;
            out_start.valid = true;
            return true;
        }
    }

    if (H_bev2mask.empty()) return false;
    out_start.valid = buildSaStartState(lane_mask, side, H_bev2mask, out_start.center, out_start.dir);
    return out_start.valid;
}

void LaneLineDetector::buildSaSeedCentersFromPrevCoeffs(const cv::Vec4d& coeffs,
                                                        const cv::Mat& H_bev2mask,
                                                        const cv::Size& mask_size,
                                                        std::vector<cv::Point2f>& out_mask_pts) const
{
    out_mask_pts.clear();
    if (coeffs == cv::Vec4d(0, 0, 0, 0) || H_bev2mask.empty()) return;

    const double x_min_rear = std::max(config_.roi_xmin + kRearOriginShiftM,
                                       config_.sa_anchor_x_start_m + kRearOriginShiftM);
    double x_max_rear = std::min(config_.roi_xmax + kRearOriginShiftM,
                                 x_min_rear + std::max(0.0, config_.sa_seed_track_length_m));
    if (config_.sw_forward_limit_m > 0.0) {
        x_max_rear = std::min(x_max_rear, config_.sw_forward_limit_m + kRearOriginShiftM);
    }
    const double step_m = std::max(0.2, config_.sa_anchor_x_step_m);
    const int max_count = std::max(1, config_.sa_seed_count_max);
    if (x_max_rear <= x_min_rear + 1e-6) return;

    std::vector<cv::Point2f> bev_pts;
    bev_pts.reserve((size_t)max_count);
    for (int i = 0; i < max_count; ++i) {
        const double x = x_min_rear + step_m * (double)i;
        if (x > x_max_rear + 1e-9) break;
        const double y = coeffs[0] * x * x * x + coeffs[1] * x * x + coeffs[2] * x + coeffs[3];
        if (!std::isfinite(y)) continue;
        const cv::Point2d bev = vehicleToBevPix(cv::Point2d(x, y));
        if (!std::isfinite(bev.x) || !std::isfinite(bev.y)) continue;
        bev_pts.emplace_back((float)bev.x, (float)bev.y);
    }
    if (bev_pts.empty()) return;

    std::vector<cv::Point2f> mask_pts;
    cv::perspectiveTransform(bev_pts, mask_pts, H_bev2mask);
    for (const auto& p : mask_pts) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
        if (p.x < 0.0f || p.x >= (float)mask_size.width || p.y < 0.0f || p.y >= (float)mask_size.height) continue;
        out_mask_pts.push_back(p);
    }
}

void LaneLineDetector::runSaFromSeeds(const cv::Mat& lane_mask,
                                      LaneSideMask side,
                                      const std::vector<cv::Point2f>& seed_centers,
                                      std::vector<cv::Point>& out_samples,
                                      int stop_line_y,
                                      std::vector<WindowTrackDebug>* debug_steps) const
{
    out_samples.clear();
    if (seed_centers.empty()) return;

    const int img_h = lane_mask.rows;
    const int img_w = lane_mask.cols;
    const int kNoHitMax = 6;
    const int kMaxSteps = std::max(8, img_h / std::max(1, config_.win_h) + std::max(1, config_.sa_seed_count_max));
    int no_hit = 0;
    int step_idx = 0;
    cv::Point2f propagated_center = seed_centers.back();
    bool has_propagated_center = false;

    auto processOne = [&](const cv::Point2f& center, bool allow_record_center) {
        const cv::Size win_size = computeSaWindowSizeForStep(step_idx);
        const int x1 = std::max(0, cvRound(center.x - (float)win_size.width * 0.5f));
        const int x2 = std::min(img_w, x1 + win_size.width);
        const int y2 = std::min(img_h, cvRound(center.y));
        const int y1 = std::max(0, y2 - win_size.height);
        const cv::Rect win_rect(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));

        WindowPixelStats stats;
        HorizontalEdgeSample sample;
        const bool ok = processTrackingWindow(lane_mask, side, win_rect, std::max(1, config_.minpix), stats, sample);

        WindowTrackDebug dbg;
        dbg.rect = win_rect;
        dbg.valid = ok;
        if (stats.has_enough_pixels) dbg.centroid_px = cv::Point(cvRound(stats.centroid.x), cvRound(stats.centroid.y));
        if (sample.valid) dbg.edge_px = sample.edge_px;

        if (ok) {
            out_samples.push_back(sample.edge_px);
            no_hit = 0;
            const int next_h = computeSaWindowSizeForStep(step_idx + 1).height;
            propagated_center = computeNextWindowBottomCenterForSA(win_rect, sample, next_h);
            has_propagated_center = allow_record_center;
            dbg.next_center_px = cv::Point(cvRound(propagated_center.x), cvRound(propagated_center.y));
        } else {
            ++no_hit;
            dbg.next_center_px = cv::Point(cvRound(center.x), cvRound(center.y - win_size.height));
        }

        if (debug_steps) debug_steps->push_back(dbg);
        ++step_idx;
        return ok;
    };

    for (const auto& seed : seed_centers) {
        processOne(seed, true);
    }

    cv::Point2f center = has_propagated_center ? propagated_center : seed_centers.back();
    while (step_idx < kMaxSteps) {
        const cv::Size win_size = computeSaWindowSizeForStep(step_idx);
        const int y2 = std::min(img_h, cvRound(center.y));
        const int y1 = std::max(0, y2 - win_size.height);
        if (stop_line_y >= 0 && y1 <= stop_line_y) break;
        const bool ok = processOne(center, true);
        if (!ok) {
            if (no_hit > kNoHitMax) break;
            center.y -= (float)win_size.height;
            continue;
        }
        center = propagated_center;
    }
}

void LaneLineDetector::runSaTrackContinuous(const cv::Mat& lane_mask,
                                            LaneSideMask side,
                                            const cv::Point2f& start_center,
                                            const cv::Point2f& start_dir,
                                            std::vector<cv::Point>& out_samples,
                                            int stop_line_y,
                                            std::vector<WindowTrackDebug>* debug_steps) const
{
    out_samples.clear();
    if (lane_mask.empty()) return;

    const int img_h = lane_mask.rows;
    const int img_w = lane_mask.cols;
    const int kNoHitMax = 3;
    const int kMaxSteps = std::max(8, img_h / std::max(1, config_.win_h) + 8);

    cv::Point2f center = start_center;
    cv::Point2f dir = start_dir;
    const float dir_len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (dir_len > 1e-6f) {
        dir.x /= dir_len;
        dir.y /= dir_len;
    } else {
        dir = cv::Point2f(0.0f, -1.0f);
    }
    if (dir.y > 0.0f) {
        dir.x = -dir.x;
        dir.y = -dir.y;
    }

    int no_hit = 0;
    int step_idx = 0;
    while (step_idx < kMaxSteps) {
        const cv::Size win_size = computeSaWindowSizeForStep(step_idx);
        const int x1 = std::max(0, cvRound(center.x - (float)win_size.width * 0.5f));
        const int x2 = std::min(img_w, x1 + win_size.width);
        const int y2 = std::min(img_h, cvRound(center.y));
        const int y1 = std::max(0, y2 - win_size.height);
        const bool reached_forward_limit = (stop_line_y >= 0 && y1 <= stop_line_y);
        const cv::Rect win_rect(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));
        if (win_rect.width <= 0 || win_rect.height <= 0) break;

        WindowPixelStats stats;
        HorizontalEdgeSample sample;
        const bool ok = processTrackingWindow(lane_mask, side, win_rect, std::max(1, config_.minpix), stats, sample);

        WindowTrackDebug dbg;
        dbg.rect = win_rect;
        dbg.valid = ok;
        if (stats.has_enough_pixels) dbg.centroid_px = cv::Point(cvRound(stats.centroid.x), cvRound(stats.centroid.y));
        if (sample.valid) dbg.edge_px = sample.edge_px;

        if (!ok) {
            ++no_hit;
            dbg.next_center_px = cv::Point(cvRound(center.x + dir.x * (float)win_size.height),
                                           cvRound(center.y + dir.y * (float)win_size.height));
            if (debug_steps) debug_steps->push_back(dbg);
            if (no_hit > kNoHitMax || reached_forward_limit) break;
            center.x += dir.x * (float)win_size.height;
            center.y += dir.y * (float)win_size.height;
            ++step_idx;
            continue;
        }

        no_hit = 0;
        out_samples.push_back(sample.edge_px);
        if (stats.pca_dir.dot(dir) > 0.0f) {
            dir = stats.pca_dir;
        }
        center = computeNextWindowBottomCenterForSA(win_rect, sample, computeSaWindowSizeForStep(step_idx + 1).height);
        dbg.next_center_px = cv::Point(cvRound(center.x), cvRound(center.y));
        if (debug_steps) debug_steps->push_back(dbg);
        if (reached_forward_limit) break;
        ++step_idx;
    }
}

void LaneLineDetector::runLaneTrackContinuous(const cv::Mat& lane_mask,
                                              LaneSideMask side,
                                              LaneTrackMode mode,
                                              const LaneTrackStart& start,
                                              std::vector<cv::Point>& out_samples,
                                              int stop_line_y,
                                              std::vector<WindowTrackDebug>* debug_steps,
                                              LaneTrackStart* out_first_valid) const
{
    out_samples.clear();
    if (out_first_valid) *out_first_valid = LaneTrackStart();
    if (lane_mask.empty() || !start.valid) return;

    const int img_h = lane_mask.rows;
    const int img_w = lane_mask.cols;
    const int kNoHitMax = 3;
    const int kMaxSteps = std::max(8, img_h / std::max(1, config_.win_h) + 8);

    cv::Point2f center = start.center;
    cv::Point2f dir = start.dir;
    const float dir_len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (dir_len > 1e-6f) {
        dir.x /= dir_len;
        dir.y /= dir_len;
    } else {
        dir = cv::Point2f(0.0f, -1.0f);
    }
    if (dir.y > 0.0f) {
        dir.x = -dir.x;
        dir.y = -dir.y;
    }

    int no_hit = 0;
    bool stored_first_valid = false;
    for (int step_idx = 0; step_idx < kMaxSteps; ++step_idx) {
        const cv::Size win_size = (mode == LaneTrackMode::PREV_SEEDED)
            ? computeSaWindowSizeForStep(step_idx)
            : cv::Size(config_.win_w, config_.win_h);
        const int x1 = std::max(0, cvRound(center.x - (float)win_size.width * 0.5f));
        const int x2 = std::min(img_w, x1 + win_size.width);
        const int y2 = std::min(img_h, cvRound(center.y));
        const int y1 = std::max(0, y2 - win_size.height);
        const bool reached_forward_limit = (stop_line_y >= 0 && y1 <= stop_line_y);
        const cv::Rect win_rect(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));
        if (win_rect.width <= 0 || win_rect.height <= 0) break;

        WindowPixelStats stats;
        HorizontalEdgeSample sample;
        const bool ok = processTrackingWindow(lane_mask, side, win_rect, std::max(1, config_.minpix), stats, sample);

        WindowTrackDebug dbg;
        dbg.rect = win_rect;
        dbg.valid = ok;
        if (stats.has_enough_pixels) dbg.centroid_px = cv::Point(cvRound(stats.centroid.x), cvRound(stats.centroid.y));
        if (sample.valid) dbg.edge_px = sample.edge_px;

        if (!ok) {
            ++no_hit;
            center.x += dir.x * (float)win_size.height;
            center.y += dir.y * (float)win_size.height;
            dbg.next_center_px = cv::Point(cvRound(center.x), cvRound(center.y));
            if (debug_steps) debug_steps->push_back(dbg);
            if (no_hit > kNoHitMax || reached_forward_limit) break;
            continue;
        }

        no_hit = 0;
        cv::Point sample_pt(cvRound(stats.centroid.x), cvRound(stats.centroid.y));
        if (mode == LaneTrackMode::PREV_SEEDED || config_.use_edge_sample_in_sw) {
            sample_pt = sample.edge_px;
        }
        out_samples.push_back(sample_pt);
        accumulateSampleSlotFromWindow(side, win_rect, sample_pt);

        if (stats.pca_dir.dot(dir) > 0.0f) {
            dir = stats.pca_dir;
        }
        if (!stored_first_valid && out_first_valid) {
            out_first_valid->center = center;
            out_first_valid->dir = dir;
            out_first_valid->valid = true;
            stored_first_valid = true;
        }

        if (mode == LaneTrackMode::PREV_SEEDED) {
            center = computeNextWindowBottomCenterForSA(win_rect, sample, computeSaWindowSizeForStep(step_idx + 1).height);
        } else {
            center = cv::Point2f(stats.centroid.x + dir.x * (float)win_size.height,
                                 center.y + dir.y * (float)win_size.height);
        }
        dbg.next_center_px = cv::Point(cvRound(center.x), cvRound(center.y));
        if (debug_steps) debug_steps->push_back(dbg);
        if (reached_forward_limit) break;
    }
}

void LaneLineDetector::drawTrackingWindowDebug(cv::Mat& vis,
                                               const std::vector<WindowTrackDebug>& steps,
                                               LaneSideMask side,
                                               LaneTrackMode mode) const
{
    const cv::Scalar kBlobSeeded(0, 255, 0);
    const cv::Scalar kPrevSeeded(80, 180, 180);
    const cv::Scalar kRect = (mode == LaneTrackMode::BLOB_SEEDED) ? kBlobSeeded : kPrevSeeded;
    const cv::Scalar kCentroid = (mode == LaneTrackMode::BLOB_SEEDED) ? kBlobSeeded : kPrevSeeded;
    const cv::Scalar kEdge = (side == LaneSideMask::Left) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);

    for (const auto& step : steps) {
        cv::rectangle(vis, step.rect, kRect, 1);
        if (step.centroid_px != cv::Point()) cv::circle(vis, step.centroid_px, 2, kCentroid, cv::FILLED);
        if (step.edge_px != cv::Point()) {
            cv::circle(vis, step.edge_px, 2, kEdge, cv::FILLED);
            if (step.centroid_px != cv::Point()) cv::line(vis, step.centroid_px, step.edge_px, kEdge, 1);
        }
    }
}

bool LaneLineDetector::runSearchAroundMaskPath(const cv::Mat& lane_mask)
{
    if (lane_mask.empty() || H_mask2bev_.empty()) return false;

    cv::Mat H_bev2mask = H_mask2bev_.inv();
    if (H_bev2mask.empty()) return false;

    cv::Point2f start_center_L, start_dir_L, start_center_R, start_dir_R;
    const bool has_start_L = buildSaStartState(lane_mask, LaneSideMask::Left, H_bev2mask, start_center_L, start_dir_L);
    const bool has_start_R = buildSaStartState(lane_mask, LaneSideMask::Right, H_bev2mask, start_center_R, start_dir_R);

    std::vector<WindowTrackDebug> debug_L, debug_R;
    const int stop_line_y = sw_forward_limit_y_cache_;
    pixels_L_.clear();
    pixels_R_.clear();
    if (has_start_L) {
        runSaTrackContinuous(lane_mask, LaneSideMask::Left, start_center_L, start_dir_L, pixels_L_, stop_line_y,
                             (config_.show_sw || config_.show_sa_anchor) ? &debug_L : nullptr);
    }
    if (has_start_R) {
        runSaTrackContinuous(lane_mask, LaneSideMask::Right, start_center_R, start_dir_R, pixels_R_, stop_line_y,
                             (config_.show_sw || config_.show_sa_anchor) ? &debug_R : nullptr);
    }

    const int min_valid = std::max(1, config_.sa_anchor_min_valid_points);
    const bool side_ok_L = has_start_L && (int)pixels_L_.size() >= min_valid;
    const bool side_ok_R = has_start_R && (int)pixels_R_.size() >= min_valid;
    if (!side_ok_L) pixels_L_.clear();
    if (!side_ok_R) pixels_R_.clear();
    const bool sa_ok = side_ok_L || side_ok_R;

    if (config_.show_sw || config_.show_sa_anchor) {
        cv::Mat vis_sa;
        cv::cvtColor(lane_mask, vis_sa, cv::COLOR_GRAY2BGR);

        const cv::Scalar kBlue(255, 0, 0);
        const cv::Scalar kRed(0, 0, 255);
        const cv::Scalar kAnchorWin(90, 180, 90);

        drawTrackingWindowDebug(vis_sa, debug_L, LaneSideMask::Left, LaneTrackMode::PREV_SEEDED);
        drawTrackingWindowDebug(vis_sa, debug_R, LaneSideMask::Right, LaneTrackMode::PREV_SEEDED);
        if (has_start_L) cv::circle(vis_sa, cv::Point(cvRound(start_center_L.x), cvRound(start_center_L.y)), 3, kAnchorWin, cv::FILLED);
        if (has_start_R) cv::circle(vis_sa, cv::Point(cvRound(start_center_R.x), cvRound(start_center_R.y)), 3, kAnchorWin, cv::FILLED);

        for (const auto& p : pixels_L_) cv::circle(vis_sa, p, 1, kBlue, cv::FILLED);
        for (const auto& p : pixels_R_) cv::circle(vis_sa, p, 1, kRed, cv::FILLED);

        const std::string mode_text = sa_ok ? "mode: SA(track)" : "mode: SA(track)->SW fallback";
        cv::putText(vis_sa, mode_text, cv::Point(10, 24),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(180, 255, 180), 2);
        pushDebugImageExec(vis_sa, "search_around_mask");
    }

    return sa_ok;
}

int LaneLineDetector::computeSwForwardLimitY(const cv::Size& mask_size, const cv::Size& rect_size) const
{
    if (config_.sw_forward_limit_m <= 0.0) return -1;
    if (config_.sw_forward_limit_m > config_.roi_xmax) return -1;
    if (mask_size.width <= 0 || mask_size.height <= 0) return -1;
    if (calib_.H_i2w_rect.empty()) return -1;

    cv::Size rect_eval = rect_size;
    if (rect_eval.width <= 0 || rect_eval.height <= 0) {
        rect_eval = calib_.imageSize;
    }
    if (rect_eval.width <= 0 || rect_eval.height <= 0) {
        rect_eval = mask_size;
    }

    const double sx = static_cast<double>(rect_eval.width) / static_cast<double>(mask_size.width);
    const double sy = static_cast<double>(rect_eval.height) / static_cast<double>(mask_size.height);
    const double u_mid_rect = (static_cast<double>(mask_size.width) * 0.5) * sx;

    cv::Mat H_i2w;
    calib_.H_i2w_rect.convertTo(H_i2w, CV_64F);
    const double h00 = H_i2w.at<double>(0, 0), h01 = H_i2w.at<double>(0, 1), h02 = H_i2w.at<double>(0, 2);
    const double h20 = H_i2w.at<double>(2, 0), h21 = H_i2w.at<double>(2, 1), h22 = H_i2w.at<double>(2, 2);

    for (int y = mask_size.height - 1; y >= 0; --y) {
        const double v_rect = static_cast<double>(y) * sy;
        const double z = h20 * u_mid_rect + h21 * v_rect + h22;
        if (!std::isfinite(z) || std::abs(z) < 1e-12) continue;
        const double x_world = (h00 * u_mid_rect + h01 * v_rect + h02) / z;
        if (!std::isfinite(x_world)) continue;
        if (x_world >= config_.sw_forward_limit_m) {
            return y;
        }
    }

    return -1;
}

void LaneLineDetector::updateSwForwardLimitCache(const cv::Size& mask_size, const cv::Size& rect_size)
{
    const bool same_key =
        sw_forward_limit_cache_valid_ &&
        sw_forward_limit_mask_size_cache_ == mask_size &&
        sw_forward_limit_rect_size_cache_ == rect_size &&
        std::abs(sw_forward_limit_m_cache_ - config_.sw_forward_limit_m) < 1e-12;

    if (same_key) return;

    sw_forward_limit_y_cache_ = computeSwForwardLimitY(mask_size, rect_size);
    sw_forward_limit_mask_size_cache_ = mask_size;
    sw_forward_limit_rect_size_cache_ = rect_size;
    sw_forward_limit_m_cache_ = config_.sw_forward_limit_m;
    sw_forward_limit_cache_valid_ = true;
}

bool LaneLineDetector::runSwV2Path(const cv::Mat& lane_mask) // 차선 마스크에서 픽셀 찾기
{
    StartBlob blob_L, blob_R;
    if (selectStartBlobsForTracking(lane_mask, blob_L, blob_R, true))
        return true; // lane_lost

    cv::Mat sw_mask = lane_mask.clone();
    if (!sw_forward_limit_cache_valid_ ||
        sw_forward_limit_mask_size_cache_ != sw_mask.size() ||
        std::abs(sw_forward_limit_m_cache_ - config_.sw_forward_limit_m) >= 1e-12) {
        cv::Size rect_size = rect_ref_size_;
        if (rect_size.width <= 0 || rect_size.height <= 0) rect_size = calib_.imageSize;
        if (rect_size.width <= 0 || rect_size.height <= 0) rect_size = sw_mask.size();
        updateSwForwardLimitCache(sw_mask.size(), rect_size);
    }
    const int stop_line_y = sw_forward_limit_y_cache_;

    // // ── Sobel 가로 성분 제거: |Gy| > |Gx| 픽셀을 마스크에서 제외 ───────────
    // // 이진 마스크에 Sobel 직접 적용 시 내부 픽셀(이웃 전부 255)은 Gx=Gy=0이 되어
    // // 방향 판별 불가 → GaussianBlur로 먼저 퍼뜨려서 내부 픽셀에도 방향 정보 전파
    // cv::Mat sw_mask = lane_mask.clone();
    // {
    //     cv::Mat blurred, gx, gy, abs_gx, abs_gy, horiz_dominant;
    //     cv::GaussianBlur(lane_mask, blurred, cv::Size(0, 0), 7.5);
    //     cv::Sobel(blurred, gx, CV_16S, 1, 0, 3);
    //     cv::Sobel(blurred, gy, CV_16S, 0, 1, 3);
    //     cv::convertScaleAbs(gx, abs_gx);
    //     cv::convertScaleAbs(gy, abs_gy);
    //     cv::compare(abs_gy, abs_gx, horiz_dominant, cv::CMP_GT); // 가로 성분 우세 픽셀
    //     sw_mask.setTo(0, horiz_dominant);
    // }

    // ── 시작 blob 시각화 ────────────────────────────────────────────────────
    if (config_.show_sw) {

        if (config_.show_start_blob) {

            cv::Mat vis_start;
            cv::cvtColor(sw_mask, vis_start, cv::COLOR_GRAY2BGR);
            
            // 화살표 길이: 하단 ROI 높이(img_h/4)를 ±half_len으로 균등 분할
            const float half_len = static_cast<float>(lane_mask.rows) / 8.0f;
            const cv::Scalar kColorL(255, 0,   0); // 좌: 파란색
            const cv::Scalar kColorR(  0, 0, 255); // 우: 빨간색
            
            auto drawBlob = [&](const StartBlob& blob, const cv::Scalar& color) {
                if (!blob.valid) return;
                const cv::Point ctr(cvRound(blob.end_pt.x), cvRound(blob.end_pt.y));
                cv::circle(vis_start, ctr, 2, color, cv::FILLED);
                const cv::Point tail(cvRound(blob.end_pt.x - blob.dir_vec.x * half_len),
                cvRound(blob.end_pt.y - blob.dir_vec.y * half_len));
                const cv::Point tip (cvRound(blob.end_pt.x + blob.dir_vec.x * half_len),
                cvRound(blob.end_pt.y + blob.dir_vec.y * half_len));
                cv::arrowedLine(vis_start, tail, tip, color, 2, cv::LINE_AA, 0, 0.25);
            };
            drawBlob(blob_L, kColorL);
            drawBlob(blob_R, kColorR);
            
            // ROI 경계선 오버레이 (1/4 등분 기준)
            const cv::Scalar kGreenLine(0, 255, 0);
            const int roi_bottom_y   = lane_mask.rows * 3 / 4;  // 270: 하단 ROI 상단 경계
            const int roi_fallback_y = lane_mask.rows / 2;       // 180: fallback ROI 상단 경계
            cv::line(vis_start, cv::Point(0, roi_bottom_y),   cv::Point(lane_mask.cols, roi_bottom_y),   kGreenLine, 2);
            cv::line(vis_start, cv::Point(0, roi_fallback_y), cv::Point(lane_mask.cols, roi_fallback_y), kGreenLine, 2);
            
            pushDebugImageExec(vis_start, "sw_start");
        }

        // ── 슬라이딩 윈도우 시각화 ──────────────────────────────────────────
        cv::Mat vis_sw;
        cv::cvtColor(sw_mask, vis_sw, cv::COLOR_GRAY2BGR);
        const cv::Scalar kGreen(0, 255, 0);
        const cv::Scalar kGray(128, 128, 128);

        std::vector<cv::Rect> wins_L, wins_R;
        runSwV2(blob_L, sw_mask, pixels_L_, stop_line_y, &wins_L);
        runSwV2(blob_R, sw_mask, pixels_R_, stop_line_y, &wins_R);

        if (stop_line_y >= 0) {
            cv::line(vis_sw,
                     cv::Point(0, stop_line_y),
                     cv::Point(vis_sw.cols - 1, stop_line_y),
                     kGray, 2);
        }

        for (const auto& r : wins_L) cv::rectangle(vis_sw, r, kGreen, 1);
        for (const auto& r : wins_R) cv::rectangle(vis_sw, r, kGreen, 1);
        for (const auto& p : pixels_L_) cv::circle(vis_sw, p, 3, kGreen, cv::FILLED);
        for (const auto& p : pixels_R_) cv::circle(vis_sw, p, 3, kGreen, cv::FILLED);

        pushDebugImageExec(vis_sw, "sliding_window");

    } else {
        runSwV2(blob_L, sw_mask, pixels_L_, stop_line_y);
        runSwV2(blob_R, sw_mask, pixels_R_, stop_line_y);
    }

    return false; // lane_lost = false
}

bool LaneLineDetector::selectStartBlobs(const cv::Mat& lane_mask,
                                        StartBlob& out_L, StartBlob& out_R)
{
    return selectStartBlobsForTracking(lane_mask, out_L, out_R, true);
}

bool LaneLineDetector::selectStartBlobsForTracking(const cv::Mat& lane_mask,
                                                   StartBlob& out_L, StartBlob& out_R,
                                                   bool commit_state)
{
    const int img_h = lane_mask.rows;  // 360
    const int img_w = lane_mask.cols;  // 640

    // ── STEP 2+3: 하단 ROI에서 blob 후보 추출 (y: 270~360) ──────────────────


    cv::Rect roi_bottom(0, img_h * 3 / 4, img_w, img_h / 4); // 0, 270, 640, 90
    std::vector<StartBlob> cands = findStartBlobsInRoi(lane_mask, roi_bottom); //

    // ── STEP 3-F: Fallback (y: 180~270) ──────────────────────────────────────
    if (cands.empty()) {
        cv::Rect roi_fallback(0, img_h / 2, img_w, img_h / 4);
        cands = findStartBlobsInRoi(lane_mask, roi_fallback);
    }

    // ── LANE LOST ─────────────────────────────────────────────────────────────
    if (cands.empty()) {
        if (commit_state) {
            cur_result_.state_L       = LaneState::NODET;
            cur_result_.state_R       = LaneState::NODET;
            cur_result_.is_detected_L = false;
            cur_result_.is_detected_R = false;
            bad_streak_L_     = config_.bad_to_sw; // 다음 프레임 SW 강제 진입
            bad_streak_R_     = config_.bad_to_sw;
            kf_initialized_L_ = false;
            kf_initialized_R_ = false;
            kf_samples_initialized_L_ = false;
            kf_samples_initialized_R_ = false;
            sample_miss_L_ = 0;
            sample_miss_R_ = 0;
            has_prev_start_L_ = false;
            has_prev_start_R_ = false;
        }
        return true; // lane_lost
    }

    // ── L/R 후보 선택 (dir.x 부호 기반) ─────────────────────────────────────
    const float half = (float)img_w * 0.5f;
    int idx_L = -1, idx_R = -1;

    // 헬퍼: prev_start까지 거리 (has_prev 없으면 FLT_MAX → 항상 "먼 쪽"으로 취급)
    auto distToPrev = [](bool has_prev, const cv::Point2f& pt, const cv::Point2f& prev) -> float {
        return has_prev ? (float)cv::norm(pt - prev) : std::numeric_limits<float>::max();
    };

    // L 선택: dir.x 최대 blob (오른쪽으로 기울어진 방향 → 좌차선)
    {
        float best = std::numeric_limits<float>::lowest();
        for (int i = 0; i < (int)cands.size(); ++i) {
            if (cands[i].dir_vec.x > best) { best = cands[i].dir_vec.x; idx_L = i; }
        }
    }

    // R 선택: dir.x 최소 blob (왼쪽으로 기울어진 방향 → 우차선)
    {
        float best = std::numeric_limits<float>::max();
        for (int i = 0; i < (int)cands.size(); ++i) {
            if (cands[i].dir_vec.x < best) { best = cands[i].dir_vec.x; idx_R = i; }
        }
    }

    out_L = (idx_L >= 0) ? cands[idx_L] : StartBlob{};
    out_R = (idx_R >= 0) ? cands[idx_R] : StartBlob{};

    // ── 동측(同側) blob 충돌 처리 ─────────────────────────────────────────────
    // 두 blob이 모두 이미지 중앙 기준 같은 쪽에 있을 경우:
    //   → 중앙에 더 가까운 blob 하나만 살리고, x < half → L, x >= half → R 재배정
    if (out_L.valid && out_R.valid) {
        const bool both_left  = (out_L.end_pt.x < half) && (out_R.end_pt.x < half);
        const bool both_right = (out_L.end_pt.x >= half) && (out_R.end_pt.x >= half);

        if (both_left || both_right) {
            const float dL_c = std::abs(out_L.end_pt.x - half);
            const float dR_c = std::abs(out_R.end_pt.x - half);
            const StartBlob survivor = (dL_c <= dR_c) ? out_L : out_R;

            if (survivor.end_pt.x < half) { out_L = survivor; out_R = StartBlob{}; }
            else                          { out_R = survivor; out_L = StartBlob{}; }
        }
    }

    // 필터 3: blob_L.x < blob_R.x 보장 — 위반 시 prev_start에서 먼 쪽 무효화
    if (out_L.valid && out_R.valid && out_L.end_pt.x >= out_R.end_pt.x) {
        const float dL = distToPrev(has_prev_start_L_, out_L.end_pt, prev_start_L_);
        const float dR = distToPrev(has_prev_start_R_, out_R.end_pt, prev_start_R_);
        if (dL <= dR) out_R = StartBlob{};
        else          out_L = StartBlob{};
    }

    // prev_start / prev_dir 업데이트
    if (commit_state) {
        if (out_L.valid) { prev_start_L_ = out_L.end_pt; prev_dir_L_ = out_L.dir_vec; has_prev_start_L_ = true; }
        else             { has_prev_start_L_ = false; }
        if (out_R.valid) { prev_start_R_ = out_R.end_pt; prev_dir_R_ = out_R.dir_vec; has_prev_start_R_ = true; }
        else             { has_prev_start_R_ = false; }
    }

    if (config_.show_sw) {
        // 시작점 + 윈도우 + 선택된 점 표시?
    }

    return false; // lane_lost = false
}

std::vector<LaneLineDetector::StartBlob>
LaneLineDetector::findStartBlobsInRoi(const cv::Mat& lane_mask, const cv::Rect& roi_rect) const
{
    std::vector<StartBlob> result;

    cv::Mat roi = lane_mask(roi_rect);
    cv::Mat labels, stats, cc_ctd;
    const int n_labels = cv::connectedComponentsWithStats(roi, labels, stats, cc_ctd, 8);

    for (int lb = 1; lb < n_labels; ++lb) {
        const int bx = stats.at<int>(lb, cv::CC_STAT_LEFT);
        const int by = stats.at<int>(lb, cv::CC_STAT_TOP);
        const int bw = stats.at<int>(lb, cv::CC_STAT_WIDTH);
        const int bh = stats.at<int>(lb, cv::CC_STAT_HEIGHT);

        // 포인터 순회로 모멘트 누적 (힙 할당 없음) + blob 하단(max-y) 픽셀 추적
        long long cnt = 0, sx = 0, sy = 0;
        double m20 = 0.0, m11 = 0.0, m02 = 0.0;
        int max_y_local = INT_MIN, max_y_x = 0;
        for (int y = by, ylim = by + bh; y < ylim; ++y) {
            const int* lrow = labels.ptr<int>(y);
            for (int x = bx, xlim = bx + bw; x < xlim; ++x) {
                if (lrow[x] != lb) continue;
                ++cnt;
                sx += x; sy += y;
                m20 += (double)x * x;
                m11 += (double)x * y;
                m02 += (double)y * y;
                if (y > max_y_local) { max_y_local = y; max_y_x = x; }
            }
        }
        if (cnt == 0) continue;

        const float cx = (float)sx / (float)cnt;
        const float cy = (float)sy / (float)cnt;

        // 해석적 2×2 공분산 PCA → 주방향 벡터
        const double inv_n = 1.0 / (double)cnt;
        const double cxx  = m20 * inv_n - (double)cx * cx;
        const double cxy  = m11 * inv_n - (double)cx * cy;
        const double cyy  = m02 * inv_n - (double)cy * cy;
        const double tr   = cxx + cyy;
        const double det  = cxx * cyy - cxy * cxy;
        double disc = tr * tr * 0.25 - det;
        if (disc < 0.0) disc = 0.0;
        const double lam1 = tr * 0.5 + std::sqrt(disc);
        const double ddx  = lam1 - cyy;
        const double ddy  = cxy;
        const double len  = std::sqrt(ddx * ddx + ddy * ddy);

        cv::Point2f dir = {0.0f, -1.0f};
        if (len > 1e-6) {
            dir = {(float)(ddx / len), (float)(ddy / len)};
            if (dir.y > 0.0f) { dir.x = -dir.x; dir.y = -dir.y; } // 항상 위 방향
            const float nlen = std::sqrt(dir.x * dir.x + dir.y * dir.y);
            if (nlen > 1e-6f) { dir.x /= nlen; dir.y /= nlen; } // 정규화 보장
        }

        // end_pt = blob 하단(max-y 픽셀) → 전체 이미지 좌표, valid=true로 추가
        const float end_x = (float)max_y_x    + (float)roi_rect.x;
        const float end_y = (float)max_y_local + (float)roi_rect.y;
        result.push_back({{end_x, end_y}, dir, true});
    }
    return result;
}

void LaneLineDetector::runSwV2(const StartBlob& blob, const cv::Mat& lane_mask,
                               std::vector<cv::Point>& out,
                               int stop_line_y,
                               std::vector<cv::Rect>* debug_wins)
{
    if (!blob.valid) { out.clear(); return; }

    const int img_h = lane_mask.rows;
    const int img_w = lane_mask.cols;
    const int win_h = config_.win_h;         // 윈도우 높이 (px)
    const int win_w = config_.win_w;          // 윈도우 폭 (px)
    constexpr int kNoCountMax = 3;
    const int kMaxSteps = std::max(8, img_h / std::max(1, win_h) + 8);

    cv::Point2f center = blob.end_pt;
    cv::Point2f dir    = blob.dir_vec;
    int no_hit = 0;
    int step = 0;

    while (step++ < kMaxSteps) {
        const int x1 = std::max(0,     (int)(center.x - (float)win_w * 0.5f));
        const int x2 = std::min(img_w, (int)(center.x + (float)win_w * 0.5f));
        const int y1 = std::max(0,     (int)(center.y - (float)win_h));
        const int y2 = std::min(img_h, (int)(center.y));
        const bool reached_forward_limit = (stop_line_y >= 0 && y1 <= stop_line_y);
        if (debug_wins) debug_wins->push_back(cv::Rect(x1, y1, x2 - x1, y2 - y1));
        if (y1 >= y2 || x1 >= x2) break;

        WindowPixelStats stats = computeWindowPixelStats(lane_mask, cv::Rect(x1, y1, x2 - x1, y2 - y1), std::max(1, config_.minpix));
        if (!stats.has_enough_pixels) {
            if (++no_hit > kNoCountMax) break;
            center.x += dir.x * (float)win_h;
            center.y += dir.y * (float)win_h;
            if (reached_forward_limit) break;
            continue;
        }

        no_hit = 0;
        const float cx = stats.centroid.x;
        const float cy = stats.centroid.y;
        if (stats.pca_dir.dot(dir) > 0.0f) {
            dir = stats.pca_dir;
        }

        cv::Point sample_pt((int)cx, (int)cy);
        if (config_.use_edge_sample_in_sw) {
            const LaneSideMask side = (blob.dir_vec.x >= 0.0f) ? LaneSideMask::Left : LaneSideMask::Right;
            const HorizontalEdgeSample edge_sample = findHorizontalInnerEdgeFromCentroid(
                lane_mask, cv::Rect(x1, y1, x2 - x1, y2 - y1), stats, side);
            if (edge_sample.valid) sample_pt = edge_sample.edge_px;
        }

        out.push_back(sample_pt);
        center = {cx       + dir.x * (float)win_h,
                  center.y + dir.y * (float)win_h}; // y: cy 아닌 center.y 기준 → 윈도우 간 gap 제거
        if (reached_forward_limit) break;
    }
}

void LaneLineDetector::runSearchAround(const cv::Mat& bev_bin, const std::vector<cv::Point>& nonzero) {

    auto cL = last_coeffs_L_;
    auto cR = last_coeffs_R_;

    int margin_px = config_.margin_sa;

    double min_gap_px = config_.lane_w_min / bev_mpp_y_;
    double safe_gap_fallback = margin_px * config_.safe_gap_multiplier;
    double effective_gap = std::max(min_gap_px, safe_gap_fallback);

    // Edge cutoff: 예측 중심이 이미지 경계에 닿는 y 행 → 그 이상(먼 곳) 픽셀 제외
    const int edge_margin_px = 5;
    int y_cutoff_L = -1;  // -1: cutoff 없음
    int y_cutoff_R = -1;

    for (int v = bev_bin.rows - 1; v >= 0; v -= 5) {
        const double x_veh = bevPixToVehicle(cv::Point2d(0.0, (double)v)).x;
        if (y_cutoff_L < 0) {
            double y_m = cL[0]*x_veh*x_veh*x_veh + cL[1]*x_veh*x_veh + cL[2]*x_veh + cL[3];
            const double u = vehicleToBevPix(cv::Point2d(x_veh, y_m)).x;
            if (u <= edge_margin_px || u >= bev_bin.cols - 1 - edge_margin_px)
                y_cutoff_L = v;
        }
        if (y_cutoff_R < 0) {
            double y_m = cR[0]*x_veh*x_veh*x_veh + cR[1]*x_veh*x_veh + cR[2]*x_veh + cR[3];
            const double u = vehicleToBevPix(cv::Point2d(x_veh, y_m)).x;
            if (u >= bev_bin.cols - 1 - edge_margin_px || u <= edge_margin_px)
                y_cutoff_R = v;
        }
        if (y_cutoff_L >= 0 && y_cutoff_R >= 0) break;
    }

    if (config_.debug_pixel_overlap && config_.log_frame_debug) {
        if (y_cutoff_L >= 0) printf("[SA] Left lane edge-cutoff at row %d\n", y_cutoff_L);
        if (y_cutoff_R >= 0) printf("[SA] Right lane edge-cutoff at row %d\n", y_cutoff_R);
    }

    for (const auto& p: nonzero) {
        const double x_veh = bevPixToVehicle(cv::Point2d(0.0, (double)p.y)).x;

        // 3차식으로 y_veh(m) 예측
        double y_pred_m_L = cL[0]*x_veh*x_veh*x_veh + cL[1]*x_veh*x_veh + cL[2]*x_veh + cL[3];
        double y_pred_m_R = cR[0]*x_veh*x_veh*x_veh + cR[1]*x_veh*x_veh + cR[2]*x_veh + cR[3];

        // y_veh -> 픽셀 u 역변환
        const double u_pred_L = vehicleToBevPix(cv::Point2d(x_veh, y_pred_m_L)).x;
        const double u_pred_R = vehicleToBevPix(cv::Point2d(x_veh, y_pred_m_R)).x;

        // [수정] enforceMinGap 적용: 최소 간격 강제
        auto [uL_adj, uR_adj] = enforceMinGap(u_pred_L, u_pred_R, effective_gap);

        // [수정] classifyPixelToSide 사용: 단일 귀속 보장
        PixelSide side = classifyPixelToSide((double)p.x, uL_adj, uR_adj, margin_px);

        if (side == PixelSide::LEFT) {
            if (y_cutoff_L < 0 || p.y > y_cutoff_L)
                pixels_L_.push_back(p);
        } else if (side == PixelSide::RIGHT) {
            if (y_cutoff_R < 0 || p.y > y_cutoff_R)
                pixels_R_.push_back(p);
        }
        // PixelSide::NONE은 버림
    }

    // 디버그 로그
    if (config_.debug_pixel_overlap && config_.log_frame_debug) {
        printf("[SA] pixels_L=%zu, pixels_R=%zu, min_gap_px=%.1f\n",
               pixels_L_.size(), pixels_R_.size(), min_gap_px);
    }

    if (config_.show_sld_win) {
        cv::cvtColor(bev_bin, sld_win, cv::COLOR_GRAY2BGR);

        int mid_img = bev_bin.cols / 2 + config_.roi_mid_offset_px;

        // 검색 영역(band) 시각화: polynomial 곡선을 따라 margin 영역 표시
        int step = 10; // 10픽셀 간격으로 검색 band 그리기
        for (int v = 0; v < bev_bin.rows; v += step) {
            const double x_veh = bevPixToVehicle(cv::Point2d(0.0, (double)v)).x;

            // polynomial 예측
            double y_pred_m_L = cL[0]*x_veh*x_veh*x_veh + cL[1]*x_veh*x_veh + cL[2]*x_veh + cL[3];
            double y_pred_m_R = cR[0]*x_veh*x_veh*x_veh + cR[1]*x_veh*x_veh + cR[2]*x_veh + cR[3];

            const double u_pred_L = vehicleToBevPix(cv::Point2d(x_veh, y_pred_m_L)).x;
            const double u_pred_R = vehicleToBevPix(cv::Point2d(x_veh, y_pred_m_R)).x;

            // enforceMinGap 적용
            auto [uL_adj, uR_adj] = enforceMinGap(u_pred_L, u_pred_R, effective_gap);

            // 중앙 clamp
            uL_adj = std::min(uL_adj, (double)(mid_img - 1));
            uR_adj = std::max(uR_adj, (double)(mid_img + 1));

            // 왼쪽 검색 band (녹색 반투명)
            int lmin = std::max(0, (int)(uL_adj - margin_px));
            int lmax = std::min(mid_img - 1, (int)(uL_adj + margin_px));
            cv::line(sld_win, cv::Point(lmin, v), cv::Point(lmax, v), cv::Scalar(0, 255, 0), 1);

            // 오른쪽 검색 band (녹색 반투명)
            int rmin = std::max(mid_img + 1, (int)(uR_adj - margin_px));
            int rmax = std::min(bev_bin.cols - 1, (int)(uR_adj + margin_px));
            cv::line(sld_win, cv::Point(rmin, v), cv::Point(rmax, v), cv::Scalar(0, 255, 0), 1);

            // 예측선 중심 (노란색 점)
            if (uL_adj >= 0 && uL_adj < mid_img)
                cv::circle(sld_win, cv::Point((int)uL_adj, v), 1, cv::Scalar(0, 255, 255), -1);
            if (uR_adj > mid_img && uR_adj < bev_bin.cols)
                cv::circle(sld_win, cv::Point((int)uR_adj, v), 1, cv::Scalar(0, 255, 255), -1);
        }

        // Edge cutoff 라인 표시 (빨간 점선)
        if (y_cutoff_L >= 0) {
            for (int x = 0; x < mid_img; x += 10)
                cv::line(sld_win, cv::Point(x, y_cutoff_L), cv::Point(x + 5, y_cutoff_L), cv::Scalar(0, 0, 255), 2);
        }
        if (y_cutoff_R >= 0) {
            for (int x = mid_img; x < bev_bin.cols; x += 10)
                cv::line(sld_win, cv::Point(x, y_cutoff_R), cv::Point(x + 5, y_cutoff_R), cv::Scalar(0, 0, 255), 2);
        }

        // 검출된 픽셀 색으로 표시 (band 위에 덮어쓰기)
        for (const auto& p : pixels_L_) sld_win.at<cv::Vec3b>(p) = cv::Vec3b(0, 0, 255);
        for (const auto& p : pixels_R_) sld_win.at<cv::Vec3b>(p) = cv::Vec3b(255, 0, 0);
    }
}

void LaneLineDetector::runStripHistogramGuidedSearch(const cv::Mat& bev_bin, const std::vector<cv::Point>& nonzero)
{
    const int strip_count = std::max(1, config_.strip_count);
    const int strip_h = std::max(1, bev_bin.rows / strip_count);
    const int peak_min = std::max(1, config_.strip_peak_min_value);
    const int empty_stop = std::max(1, config_.strip_empty_stop);
    const int band_margin = std::max(1, config_.strip_band_margin);
    const int noise_min_pixels = std::max(1, config_.strip_noise_min_pixels);
    const double min_gap_px = config_.lane_w_min / bev_mpp_y_;
    const double expected_gap_px = std::max(min_gap_px, config_.lane_expected_w / bev_mpp_y_);
    const int peak_suppress_radius = std::max(3, band_margin / 2);
    const int max_peak_candidates = 4;
    const double pred_mid_alpha = 0.7;
    const double max_mid_step_px = (double)std::max(5, config_.max_shift);

    int static_mid = bev_bin.cols / 2 + config_.roi_mid_offset_px;
    static_mid = std::max(1, std::min(bev_bin.cols - 2, static_mid));
    double dynamic_mid = (double)static_mid;

    auto clampInt = [](int v, int lo, int hi) {
        return std::max(lo, std::min(hi, v));
    };

    auto clampDouble = [](double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    };

    auto predictFromLastCoeff = [this, &bev_bin](const cv::Vec4d& c, int v, bool& ok) -> double {
        if (c == cv::Vec4d(0, 0, 0, 0)) {
            ok = false;
            return 0.0;
        }

        const double x_veh = bevPixToVehicle(cv::Point2d(0.0, (double)v)).x;
        double y_pred = c[0] * x_veh * x_veh * x_veh +
                        c[1] * x_veh * x_veh +
                        c[2] * x_veh + c[3];
        const double u = vehicleToBevPix(cv::Point2d(x_veh, y_pred)).x;
        ok = (u >= 0.0 && u < bev_bin.cols);
        return u;
    };

    if (config_.show_sld_win) {
        cv::cvtColor(bev_bin, sld_win, cv::COLOR_GRAY2BGR);
    }

    std::vector<cv::Point2d> samples_L; // (v, u)
    std::vector<cv::Point2d> samples_R; // (v, u)

    bool has_prev_l = false;
    bool has_prev_r = false;
    int prev_x_l = 0;
    int prev_x_r = 0;
    int prev_strip_pixels = -1;
    int empty_run = 0;

    for (int strip_idx = 0; strip_idx < strip_count; ++strip_idx) {
        int y_max = bev_bin.rows - strip_idx * strip_h;
        int y_min = std::max(0, y_max - strip_h);
        if (strip_idx == strip_count - 1) y_min = 0; // 최상단 strip까지 커버
        if (y_min >= y_max) continue;

        cv::Mat strip = bev_bin.rowRange(y_min, y_max);
        cv::Mat hist_strip;
        cv::reduce(strip, hist_strip, 0, cv::REDUCE_SUM, CV_32S);
        const int* hptr = hist_strip.ptr<int>(0);

        int occupied_cols = 0;
        for (int x = 0; x < bev_bin.cols; ++x) {
            if (hptr[x] > 0) occupied_cols++;
        }

        const int strip_pixels = cv::countNonZero(strip);
        bool noisy_strip = false;
        if (strip_pixels >= noise_min_pixels) {
            if (prev_strip_pixels > 0 &&
                strip_pixels > (int)std::round(prev_strip_pixels * config_.strip_noise_growth_ratio)) {
                noisy_strip = true;
            }

            const double occupied_ratio = (double)occupied_cols / (double)bev_bin.cols;
            if (occupied_ratio > config_.strip_noise_max_occupancy_ratio) {
                noisy_strip = true;
            }
        }

        if (noisy_strip) {
            if (config_.show_sld_win) {
                cv::rectangle(sld_win, cv::Point(0, y_min), cv::Point(bev_bin.cols - 1, y_max - 1),
                              cv::Scalar(60, 60, 60), 1);
            }
            prev_strip_pixels = strip_pixels;
            continue;
        }

        const int y_center = (y_min + y_max) / 2;

        auto collectPeaks = [&](const int* hp) {
            std::vector<std::pair<int, int>> peaks; // (x, value)
            std::vector<bool> used(bev_bin.cols, false);
            peaks.reserve(max_peak_candidates);

            for (int k = 0; k < max_peak_candidates; ++k) {
                int best_x = -1;
                int best_val = 0;

                for (int x = 0; x < bev_bin.cols; ++x) {
                    if (used[x]) continue;
                    int val = hp[x];
                    if (val > best_val) {
                        best_val = val;
                        best_x = x;
                    }
                }

                if (best_x < 0 || best_val < peak_min) break;
                peaks.emplace_back(best_x, best_val);

                int lo = std::max(0, best_x - peak_suppress_radius);
                int hi = std::min(bev_bin.cols - 1, best_x + peak_suppress_radius);
                for (int x = lo; x <= hi; ++x) used[x] = true;
            }

            return peaks;
        };

        auto peak_candidates = collectPeaks(hptr);

        bool pred_ok_l = false;
        bool pred_ok_r = false;
        double pred_l = 0.0;
        double pred_r = 0.0;

        if (has_prev_l) {
            pred_l = (double)prev_x_l;
            pred_ok_l = true;
        } else {
            pred_l = predictFromLastCoeff(last_coeffs_L_, y_center, pred_ok_l);
        }
        if (has_prev_r) {
            pred_r = (double)prev_x_r;
            pred_ok_r = true;
        } else {
            pred_r = predictFromLastCoeff(last_coeffs_R_, y_center, pred_ok_r);
        }

        if (!pred_ok_l && pred_ok_r) {
            pred_l = pred_r - expected_gap_px;
            pred_ok_l = true;
        } else if (!pred_ok_r && pred_ok_l) {
            pred_r = pred_l + expected_gap_px;
            pred_ok_r = true;
        } else if (!pred_ok_l && !pred_ok_r) {
            pred_l = dynamic_mid - 0.5 * expected_gap_px;
            pred_r = dynamic_mid + 0.5 * expected_gap_px;
            pred_ok_l = true;
            pred_ok_r = true;
        }

        auto [pred_l_adj, pred_r_adj] = enforceMinGap(pred_l, pred_r, min_gap_px);
        pred_l = clampDouble(pred_l_adj, 0.0, (double)(bev_bin.cols - 1));
        pred_r = clampDouble(pred_r_adj, 0.0, (double)(bev_bin.cols - 1));
        const double pred_mid = 0.5 * (pred_l + pred_r);

        auto pickPeakByCost = [&](double pred_u, bool prefer_left, int banned_idx, bool use_ban,
                                  int& out_idx, int& out_x, int& out_val) -> bool {
            out_idx = -1;
            out_x = -1;
            out_val = 0;
            double best_cost = 1e18;

            for (size_t i = 0; i < peak_candidates.size(); ++i) {
                if (use_ban && (int)i == banned_idx) continue;
                const int x = peak_candidates[i].first;
                const int v = peak_candidates[i].second;

                double side_penalty = 0.0;
                if (prefer_left && x > pred_mid) {
                    side_penalty = (x - pred_mid) * 2.0;
                } else if (!prefer_left && x < pred_mid) {
                    side_penalty = (pred_mid - x) * 2.0;
                }

                double cost = std::abs((double)x - pred_u) + side_penalty;
                if (cost < best_cost) {
                    best_cost = cost;
                    out_idx = (int)i;
                    out_x = x;
                    out_val = v;
                }
            }

            return (out_idx >= 0);
        };

        bool assigned_l = false;
        bool assigned_r = false;
        int x_l = -1;
        int x_r = -1;
        int val_l = 0;
        int val_r = 0;
        int idx_l = -1;
        int idx_r = -1;

        if (!peak_candidates.empty()) {
            assigned_l = pickPeakByCost(pred_l, true, -1, false, idx_l, x_l, val_l);
            assigned_r = pickPeakByCost(pred_r, false, idx_l, assigned_l, idx_r, x_r, val_r);

            if (!assigned_r) {
                assigned_r = pickPeakByCost(pred_r, false, -1, false, idx_r, x_r, val_r);
            }
            if (!assigned_l && assigned_r) {
                assigned_l = pickPeakByCost(pred_l, true, idx_r, true, idx_l, x_l, val_l);
            }
        }

        if (assigned_l && assigned_r && std::abs(x_r - x_l) < min_gap_px) {
            if (val_l >= val_r) {
                assigned_r = false;
            } else {
                assigned_l = false;
            }
        }

        if (assigned_l && assigned_r) {
            auto [xL_adj, xR_adj] = enforceMinGap((double)x_l, (double)x_r, min_gap_px);
            x_l = clampInt((int)std::round(xL_adj), 0, bev_bin.cols - 1);
            x_r = clampInt((int)std::round(xR_adj), 0, bev_bin.cols - 1);
            if (x_l >= x_r) {
                if (val_l >= val_r) {
                    assigned_r = false;
                } else {
                    assigned_l = false;
                }
            }
        }

        if (!assigned_l && !assigned_r) {
            empty_run++;
            if (config_.show_sld_win) {
                cv::rectangle(sld_win, cv::Point(0, y_min), cv::Point(bev_bin.cols - 1, y_max - 1),
                              cv::Scalar(0, 0, 255), 1);
            }
            if (empty_run >= empty_stop) break;
            prev_strip_pixels = strip_pixels;
            continue;
        }

        double obs_mid = pred_mid;
        if (assigned_l && assigned_r) {
            obs_mid = 0.5 * ((double)x_l + (double)x_r);
        } else if (assigned_l) {
            obs_mid = 0.5 * ((double)x_l + pred_r);
        } else if (assigned_r) {
            obs_mid = 0.5 * (pred_l + (double)x_r);
        }

        const double target_mid = pred_mid_alpha * pred_mid + (1.0 - pred_mid_alpha) * obs_mid;
        double delta_mid = target_mid - dynamic_mid;
        delta_mid = clampDouble(delta_mid, -max_mid_step_px, max_mid_step_px);
        dynamic_mid = clampDouble(dynamic_mid + delta_mid, 1.0, (double)(bev_bin.cols - 2));

        empty_run = 0;
        if (assigned_l) {
            x_l = clampInt(x_l, 0, bev_bin.cols - 1);
            samples_L.emplace_back((double)y_center, (double)x_l);
            prev_x_l = x_l;
            has_prev_l = true;
        }
        if (assigned_r) {
            x_r = clampInt(x_r, 0, bev_bin.cols - 1);
            samples_R.emplace_back((double)y_center, (double)x_r);
            prev_x_r = x_r;
            has_prev_r = true;
        }

        if (config_.show_sld_win) {
            cv::rectangle(sld_win, cv::Point(0, y_min), cv::Point(bev_bin.cols - 1, y_max - 1),
                          cv::Scalar(90, 90, 90), 1);
            if (assigned_l) cv::circle(sld_win, cv::Point(x_l, y_center), 3, cv::Scalar(0, 0, 255), -1);
            if (assigned_r) cv::circle(sld_win, cv::Point(x_r, y_center), 3, cv::Scalar(255, 0, 0), -1);
        }

        prev_strip_pixels = strip_pixels;
    }

    auto fitQuadratic = [](const std::vector<cv::Point2d>& pts, cv::Vec3d& coeff) -> bool {
        if (pts.size() < 3) return false;

        cv::Mat A((int)pts.size(), 3, CV_64F);
        cv::Mat B((int)pts.size(), 1, CV_64F);
        for (size_t i = 0; i < pts.size(); ++i) {
            double v = pts[i].x;
            double u = pts[i].y;
            A.at<double>((int)i, 0) = v * v;
            A.at<double>((int)i, 1) = v;
            A.at<double>((int)i, 2) = 1.0;
            B.at<double>((int)i, 0) = u;
        }

        cv::Mat C;
        if (!cv::solve(A, B, C, cv::DECOMP_SVD)) return false;
        coeff = cv::Vec3d(C.at<double>(0, 0), C.at<double>(1, 0), C.at<double>(2, 0));
        return true;
    };

    cv::Vec3d guide_L(0, 0, 0);
    cv::Vec3d guide_R(0, 0, 0);
    const int min_guide_points = std::max(3, config_.strip_min_guide_points);

    bool has_guide_l = false;
    bool has_guide_r = false;
    if ((int)samples_L.size() >= min_guide_points) has_guide_l = fitQuadratic(samples_L, guide_L);
    if ((int)samples_R.size() >= min_guide_points) has_guide_r = fitQuadratic(samples_R, guide_R);

    if (!has_guide_l && !has_guide_r) {
        return;
    }

    for (const auto& p : nonzero) {
        bool ok_l = false;
        bool ok_r = false;
        double u_l = 0.0;
        double u_r = 0.0;

        if (has_guide_l) {
            double v = (double)p.y;
            u_l = guide_L[0] * v * v + guide_L[1] * v + guide_L[2];
            ok_l = (u_l >= 0.0 && u_l < bev_bin.cols);
        } else {
            u_l = predictFromLastCoeff(last_coeffs_L_, p.y, ok_l);
        }

        if (has_guide_r) {
            double v = (double)p.y;
            u_r = guide_R[0] * v * v + guide_R[1] * v + guide_R[2];
            ok_r = (u_r >= 0.0 && u_r < bev_bin.cols);
        } else {
            u_r = predictFromLastCoeff(last_coeffs_R_, p.y, ok_r);
        }

        if (ok_l && ok_r) {
            auto [uL_adj, uR_adj] = enforceMinGap(u_l, u_r, min_gap_px);
            PixelSide side = classifyPixelToSide((double)p.x, uL_adj, uR_adj, (double)band_margin);
            if (side == PixelSide::LEFT) pixels_L_.push_back(p);
            else if (side == PixelSide::RIGHT) pixels_R_.push_back(p);
        } else if (ok_l) {
            if (std::abs((double)p.x - u_l) <= band_margin) pixels_L_.push_back(p);
        } else if (ok_r) {
            if (std::abs((double)p.x - u_r) <= band_margin) pixels_R_.push_back(p);
        }
    }

    if (config_.debug_pixel_overlap && config_.log_frame_debug) {
        printf("[StripGuide] samples(L/R)=%zu/%zu guide(L/R)=%d/%d pixels(L/R)=%zu/%zu\n",
               samples_L.size(), samples_R.size(),
               (int)has_guide_l, (int)has_guide_r,
               pixels_L_.size(), pixels_R_.size());
    }

    if (config_.show_sld_win) {
        for (int v = 0; v < bev_bin.rows; v += 5) {
            bool ok_l = has_guide_l;
            bool ok_r = has_guide_r;
            double u_l = 0.0;
            double u_r = 0.0;

            if (has_guide_l) {
                u_l = guide_L[0] * v * v + guide_L[1] * v + guide_L[2];
                ok_l = (u_l >= 0.0 && u_l < bev_bin.cols);
            } else {
                u_l = predictFromLastCoeff(last_coeffs_L_, v, ok_l);
            }
            if (has_guide_r) {
                u_r = guide_R[0] * v * v + guide_R[1] * v + guide_R[2];
                ok_r = (u_r >= 0.0 && u_r < bev_bin.cols);
            } else {
                u_r = predictFromLastCoeff(last_coeffs_R_, v, ok_r);
            }

            if (ok_l) {
                int x = clampInt((int)std::round(u_l), 0, bev_bin.cols - 1);
                cv::circle(sld_win, cv::Point(x, v), 1, cv::Scalar(0, 200, 255), -1);
            }
            if (ok_r) {
                int x = clampInt((int)std::round(u_r), 0, bev_bin.cols - 1);
                cv::circle(sld_win, cv::Point(x, v), 1, cv::Scalar(0, 200, 255), -1);
            }
        }

        for (const auto& p : pixels_L_) sld_win.at<cv::Vec3b>(p) = cv::Vec3b(0, 0, 255);
        for (const auto& p : pixels_R_) sld_win.at<cv::Vec3b>(p) = cv::Vec3b(255, 0, 0);
    }
}

void LaneLineDetector::computeSlidingWindowBase(const cv::Mat& bev_bin,
    int& base_xl, int& base_xr, double& peak_l_val, double& peak_r_val)
{
    /* histogram -> 윈도우 시작점 계산 */
    int hist_divisor = std::max(1, config_.sw_hist_bottom_divisor);
    int hist_h = std::max(1, bev_bin.rows / hist_divisor);
    int hist_start_row = bev_bin.rows - hist_h; // 하단 1/N 영역 사용
    half = bev_bin.rowRange(hist_start_row, bev_bin.rows);
    cv::reduce(half, hist, 0, cv::REDUCE_SUM, CV_32S);

    cv::Point peak_l_loc, peak_r_loc;

    int mid_img = hist.cols / 2;

    cv::minMaxLoc(hist.colRange(0, mid_img), 0, &peak_l_val, 0, &peak_l_loc);
    cv::minMaxLoc(hist.colRange(mid_img, hist.cols), 0, &peak_r_val, 0, &peak_r_loc);
    // printf("peak_l_val: %.4f | peak_r_val: %.4f\n", peak_l_val, peak_r_val);

    base_xl = peak_l_loc.x;
    base_xr = peak_r_loc.x + mid_img;
}

void LaneLineDetector::runSlidingWindow(const cv::Mat& bev_bin, const std::vector<cv::Point>& nonzero) {
    int base_xl = 0;
    int base_xr = 0;
    double peak_l_val = 0.0;
    double peak_r_val = 0.0;
    computeSlidingWindowBase(bev_bin, base_xl, base_xr, peak_l_val, peak_r_val); //TODO: 여기서 차선 없음/한쪽 이런거 검출?

    int base_xl_curr = base_xl;
    int base_xr_curr = base_xr;

    int expected_w = config_.expected_w;

    // 최소 간격(픽셀): lane_w_min(m) 기반 계산
    double min_gap_px = config_.lane_w_min / bev_mpp_y_;

    /* sliding window */
    int window_h = bev_bin.rows / config_.n_windows;

    if (config_.show_sld_win) cv::cvtColor(bev_bin, sld_win, cv::COLOR_GRAY2BGR);

    bool stop_left = false;
    bool stop_right = false;
    const int edge_margin_px = 5;

    for (int win=0; win<config_.n_windows; ++win) {
        int win_y_min = bev_bin.rows - (win + 1) * window_h;
        int win_y_max = bev_bin.rows - win * window_h;

        int search_center_l = base_xl_curr + momentum_l;
        int search_center_r = base_xr_curr + momentum_r;

        // [수정] 윈도우가 너무 가까워지면 expected_w 기반으로 재설정
        if (search_center_l > search_center_r - config_.min_window_gap) {
            if (peak_l_val > peak_r_val) {
                search_center_r = search_center_l + expected_w;
                momentum_r = momentum_l;
            } else {
                search_center_l = search_center_r - expected_w;
                momentum_l = momentum_r;
            }
        }

        int win_xl_min = search_center_l - config_.margin_sw;
        int win_xl_max = search_center_l + config_.margin_sw;
        int win_xr_min = search_center_r - config_.margin_sw;
        int win_xr_max = search_center_r + config_.margin_sw;

        // [핵심 수정] 윈도우 겹침 방지: mid 경계 기준 클램프
        if (win_xl_max >= win_xr_min) {
            int mid_boundary = (search_center_l + search_center_r) / 2;
            win_xl_max = std::min(win_xl_max, mid_boundary - 1);
            win_xr_min = std::max(win_xr_min, mid_boundary + 1);

            // min_gap_px 보장: 클램프 후에도 너무 좁아지면 강제 벌림
            if (win_xr_min - win_xl_max < (int)min_gap_px) {
                int center = (win_xl_max + win_xr_min) / 2;
                win_xl_max = center - (int)(min_gap_px / 2);
                win_xr_min = center + (int)(min_gap_px / 2);
            }
        }

        // 디버그: 클램프된 윈도우 그리기 (edge-stop된 차선은 빨간색)
        if (config_.show_sld_win) {
            cv::Scalar color_l = stop_left ? cv::Scalar(0, 0, 255) : cv::Scalar(128, 128, 128);
            cv::Scalar color_r = stop_right ? cv::Scalar(0, 0, 255) : cv::Scalar(128, 128, 128);
            cv::rectangle(sld_win, cv::Point(win_xl_min, win_y_min),
                          cv::Point(win_xl_max, win_y_max), color_l, 2);
            cv::rectangle(sld_win, cv::Point(win_xr_min, win_y_min),
                          cv::Point(win_xr_max, win_y_max), color_r, 2);
        }

        std::vector<int> good_l_inds;
        std::vector<int> good_r_inds;

        for (size_t i=0; i<nonzero.size(); i++) {
            int ny = nonzero[i].y;
            int nx = nonzero[i].x;

            if (ny >= win_y_min && ny < win_y_max) {
                // [핵심 수정] 단일 귀속: else if로 중복 배정 방지
                // mid 경계 기준으로 좌/우 배타적 배정
                int mid_boundary = (win_xl_max + win_xr_min) / 2;

                if (!stop_left && nx >= win_xl_min && nx <= win_xl_max && nx < mid_boundary) {
                    good_l_inds.push_back(i);
                    pixels_L_.push_back(nonzero[i]);
                }
                else if (!stop_right && nx >= win_xr_min && nx <= win_xr_max && nx >= mid_boundary) {
                    good_r_inds.push_back(i);
                    pixels_R_.push_back(nonzero[i]);
                }
            }
        }

        // Edge-touching 조기 종료: 차선이 이미지 경계를 벗어나면 이후 윈도우 중단
        if (!stop_left && good_l_inds.size() > 0 && win_xl_min <= edge_margin_px) {
            int edge_count = 0;
            for (int idx : good_l_inds) {
                if (nonzero[idx].x <= edge_margin_px) edge_count++;
            }
            if ((double)edge_count / good_l_inds.size() >= 0.3) {
                stop_left = true;
                if (config_.debug_pixel_overlap && config_.log_frame_debug)
                    printf("[SW] Left lane edge-stop at window %d\n", win);
            }
        }
        if (!stop_right && good_r_inds.size() > 0 &&
            win_xr_max >= bev_bin.cols - 1 - edge_margin_px) {
            int edge_count = 0;
            for (int idx : good_r_inds) {
                if (nonzero[idx].x >= bev_bin.cols - 1 - edge_margin_px) edge_count++;
            }
            if ((double)edge_count / good_r_inds.size() >= 0.3) {
                stop_right = true;
                if (config_.debug_pixel_overlap && config_.log_frame_debug)
                    printf("[SW] Right lane edge-stop at window %d\n", win);
            }
        }

        // 다음 윈도우 중심 이동
        if (good_l_inds.size() > (size_t)config_.minpix) {
            long long sum = 0;
            for (int idx : good_l_inds) sum += nonzero[idx].x;
            int new_xl = (int)(sum / good_l_inds.size());
            momentum_l = new_xl - search_center_l;
            base_xl_curr = new_xl;
        } else {
            base_xl_curr = search_center_l;
        }

        if (good_r_inds.size() > (size_t)config_.minpix) {
            long long sum = 0;
            for (int idx : good_r_inds) sum += nonzero[idx].x;
            int new_xr = (int)(sum / good_r_inds.size());
            momentum_r = new_xr - search_center_r;
            base_xr_curr = new_xr;
        } else {
            base_xr_curr = search_center_r;
        }
    }

    // 디버그 로그: 침범 감지
    if (config_.debug_pixel_overlap && config_.log_frame_debug) {
        printf("[SW] pixels_L=%zu, pixels_R=%zu\n", pixels_L_.size(), pixels_R_.size());
    }

    // 디버깅: 검출된 픽셀 색으로 표시
    if (config_.show_sld_win) {
        for (const auto& p: pixels_L_) sld_win.at<cv::Vec3b>(p) = cv::Vec3b(0,0,255);
        for (const auto& p: pixels_R_) sld_win.at<cv::Vec3b>(p) = cv::Vec3b(255,0,0);
    }
}

void LaneLineDetector::fitPolynomial() {

    /* 좌표 변환: BEV 픽셀 좌표계 -> 차량 좌표계 */ //-> calcLaneCoeffs에서 진행

    // calcLaneCoeffs() 내부에서 피팅 품질 낮으면 0,0,0,0으로 리턴
    // fit 결과를 coeffs_fit_*에 저장 (raw 피팅 결과)
    cur_result_.coeffs_fit_L = calcLaneCoeffs(pixels_L_);
    cur_result_.coeffs_fit_R = calcLaneCoeffs(pixels_R_);

    // fit 검출 여부 플래그: 피팅 성공 여부
    cur_result_.detected_fit_L = (cur_result_.coeffs_fit_L != cv::Vec4d(0,0,0,0));
    cur_result_.detected_fit_R = (cur_result_.coeffs_fit_R != cv::Vec4d(0,0,0,0));

    // coeffs_L/R은 validateAndSmooth()에서 KF 처리 후 최종값으로 설정됨
    // 여기서는 fit 결과를 임시로 복사 (validate에서 덮어씀)
    cur_result_.coeffs_L = cur_result_.coeffs_fit_L;
    cur_result_.coeffs_R = cur_result_.coeffs_fit_R;
    cur_result_.is_detected_L = cur_result_.detected_fit_L;
    cur_result_.is_detected_R = cur_result_.detected_fit_R;

    // 픽셀 개수 저장
    cur_result_.pixel_count_L = pixels_L_.size();
    cur_result_.pixel_count_R = pixels_R_.size();

    if (config_.log_frame_debug) {
        std::cout << "[DEBUG] Pixels L: " << pixels_L_.size()
                  << " | R: " << pixels_R_.size()
                  << " (Threshold: " << config_.minpix << ")" << std::endl;
        std::cout << "[DEBUG] coeffs_fit_L:" << cur_result_.coeffs_fit_L
                  << " | R: " << cur_result_.coeffs_fit_R << std::endl;
    }
              

    if (config_.show_sld_win) {
        drawPolyBev(sld_win, cv::Scalar(0, 255, 255),
                    cur_result_.coeffs_fit_L, cur_result_.coeffs_fit_R,
                    cur_result_.detected_fit_L, cur_result_.detected_fit_R);
    }
}

void LaneLineDetector::normalizeMaskLaneSlotsByTopEndpoint(std::vector<cv::Point>& pts_L,
                                                           std::vector<cv::Point>& pts_R)
{
    if (!config_.lr_slot_by_top_endpoint) {
        return;
    }

    auto pickBottomPoint = [](const std::vector<cv::Point>& pts, cv::Point& out) -> bool {
        if (pts.empty()) return false;
        out = pts.front();
        for (const auto& p : pts) {
            if (p.y > out.y) {
                out = p;
            }
        }
        return true;
    };

    cv::Point bottom_L, bottom_R;
    bool has_L = pickBottomPoint(pts_L, bottom_L);
    bool has_R = pickBottomPoint(pts_R, bottom_R);
    if (!has_L && !has_R) {
        return;
    }

    const double mid_u = 0.5 * static_cast<double>(config_.bev_size.width);
    auto sideByTop = [mid_u](const cv::Point& p) -> int {
        return (static_cast<double>(p.x) < mid_u) ? -1 : 1; // -1:left, +1:right
    };

    int side_L = has_L ? sideByTop(bottom_L) : 0;
    int side_R = has_R ? sideByTop(bottom_R) : 0;

    if (has_L && has_R && side_L == side_R) {
        if (!config_.lr_keep_closer_to_mid) {
            return;
        }

        const double d_mid_L = std::abs(static_cast<double>(bottom_L.x) - mid_u);
        const double d_mid_R = std::abs(static_cast<double>(bottom_R.x) - mid_u);
        const bool keep_L = (d_mid_L <= d_mid_R);
        if (keep_L) {
            pts_R.clear();
            has_R = false;
            side_R = 0;
        } else {
            pts_L.clear();
            has_L = false;
            side_L = 0;
        }
    }

    std::vector<cv::Point> out_L;
    std::vector<cv::Point> out_R;
    if (has_L) {
        if (side_L < 0) out_L.swap(pts_L);
        else out_R.swap(pts_L);
    } else {
        pts_L.clear();
    }

    if (has_R) {
        if (side_R < 0) {
            if (out_L.empty()) out_L.swap(pts_R);
            else pts_R.clear();
        } else {
            if (out_R.empty()) out_R.swap(pts_R);
            else pts_R.clear();
        }
    } else {
        pts_R.clear();
    }

    pts_L.swap(out_L);
    pts_R.swap(out_R);

    if (config_.log_lr_separation) {
        std::cout << "[lr_sep][f=" << frame_idx_ << "] bottom_slot: "
                  << "L=" << (pts_L.empty() ? 0 : 1)
                  << ",R=" << (pts_R.empty() ? 0 : 1) << std::endl;
    }
}

void LaneLineDetector::applyMaskLiteSeparationGate(bool& hard_fail_L, bool& hard_fail_R,
                                                   std::string& kf_action_L, std::string& kf_action_R)
{
    auto pickBottomPoint = [](const std::vector<cv::Point>& pts, cv::Point& out) -> bool {
        if (pts.empty()) return false;
        out = pts.front();
        for (const auto& p : pts) {
            if (p.y > out.y) out = p;
        }
        return true;
    };

    auto dropLeft = [&]() {
        cur_result_.is_detected_L = false;
        cur_result_.coeffs_L = cv::Vec4d(0, 0, 0, 0);
        cur_result_.state_L = LaneState::NODET;
        cur_result_.quality_L = LaneQuality::BAD;
        hard_fail_L = false;
        kf_initialized_L_ = false;
        mask_lite_miss_L_ = std::max(1, config_.max_bad_frames);
        kf_action_L = "reset_sep";
    };

    auto dropRight = [&]() {
        cur_result_.is_detected_R = false;
        cur_result_.coeffs_R = cv::Vec4d(0, 0, 0, 0);
        cur_result_.state_R = LaneState::NODET;
        cur_result_.quality_R = LaneQuality::BAD;
        hard_fail_R = false;
        kf_initialized_R_ = false;
        mask_lite_miss_R_ = std::max(1, config_.max_bad_frames);
        kf_action_R = "reset_sep";
    };

    if (config_.lr_slot_by_top_endpoint && cur_result_.is_detected_L && cur_result_.is_detected_R) {
        cv::Point bottom_L, bottom_R;
        const bool has_bottom_L = pickBottomPoint(pixels_L_, bottom_L);
        const bool has_bottom_R = pickBottomPoint(pixels_R_, bottom_R);
        if (has_bottom_L && has_bottom_R) {
            const double mid_u = 0.5 * static_cast<double>(config_.bev_size.width);
            const int side_L = (static_cast<double>(bottom_L.x) < mid_u) ? -1 : 1;
            const int side_R = (static_cast<double>(bottom_R.x) < mid_u) ? -1 : 1;
            if (side_L == side_R && config_.lr_keep_closer_to_mid) {
                const double d_mid_L = std::abs(static_cast<double>(bottom_L.x) - mid_u);
                const double d_mid_R = std::abs(static_cast<double>(bottom_R.x) - mid_u);
                if (d_mid_L <= d_mid_R) dropRight();
                else dropLeft();
                if (config_.log_lr_separation) {
                    std::cout << "[lr_sep][f=" << frame_idx_
                              << "] same_side_drop: keep=" << (d_mid_L <= d_mid_R ? "L" : "R")
                              << std::endl;
                }
            }
        }
    }

    if (cur_result_.is_detected_L && cur_result_.is_detected_R) {
        auto polyEval = [](const cv::Vec4d& c, double x) -> double {
            return c[0]*x*x*x + c[1]*x*x + c[2]*x + c[3];
        };
        const double y_L = polyEval(cur_result_.coeffs_L, config_.x_check);
        const double y_R = polyEval(cur_result_.coeffs_R, config_.x_check);
        const double sep_m = std::abs(y_L - y_R);
        if (sep_m < config_.lane_sep_min_m) {
            if (cur_result_.rmse_fit_m_L <= cur_result_.rmse_fit_m_R) dropRight();
            else dropLeft();
            if (config_.log_lr_separation) {
                std::cout << "[lr_sep][f=" << frame_idx_ << "] sep_drop: sep="
                          << sep_m << " < " << config_.lane_sep_min_m << std::endl;
            }
        }
    }
}

bool LaneLineDetector::buildLaneSampleMeasurementFromPoints(const std::vector<cv::Point2f>& pts_vehicle,
                                                            cv::Mat& z_out,
                                                            std::vector<uint8_t>& valid_mask) const
{
    const int n = sampleCount();
    if (n < 4) {
        return false;
    }

    z_out = cv::Mat::zeros(n, 1, CV_32F);
    valid_mask.assign(n, 0U);

    std::vector<std::vector<float>> slot_ys(n);
    for (const auto& pt : pts_vehicle) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) {
            continue;
        }
        for (int i = 0; i < n; ++i) {
            const double dx = std::abs(static_cast<double>(pt.x) - config_.kf_sample_x_m[i]);
            if (dx <= config_.kf_sample_band_half_width_m) {
                slot_ys[i].push_back(pt.y);
            }
        }
    }

    int valid_count = 0;
    for (int i = 0; i < n; ++i) {
        auto& ys = slot_ys[i];
        if ((int)ys.size() < std::max(1, config_.kf_sample_min_points_per_slot)) {
            continue;
        }
        std::sort(ys.begin(), ys.end());
        const size_t mid = ys.size() / 2;
        const float y = (ys.size() % 2 == 0) ? 0.5f * (ys[mid - 1] + ys[mid]) : ys[mid];
        if (!std::isfinite(y)) {
            continue;
        }
        z_out.at<float>(i) = y;
        valid_mask[i] = 1U;
        ++valid_count;
    }

    if (valid_count <= 0) {
        z_out.release();
        valid_mask.clear();
        return false;
    }

    return true;
}

bool LaneLineDetector::buildLaneSampleMeasurementFromAccumulator(const std::vector<std::vector<float>>& slot_y_accum,
                                                                 cv::Mat& z_out,
                                                                 std::vector<uint8_t>& valid_mask) const
{
    const int n = sampleCount();
    if (n < 4 || (int)slot_y_accum.size() != n) {
        return false;
    }

    z_out = cv::Mat::zeros(n, 1, CV_32F);
    valid_mask.assign(n, 0U);
    int valid_count = 0;
    for (int i = 0; i < n; ++i) {
        const auto& ys = slot_y_accum[(size_t)i];
        if ((int)ys.size() < std::max(1, config_.kf_sample_min_points_per_slot)) {
            continue;
        }
        std::vector<float> sorted = ys;
        std::sort(sorted.begin(), sorted.end());
        const size_t mid = sorted.size() / 2;
        const float y = (sorted.size() % 2 == 0) ? 0.5f * (sorted[mid - 1] + sorted[mid]) : sorted[mid];
        if (!std::isfinite(y)) {
            continue;
        }
        z_out.at<float>(i) = y;
        valid_mask[i] = 1U;
        ++valid_count;
    }

    if (valid_count <= 0) {
        z_out.release();
        valid_mask.clear();
        return false;
    }

    return true;
}

bool LaneLineDetector::fillMissingLaneSampleSlots(cv::Mat& z_io,
                                                  const std::vector<uint8_t>& valid_mask) const
{
    const int n = sampleCount();
    if (z_io.empty() || z_io.rows < n || (int)valid_mask.size() != n) {
        return false;
    }

    int first_valid = -1;
    int last_valid = -1;
    for (int i = 0; i < n; ++i) {
        if (valid_mask[i]) {
            if (first_valid < 0) {
                first_valid = i;
            }
            last_valid = i;
        }
    }
    if (first_valid < 0) {
        return false;
    }

    const float first_y = z_io.at<float>(first_valid);
    for (int i = 0; i < first_valid; ++i) {
        z_io.at<float>(i) = first_y;
    }

    const float last_y = z_io.at<float>(last_valid);
    for (int i = last_valid + 1; i < n; ++i) {
        z_io.at<float>(i) = last_y;
    }

    int left_valid = first_valid;
    for (int i = first_valid + 1; i < n; ++i) {
        if (valid_mask[i]) {
            const int right_valid = i;
            const float y_left = z_io.at<float>(left_valid);
            const float y_right = z_io.at<float>(right_valid);
            const double x_left = config_.kf_sample_x_m[left_valid];
            const double x_right = config_.kf_sample_x_m[right_valid];
            const double denom = x_right - x_left;
            for (int j = left_valid + 1; j < right_valid; ++j) {
                const double t = (denom == 0.0) ? 0.0 : (config_.kf_sample_x_m[j] - x_left) / denom;
                z_io.at<float>(j) = static_cast<float>(y_left + (y_right - y_left) * t);
            }
            left_valid = right_valid;
        }
    }

    return true;
}

void LaneLineDetector::resetSampleSlotAccumulator(LaneSideMask side) const
{
    std::vector<std::vector<float>>& accum = (side == LaneSideMask::Left) ? sample_slot_y_accum_L_ : sample_slot_y_accum_R_;
    accum.assign((size_t)sampleCount(), std::vector<float>());
}

void LaneLineDetector::resetAllSampleSlotAccumulators() const
{
    resetSampleSlotAccumulator(LaneSideMask::Left);
    resetSampleSlotAccumulator(LaneSideMask::Right);
}

void LaneLineDetector::accumulateSampleSlotFromWindow(LaneSideMask side,
                                                      const cv::Rect& win_rect,
                                                      const cv::Point& sample_pt) const
{
    const int n = sampleCount();
    if (n <= 0 || H_mask2bev_.empty() || win_rect.width <= 0 || win_rect.height <= 0) {
        return;
    }

    const float sample_x = std::clamp((float)sample_pt.x,
                                      (float)win_rect.x,
                                      (float)(win_rect.x + win_rect.width - 1));
    const float top_y = (float)win_rect.y;
    const float bottom_y = (float)(win_rect.y + win_rect.height - 1);
    std::vector<cv::Point2f> mask_pts = {
        cv::Point2f(sample_x, top_y),
        cv::Point2f(sample_x, bottom_y),
        cv::Point2f((float)sample_pt.x, (float)sample_pt.y)
    };
    std::vector<cv::Point2f> bev_pts;
    cv::perspectiveTransform(mask_pts, bev_pts, H_mask2bev_);
    if (bev_pts.size() != 3) {
        return;
    }

    const cv::Point2d veh_top = bevPixToVehicle(cv::Point2d((double)bev_pts[0].x, (double)bev_pts[0].y));
    const cv::Point2d veh_bottom = bevPixToVehicle(cv::Point2d((double)bev_pts[1].x, (double)bev_pts[1].y));
    const cv::Point2d veh_sample = bevPixToVehicle(cv::Point2d((double)bev_pts[2].x, (double)bev_pts[2].y));
    if (!std::isfinite(veh_top.x) || !std::isfinite(veh_bottom.x) || !std::isfinite(veh_sample.y)) {
        return;
    }

    const double x_min = std::min(veh_top.x, veh_bottom.x);
    const double x_max = std::max(veh_top.x, veh_bottom.x);
    std::vector<std::vector<float>>& accum = (side == LaneSideMask::Left) ? sample_slot_y_accum_L_ : sample_slot_y_accum_R_;
    if ((int)accum.size() != n) {
        accum.assign((size_t)n, std::vector<float>());
    }
    for (int i = 0; i < n; ++i) {
        const double x_target = config_.kf_sample_x_m[i];
        if (x_target >= x_min && x_target <= x_max) {
            accum[(size_t)i].push_back((float)veh_sample.y);
        }
    }
}

cv::Vec4d LaneLineDetector::fitPolynomialFromSamples(const cv::Mat& state_or_meas) const
{
    const int n = sampleCount();
    if (n < 4 || state_or_meas.empty() || state_or_meas.rows < n) {
        return cv::Vec4d(0, 0, 0, 0);
    }

    cv::Mat X(n, 4, CV_64F);
    cv::Mat Y(n, 1, CV_64F);
    for (int i = 0; i < n; ++i) {
        const double x = config_.kf_sample_x_m[i];
        const double y = static_cast<double>(state_or_meas.at<float>(i));
        X.at<double>(i, 0) = x * x * x;
        X.at<double>(i, 1) = x * x;
        X.at<double>(i, 2) = x;
        X.at<double>(i, 3) = 1.0;
        Y.at<double>(i, 0) = y;
    }

    cv::Mat coeffs;
    if (!cv::solve(X, Y, coeffs, cv::DECOMP_SVD)) {
        return cv::Vec4d(0, 0, 0, 0);
    }

    return cv::Vec4d(coeffs.at<double>(0), coeffs.at<double>(1),
                     coeffs.at<double>(2), coeffs.at<double>(3));
}

LaneGeom LaneLineDetector::fitLaneGeometry(const std::vector<cv::Point2d>& pts) const
{
    LaneGeom out;
    if (pts.size() < 4) {
        return out;
    }

    cv::Mat X(static_cast<int>(pts.size()), 4, CV_64F);
    cv::Mat Y(static_cast<int>(pts.size()), 1, CV_64F);
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        const double x = pts[(size_t)i].x;
        const double y = pts[(size_t)i].y;
        X.at<double>(i, 0) = 1.0;
        X.at<double>(i, 1) = x;
        X.at<double>(i, 2) = 0.5 * x * x;
        X.at<double>(i, 3) = (1.0 / 6.0) * x * x * x;
        Y.at<double>(i, 0) = y;
    }

    cv::Mat geom;
    if (!cv::solve(X, Y, geom, cv::DECOMP_SVD)) {
        return out;
    }

    out.y0 = geom.at<double>(0, 0);
    out.theta = geom.at<double>(1, 0);
    out.kappa = geom.at<double>(2, 0);
    out.kappa_dot = geom.at<double>(3, 0);
    out.valid = std::isfinite(out.y0) && std::isfinite(out.theta) &&
                std::isfinite(out.kappa) && std::isfinite(out.kappa_dot);
    return out;
}

cv::Vec4d LaneLineDetector::geometryToPolynomial(const LaneGeom& g) const
{
    if (!g.valid) {
        return cv::Vec4d(0, 0, 0, 0);
    }
    return cv::Vec4d(g.kappa_dot / 6.0, g.kappa / 2.0, g.theta, g.y0);
}

void LaneLineDetector::setSampleMeasurementNoiseWithValidity(cv::KalmanFilter& kf,
                                                             const std::vector<uint8_t>& valid_mask) const
{
    const int n = sampleCount();
    if (n <= 0 || (int)valid_mask.size() != n) {
        return;
    }

    kf.measurementNoiseCov = cv::Mat::zeros(n, n, CV_32F);
    for (int i = 0; i < n; ++i) {
        if (!valid_mask[i]) {
            kf.measurementNoiseCov.at<float>(i, i) = static_cast<float>(config_.kf_sample_invalid_R);
            continue;
        }
        kf.measurementNoiseCov.at<float>(i, i) =
            static_cast<float>(config_.kf_sample_R_diag_list[(size_t)i]);
    }
}

void LaneLineDetector::trackLaneGeometryKF()
{
    const std::vector<cv::Point2f> pixels_vehicle_L_f = bevToVehicleLocal(pixels_L_);
    const std::vector<cv::Point2f> pixels_vehicle_R_f = bevToVehicleLocal(pixels_R_);
    std::vector<cv::Point2d> pixels_vehicle_L;
    std::vector<cv::Point2d> pixels_vehicle_R;
    pixels_vehicle_L.reserve(pixels_vehicle_L_f.size());
    pixels_vehicle_R.reserve(pixels_vehicle_R_f.size());
    for (const auto& p : pixels_vehicle_L_f) pixels_vehicle_L.emplace_back(p.x, p.y);
    for (const auto& p : pixels_vehicle_R_f) pixels_vehicle_R.emplace_back(p.x, p.y);

    const LaneGeom meas_geom_L = fitLaneGeometry(pixels_vehicle_L);
    const LaneGeom meas_geom_R = fitLaneGeometry(pixels_vehicle_R);
    const bool det_geom_L = meas_geom_L.valid;
    const bool det_geom_R = meas_geom_R.valid;
    const cv::Vec4d raw_geom_coeffs_L = geometryToPolynomial(meas_geom_L);
    const cv::Vec4d raw_geom_coeffs_R = geometryToPolynomial(meas_geom_R);

    auto geomToMat = [](const LaneGeom& g) {
        cv::Mat z = cv::Mat::zeros(4, 1, CV_32F);
        z.at<float>(0) = static_cast<float>(g.y0);
        z.at<float>(1) = static_cast<float>(g.theta);
        z.at<float>(2) = static_cast<float>(g.kappa);
        z.at<float>(3) = static_cast<float>(g.kappa_dot);
        return z;
    };
    auto matToGeom = [](const cv::Mat& x) {
        LaneGeom g;
        if (x.empty() || x.rows < 4) return g;
        g.y0 = static_cast<double>(x.at<float>(0));
        g.theta = static_cast<double>(x.at<float>(1));
        g.kappa = static_cast<double>(x.at<float>(2));
        g.kappa_dot = static_cast<double>(x.at<float>(3));
        g.valid = std::isfinite(g.y0) && std::isfinite(g.theta) &&
                  std::isfinite(g.kappa) && std::isfinite(g.kappa_dot);
        return g;
    };

    const cv::Mat pred_geom_L = kf_geom_initialized_L_ ? kf_geom_L_.predict() : cv::Mat::zeros(4, 1, CV_32F);
    const cv::Mat pred_geom_R = kf_geom_initialized_R_ ? kf_geom_R_.predict() : cv::Mat::zeros(4, 1, CV_32F);
    const int hold_frames = std::max(1, config_.geom_kf_hold_frames);

    auto processSide = [&](bool det_geom_ok, const LaneGeom& meas_geom,
                           cv::KalmanFilter& kf, bool& kf_initialized,
                           const cv::Mat& pred_geom, int& miss_count,
                           cv::Vec4d& out_coeff, bool& out_detected,
                           LaneState& out_state, LaneQuality& out_quality,
                           uint8_t& out_availability,
                           double& out_view_range_m,
                           double& out_lane_start_m, double& out_lane_end_m, double& out_span_m,
                           const std::vector<cv::Point>& pixels,
                           bool& hold_valid,
                           double& hold_lane_start_m, double& hold_lane_end_m,
                           double& hold_span_m, double& hold_view_range_m) {
        LaneGeom geom_out;
        if (det_geom_ok) {
            miss_count = 0;
            const cv::Mat z = geomToMat(meas_geom);
            if (!kf_initialized) {
                kf.statePost = z.clone();
                kf.statePre = z.clone();
                kf_initialized = true;
                geom_out = meas_geom;
            } else {
                geom_out = matToGeom(kf.correct(z));
            }
        } else if (kf_initialized && miss_count < hold_frames) {
            ++miss_count;
            geom_out = matToGeom(pred_geom);
        }

        out_coeff = geometryToPolynomial(geom_out);
        out_detected = (out_coeff != cv::Vec4d(0, 0, 0, 0));
        if (out_detected) {
            const bool is_hold = !det_geom_ok;
            out_state = is_hold ? LaneState::WEAK : LaneState::GOOD;
            out_quality = is_hold ? LaneQuality::WEAK : LaneQuality::GOOD;
            out_availability = 1U;
            if (!is_hold && calcLaneRangeM(pixels, out_lane_start_m, out_lane_end_m, out_span_m)) {
                out_view_range_m = std::max(0.0, out_span_m);
                hold_lane_start_m = out_lane_start_m;
                hold_lane_end_m = out_lane_end_m;
                hold_span_m = out_span_m;
                hold_view_range_m = out_view_range_m;
                hold_valid = true;
            } else if (is_hold && hold_valid) {
                out_lane_start_m = hold_lane_start_m;
                out_lane_end_m = hold_lane_end_m;
                out_span_m = hold_span_m;
                out_view_range_m = hold_view_range_m;
            } else {
                out_lane_start_m = 0.0;
                out_lane_end_m = 0.0;
                out_span_m = 0.0;
                out_view_range_m = 0.0;
            }
            return;
        }

        miss_count = hold_frames;
        kf_initialized = false;
        hold_valid = false;
        out_coeff = cv::Vec4d(0, 0, 0, 0);
        out_detected = false;
        out_state = LaneState::NODET;
        out_quality = LaneQuality::BAD;
        out_availability = 0U;
        out_lane_start_m = 0.0;
        out_lane_end_m = 0.0;
        out_span_m = 0.0;
        out_view_range_m = 0.0;
    };

    processSide(det_geom_L, meas_geom_L,
                kf_geom_L_, kf_geom_initialized_L_, pred_geom_L, geom_miss_L_,
                cur_result_.coeffs_L, cur_result_.is_detected_L,
                cur_result_.state_L, cur_result_.quality_L,
                cur_result_.availability_L, cur_result_.view_range_m_L,
                cur_result_.lane_start_m_L, cur_result_.lane_end_m_L, cur_result_.span_m_L,
                pixels_L_,
                geom_hold_valid_L_,
                geom_hold_lane_start_m_L_, geom_hold_lane_end_m_L_,
                geom_hold_span_m_L_, geom_hold_view_range_m_L_);

    processSide(det_geom_R, meas_geom_R,
                kf_geom_R_, kf_geom_initialized_R_, pred_geom_R, geom_miss_R_,
                cur_result_.coeffs_R, cur_result_.is_detected_R,
                cur_result_.state_R, cur_result_.quality_R,
                cur_result_.availability_R, cur_result_.view_range_m_R,
                cur_result_.lane_start_m_R, cur_result_.lane_end_m_R, cur_result_.span_m_R,
                pixels_R_,
                geom_hold_valid_R_,
                geom_hold_lane_start_m_R_, geom_hold_lane_end_m_R_,
                geom_hold_span_m_R_, geom_hold_view_range_m_R_);

    cur_result_.rmse_fit_m_L = 0.0;
    cur_result_.rmse_fit_m_R = 0.0;
    cur_result_.hard_fail_L = false;
    cur_result_.hard_fail_R = false;
    cur_result_.delta_y_m_L = 0.0;
    cur_result_.delta_y_m_R = 0.0;
    cur_result_.delta_kappa_L = 0.0;
    cur_result_.delta_kappa_R = 0.0;

    if (cur_result_.state_L == LaneState::GOOD || cur_result_.state_L == LaneState::WEAK) {
        last_coeffs_L_ = cur_result_.coeffs_L;
    }
    if (cur_result_.state_R == LaneState::GOOD || cur_result_.state_R == LaneState::WEAK) {
        last_coeffs_R_ = cur_result_.coeffs_R;
    }

    auto updateStreak = [](LaneState s, int& bad_streak, int& weak_streak) {
        switch (s) {
            case LaneState::NODET:
            case LaneState::BAD:
                bad_streak++;
                weak_streak = 0;
                break;
            case LaneState::WEAK:
                weak_streak++;
                bad_streak = 0;
                break;
            case LaneState::GOOD:
                weak_streak = 0;
                bad_streak = 0;
                break;
        }
    };
    updateStreak(cur_result_.state_L, bad_streak_L_, weak_streak_L_);
    updateStreak(cur_result_.state_R, bad_streak_R_, weak_streak_R_);

    prev_state_L_ = cur_result_.state_L;
    prev_state_R_ = cur_result_.state_R;
    prev_real_L_ = (cur_result_.state_L != LaneState::NODET);
    prev_real_R_ = (cur_result_.state_R != LaneState::NODET);

    drawLaneGeometryKfDebug(raw_geom_coeffs_L, det_geom_L,
                            raw_geom_coeffs_R, det_geom_R,
                            cur_result_.coeffs_L, cur_result_.is_detected_L,
                            cur_result_.coeffs_R, cur_result_.is_detected_R);
}

void LaneLineDetector::drawLaneGeometryKfDebug(const cv::Vec4d& raw_L, bool raw_valid_L,
                                               const cv::Vec4d& raw_R, bool raw_valid_R,
                                               const cv::Vec4d& kf_L, bool kf_valid_L,
                                               const cv::Vec4d& kf_R, bool kf_valid_R)
{
    if (!config_.show_fitpoly) {
        return;
    }

    cv::Mat vis_bev = cv::Mat::zeros(config_.bev_size, CV_8UC3);
    const cv::Scalar kRaw(0, 255, 255);
    const cv::Scalar kKf(0, 255, 0);

    auto drawCurve = [&](const cv::Vec4d& coeffs, bool detected, const cv::Scalar& color) {
        if (!detected) return;
        std::vector<cv::Point> curve_pts;
        constexpr double kDxM = 0.2;
        const double x_min = config_.roi_xmin + kRearOriginShiftM;
        const double x_max = config_.roi_xmax + kRearOriginShiftM;
        for (double x_veh = x_min; x_veh <= x_max; x_veh += kDxM) {
            const double y_veh = coeffs[0] * x_veh * x_veh * x_veh +
                                 coeffs[1] * x_veh * x_veh +
                                 coeffs[2] * x_veh +
                                 coeffs[3];
            const cv::Point2d p_bev = vehicleToBevPix(cv::Point2d(x_veh, y_veh));
            if (!std::isfinite(p_bev.x) || !std::isfinite(p_bev.y)) continue;
            const int u = cvRound(p_bev.x);
            const int v = cvRound(p_bev.y);
            if (u < 0 || u >= vis_bev.cols || v < 0 || v >= vis_bev.rows) continue;
            curve_pts.emplace_back(u, v);
        }
        if (curve_pts.size() >= 2) {
            cv::polylines(vis_bev, curve_pts, false, color, 2);
        }
    };

    auto drawLegendItem = [&](const cv::Point& origin, const cv::Scalar& color, const std::string& label) {
        constexpr int kLineLenPx = 24;
        constexpr int kTextOffsetXPx = 8;
        cv::line(vis_bev, origin, cv::Point(origin.x + kLineLenPx, origin.y), color, 2, cv::LINE_AA);
        cv::putText(vis_bev, label,
                    cv::Point(origin.x + kLineLenPx + kTextOffsetXPx, origin.y + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    };

    drawCurve(raw_L, raw_valid_L, kRaw);
    drawCurve(raw_R, raw_valid_R, kRaw);
    drawCurve(kf_L, kf_valid_L, kKf);
    drawCurve(kf_R, kf_valid_R, kKf);

    int legend_x = 12;
    drawLegendItem(cv::Point(legend_x, 18), kRaw, "Geometry fit");
    drawLegendItem(cv::Point(legend_x + 160, 18), kKf, "KF corrected");

    pushDebugImageExec(vis_bev, "KF_tracking");
}

void LaneLineDetector::drawLaneSampleKfDebug(const cv::Mat& filtered_L, bool filtered_valid_L,
                                             const cv::Mat& filtered_R, bool filtered_valid_R)
{
    if (!config_.show_fitpoly) {
        return;
    }

    cv::Mat vis_bev = cv::Mat::zeros(config_.bev_size, CV_8UC3);
    const cv::Scalar kRaw(0, 255, 0);
    const cv::Scalar kSamples(0, 255, 255);
    const cv::Scalar kSamplesOutline(0, 0, 0);
    const cv::Scalar kFiltered(0, 255, 255);

    auto drawCurve = [&](const cv::Vec4d& coeffs, bool detected, const cv::Scalar& color) {
        if (!detected) return;
        std::vector<cv::Point> curve_pts;
        constexpr double kDxM = 0.2;
        const double x_min = config_.roi_xmin + kRearOriginShiftM;
        const double x_max = config_.roi_xmax + kRearOriginShiftM;
        for (double x_veh = x_min; x_veh <= x_max; x_veh += kDxM) {
            const double y_veh = coeffs[0] * x_veh * x_veh * x_veh +
                                 coeffs[1] * x_veh * x_veh +
                                 coeffs[2] * x_veh +
                                 coeffs[3];
            const cv::Point2d p_bev = vehicleToBevPix(cv::Point2d(x_veh, y_veh));
            if (!std::isfinite(p_bev.x) || !std::isfinite(p_bev.y)) continue;
            const int u = cvRound(p_bev.x);
            const int v = cvRound(p_bev.y);
            if (u < 0 || u >= vis_bev.cols || v < 0 || v >= vis_bev.rows) continue;
            curve_pts.emplace_back(u, v);
        }
        if (curve_pts.size() >= 2) {
            cv::polylines(vis_bev, curve_pts, false, color, 2);
        }
    };

    auto drawSamples = [&](const cv::Mat& state, bool valid) {
        if (!valid || state.empty() || state.rows < sampleCount()) return;
        constexpr int kSampleRadiusPx = 9;
        for (int i = 0; i < sampleCount(); ++i) {
            const double x = config_.kf_sample_x_m[i];
            const double y = static_cast<double>(state.at<float>(i));
            const cv::Point2d p_bev = vehicleToBevPix(cv::Point2d(x, y));
            if (!std::isfinite(p_bev.x) || !std::isfinite(p_bev.y)) continue;
            const int u = cvRound(p_bev.x);
            const int v = cvRound(p_bev.y);
            if (u < 0 || u >= vis_bev.cols || v < 0 || v >= vis_bev.rows) continue;
            cv::circle(vis_bev, cv::Point(u, v), kSampleRadiusPx, kSamples, cv::FILLED, cv::LINE_AA);
            cv::circle(vis_bev, cv::Point(u, v), kSampleRadiusPx, kSamplesOutline, 1, cv::LINE_AA);
        }
    };

    drawCurve(cur_result_.coeffs_fit_L, cur_result_.detected_fit_L, kRaw);
    drawCurve(cur_result_.coeffs_fit_R, cur_result_.detected_fit_R, kRaw);
    drawSamples(filtered_L, filtered_valid_L);
    drawSamples(filtered_R, filtered_valid_R);
    drawCurve(cur_result_.coeffs_L, cur_result_.is_detected_L, kFiltered);
    drawCurve(cur_result_.coeffs_R, cur_result_.is_detected_R, kFiltered);

    auto drawLegend = [&](const cv::Point& origin, const cv::Scalar& color, const std::string& label) {
        constexpr int kLineLenPx = 24;
        constexpr int kTextOffsetXPx = 10;
        constexpr int kTextBaselineYPx = 5;
        cv::line(vis_bev, origin, cv::Point(origin.x + kLineLenPx, origin.y), color, 2, cv::LINE_AA);
        cv::putText(vis_bev, label,
                    cv::Point(origin.x + kLineLenPx + kTextOffsetXPx, origin.y + kTextBaselineYPx),
                    cv::FONT_HERSHEY_SIMPLEX, 1.50, color, 2, cv::LINE_AA);
    };

    // const int legend_x = std::max(10, vis_bev.cols - 180);
    const int legend_y = 35;
    drawLegend(cv::Point(0, legend_y), kRaw, "Raw fit");
    drawLegend(cv::Point((int)(vis_bev.cols/2), legend_y), kFiltered, "KF corrected");

    pushDebugImageExec(vis_bev, "bev_lane_kf");
}

void LaneLineDetector::trackLaneSamplesKF()
{
    const int n = sampleCount();
    // if (n < 4) {
    //     validateAndSmoothMaskLite();
    //     return;
    // }

    cur_result_.rmse_fit_m_L = 0.0;
    cur_result_.rmse_fit_m_R = 0.0;

    std::vector<uint8_t> valid_mask_L;
    std::vector<uint8_t> valid_mask_R;
    const bool has_accum_L = !sample_slot_y_accum_L_.empty();
    const bool has_accum_R = !sample_slot_y_accum_R_.empty();
    const std::vector<cv::Point2f> pixels_vehicle_L = has_accum_L ? std::vector<cv::Point2f>() : bevToVehicleLocal(pixels_L_);
    const std::vector<cv::Point2f> pixels_vehicle_R = has_accum_R ? std::vector<cv::Point2f>() : bevToVehicleLocal(pixels_R_);

    const bool det_fit_L = cur_result_.detected_fit_L &&
                           (has_accum_L
                                ? buildLaneSampleMeasurementFromAccumulator(sample_slot_y_accum_L_, meas_samples_L_, valid_mask_L)
                                : buildLaneSampleMeasurementFromPoints(pixels_vehicle_L, meas_samples_L_, valid_mask_L));
    const bool det_fit_R = cur_result_.detected_fit_R &&
                           (has_accum_R
                                ? buildLaneSampleMeasurementFromAccumulator(sample_slot_y_accum_R_, meas_samples_R_, valid_mask_R)
                                : buildLaneSampleMeasurementFromPoints(pixels_vehicle_R, meas_samples_R_, valid_mask_R));

    pred_samples_L_ = cv::Mat::zeros(2 * n, 1, CV_32F);
    pred_samples_R_ = cv::Mat::zeros(2 * n, 1, CV_32F);
    pred_samples_L_ = kf_samples_initialized_L_ ? kf_L_samples_.predict() : cv::Mat::zeros(2 * n, 1, CV_32F);
    pred_samples_R_ = kf_samples_initialized_R_ ? kf_R_samples_.predict() : cv::Mat::zeros(2 * n, 1, CV_32F);

    const int hold_frames = std::max(1, config_.kf_sample_hold_frames);
    std::string kf_action_L = "none";
    std::string kf_action_R = "none";

    auto seedFromMeasurement = [n](cv::KalmanFilter& kf, const cv::Mat& z) {
        kf.statePost = cv::Mat::zeros(2 * n, 1, CV_32F);
        for (int i = 0; i < n; ++i) {
            kf.statePost.at<float>(i) = z.at<float>(i);
        }
        kf.statePre = kf.statePost.clone();
    };

    auto processSide = [&](bool det_fit_ok, const cv::Mat& meas,
                           const std::vector<uint8_t>& valid_mask,
                           cv::KalmanFilter& kf, bool& kf_initialized,
                           cv::Mat& pred, cv::Mat& estimated,
                           int& miss_count,
                           cv::Vec4d& out_coeff, bool& out_detected,
                           LaneState& out_state, LaneQuality& out_quality,
                           uint8_t& out_availability,
                           double& out_view_range_m,
                           double& out_lane_start_m, double& out_lane_end_m, double& out_span_m,
                           const std::vector<cv::Point>& pixels,
                           std::string& out_kf_action) {
        if (det_fit_ok) {
            miss_count = 0;
            if (!kf_initialized) {
                cv::Mat seed_meas = meas.clone();
                if (!fillMissingLaneSampleSlots(seed_meas, valid_mask)) {
                    det_fit_ok = false;
                } else {
                    seedFromMeasurement(kf, seed_meas);
                    kf_initialized = true;
                    estimated = kf.statePost.clone();
                    out_kf_action = "seed";
                }
            } else {
                cv::Mat corr_meas = pred.rowRange(0, n).clone();
                for (int i = 0; i < n; ++i) {
                    if (valid_mask[i]) {
                        corr_meas.at<float>(i) = meas.at<float>(i);
                    }
                }
                setSampleMeasurementNoiseWithValidity(kf, valid_mask);
                setSampleMeasurementNoiseWithValidity(kf, valid_mask);
                estimated = kf.correct(corr_meas);
                out_kf_action = "correct";
            }
        }

        if (det_fit_ok && kf_initialized) {
            out_coeff = fitPolynomialFromSamples(estimated);
            out_detected = (out_coeff != cv::Vec4d(0, 0, 0, 0));
            out_state = out_detected ? LaneState::GOOD : LaneState::NODET;
            out_quality = out_detected ? LaneQuality::GOOD : LaneQuality::BAD;
            out_availability = out_detected ? 1U : 0U;
            if (out_detected && calcLaneRangeM(pixels, out_lane_start_m, out_lane_end_m, out_span_m)) {
                out_view_range_m = std::max(0.0, out_span_m);
            } else {
                out_lane_start_m = 0.0;
                out_lane_end_m = 0.0;
                out_span_m = 0.0;
                out_view_range_m = 0.0;
            }
            return;
        }

        if (det_fit_ok && !kf_initialized) {
            out_kf_action = "hold_invalid";
        }

        if (kf_initialized && miss_count < hold_frames) {
            ++miss_count;
            estimated = pred.clone();
            out_coeff = fitPolynomialFromSamples(estimated);
            out_detected = (out_coeff != cv::Vec4d(0, 0, 0, 0));
            out_state = out_detected ? LaneState::WEAK : LaneState::NODET;
            out_quality = out_detected ? LaneQuality::WEAK : LaneQuality::BAD;
            out_availability = 0U;
            out_lane_start_m = 0.0;
            out_lane_end_m = 0.0;
            out_span_m = 0.0;
            out_view_range_m = 0.0;
            out_kf_action = "predict";
            return;
        }

        miss_count = hold_frames;
        kf_initialized = false;
        out_coeff = cv::Vec4d(0, 0, 0, 0);
        out_detected = false;
        out_state = LaneState::NODET;
        out_quality = LaneQuality::BAD;
        out_availability = 0U;
        out_lane_start_m = 0.0;
        out_lane_end_m = 0.0;
        out_span_m = 0.0;
        out_view_range_m = 0.0;
        out_kf_action = "reset";
    };

    processSide(det_fit_L, meas_samples_L_, valid_mask_L,
                kf_L_samples_, kf_samples_initialized_L_,
                pred_samples_L_, estimated_samples_L_,
                sample_miss_L_,
                cur_result_.coeffs_L, cur_result_.is_detected_L,
                cur_result_.state_L, cur_result_.quality_L,
                cur_result_.availability_L,
                cur_result_.view_range_m_L,
                cur_result_.lane_start_m_L, cur_result_.lane_end_m_L, cur_result_.span_m_L,
                pixels_L_, kf_action_L);

    processSide(det_fit_R, meas_samples_R_, valid_mask_R,
                kf_R_samples_, kf_samples_initialized_R_,
                pred_samples_R_, estimated_samples_R_,
                sample_miss_R_,
                cur_result_.coeffs_R, cur_result_.is_detected_R,
                cur_result_.state_R, cur_result_.quality_R,
                cur_result_.availability_R,
                cur_result_.view_range_m_R,
                cur_result_.lane_start_m_R, cur_result_.lane_end_m_R, cur_result_.span_m_R,
                pixels_R_, kf_action_R);

    if (config_.log_sample_kf_trace) {
        auto logSide = [this, n](const char side_tag,
                                 bool det_fit_ok,
                                 const cv::Mat& meas,
                                 const std::vector<uint8_t>& valid_mask,
                                 const std::string& kf_action) {
            std::ostringstream oss;
            int valid_count = 0;
            for (uint8_t v : valid_mask) {
                valid_count += (v != 0U) ? 1 : 0;
            }

            oss << "[kf_sample][f=" << frame_idx_ << "][" << side_tag << "]"
                << " act=" << kf_action
                << " fit=" << (det_fit_ok ? 1 : 0)
                << " valid=" << valid_count << "/" << n
                << " mask=";
            for (int i = 0; i < n; ++i) {
                oss << (((int)valid_mask.size() == n && valid_mask[i]) ? '1' : '0');
            }
            oss << " y=[";
            for (int i = 0; i < n; ++i) {
                if (i > 0) oss << ",";
                if (meas.empty() || meas.rows < n || (int)valid_mask.size() != n || !valid_mask[i]) {
                    oss << "-";
                } else {
                    oss << std::fixed << std::setprecision(2) << static_cast<double>(meas.at<float>(i));
                }
            }
            oss << "]";
            std::cout << oss.str() << std::endl;
        };

        logSide('L', det_fit_L, meas_samples_L_, valid_mask_L, kf_action_L);
        logSide('R', det_fit_R, meas_samples_R_, valid_mask_R, kf_action_R);
    }

    cur_result_.hard_fail_L = false;
    cur_result_.hard_fail_R = false;
    cur_result_.delta_y_m_L = 0.0;
    cur_result_.delta_y_m_R = 0.0;
    cur_result_.delta_kappa_L = 0.0;
    cur_result_.delta_kappa_R = 0.0;

    if (cur_result_.state_L == LaneState::GOOD || cur_result_.state_L == LaneState::WEAK) {
        last_coeffs_L_ = cur_result_.coeffs_L;
    }
    if (cur_result_.state_R == LaneState::GOOD || cur_result_.state_R == LaneState::WEAK) {
        last_coeffs_R_ = cur_result_.coeffs_R;
    }

    auto updateStreak = [](LaneState s, int& bad_streak, int& weak_streak) {
        switch (s) {
            case LaneState::NODET:
            case LaneState::BAD:
                bad_streak++;
                weak_streak = 0;
                break;
            case LaneState::WEAK:
                weak_streak++;
                bad_streak = 0;
                break;
            case LaneState::GOOD:
                weak_streak = 0;
                bad_streak = 0;
                break;
        }
    };
    updateStreak(cur_result_.state_L, bad_streak_L_, weak_streak_L_);
    updateStreak(cur_result_.state_R, bad_streak_R_, weak_streak_R_);

    prev_state_L_ = cur_result_.state_L;
    prev_state_R_ = cur_result_.state_R;
    prev_real_L_ = (cur_result_.state_L != LaneState::NODET);
    prev_real_R_ = (cur_result_.state_R != LaneState::NODET);

    drawLaneSampleKfDebug(estimated_samples_L_, cur_result_.is_detected_L,
                          estimated_samples_R_, cur_result_.is_detected_R);

    if (config_.log_gate_trace) {
        ld_helpers::printGateTraceFrameHeader(true, frame_idx_, ld_helpers::GateTraceProfile::MaskLite);
        ld_helpers::GateTraceSideData trace_L;
        trace_L.side = 'L';
        trace_L.det = det_fit_L;
        trace_L.rmse_known = false;
        trace_L.rmse_value = 0.0;
        trace_L.span_g_known = cur_result_.is_detected_L;
        trace_L.span_value = cur_result_.span_m_L;
        trace_L.st = stateToString(cur_result_.state_L);
        trace_L.kf = kf_action_L;
        trace_L.vr_known = cur_result_.is_detected_L;
        trace_L.vr_value = cur_result_.view_range_m_L;
        trace_L.av = (cur_result_.availability_L == 1U);
        ld_helpers::printGateTraceSide(true, trace_L, ld_helpers::GateTraceProfile::MaskLite);

        ld_helpers::GateTraceSideData trace_R;
        trace_R.side = 'R';
        trace_R.det = det_fit_R;
        trace_R.rmse_known = false;
        trace_R.rmse_value = 0.0;
        trace_R.span_g_known = cur_result_.is_detected_R;
        trace_R.span_value = cur_result_.span_m_R;
        trace_R.st = stateToString(cur_result_.state_R);
        trace_R.kf = kf_action_R;
        trace_R.vr_known = cur_result_.is_detected_R;
        trace_R.vr_value = cur_result_.view_range_m_R;
        trace_R.av = (cur_result_.availability_R == 1U);
        ld_helpers::printGateTraceSide(true, trace_R, ld_helpers::GateTraceProfile::MaskLite);
    }
}

void LaneLineDetector::resultsFromRawFit()
{
    auto calcRMSE_m = [this](const std::vector<cv::Point>& pixels, const cv::Vec4d& coeffs) -> double {
        if (pixels.empty()) return 1e9;
        double sum_sq = 0.0;
        for (const auto& p : pixels) {
            const cv::Point2d p_veh = bevPixToVehicle(cv::Point2d((double)p.x, (double)p.y));
            const double x_veh = p_veh.x;
            const double y_veh = p_veh.y;
            const double y_pred = coeffs[0]*x_veh*x_veh*x_veh + coeffs[1]*x_veh*x_veh +
                                  coeffs[2]*x_veh + coeffs[3];
            const double e = y_veh - y_pred;
            sum_sq += e * e;
        }
        return std::sqrt(sum_sq / pixels.size());
    };

    const bool det_fit_L = cur_result_.detected_fit_L;
    const bool det_fit_R = cur_result_.detected_fit_R;
    const bool pix_found_L = !pixels_L_.empty();
    const bool pix_found_R = !pixels_R_.empty();
    const int px_count_L = static_cast<int>(pixels_L_.size());
    const int px_count_R = static_cast<int>(pixels_R_.size());
    cur_result_.rmse_fit_m_L = det_fit_L ? calcRMSE_m(pixels_L_, cur_result_.coeffs_fit_L) : 0.0;
    cur_result_.rmse_fit_m_R = det_fit_R ? calcRMSE_m(pixels_R_, cur_result_.coeffs_fit_R) : 0.0;

    const int hold_frames = std::max(0, config_.rawfit_hold_frames);
    std::string src_L = "none";
    std::string src_R = "none";

    auto applyRawSide = [&](bool det_fit,
                            const cv::Vec4d& fit_coeff,
                            const std::vector<cv::Point>& pixels,
                            cv::Vec4d& out_coeff,
                            bool& out_detected,
                            LaneState& out_state,
                            LaneQuality& out_quality,
                            uint8_t& out_availability,
                            double& out_lane_start_m, double& out_lane_end_m, double& out_span_m, double& out_view_range_m,
                            cv::Vec4d& hold_coeff, bool& hold_valid, int& hold_miss,
                            double& hold_lane_start_m, double& hold_lane_end_m, double& hold_span_m, double& hold_view_range_m,
                            cv::Vec4d& last_coeff,
                            std::string& src) {
        if (det_fit) {
            out_coeff = fit_coeff;
            out_detected = true;
            out_state = LaneState::GOOD;
            out_quality = LaneQuality::GOOD;
            out_availability = 1U;
            if (calcLaneRangeM(pixels, out_lane_start_m, out_lane_end_m, out_span_m)) {
                out_view_range_m = std::max(0.0, out_span_m);
            } else {
                out_lane_start_m = 0.0;
                out_lane_end_m = 0.0;
                out_span_m = 0.0;
                out_view_range_m = 0.0;
            }
            hold_coeff = out_coeff;
            hold_valid = true;
            hold_miss = 0;
            hold_lane_start_m = out_lane_start_m;
            hold_lane_end_m = out_lane_end_m;
            hold_span_m = out_span_m;
            hold_view_range_m = out_view_range_m;
            last_coeff = out_coeff;
            src = "raw";
            return;
        }

        if (hold_valid && hold_miss < hold_frames) {
            ++hold_miss;
            out_coeff = hold_coeff;
            out_detected = true;
            out_state = LaneState::WEAK;
            out_quality = LaneQuality::WEAK;
            out_availability = 1U;
            out_lane_start_m = hold_lane_start_m;
            out_lane_end_m = hold_lane_end_m;
            out_span_m = hold_span_m;
            out_view_range_m = hold_view_range_m;
            src = "hold";
            return;
        }

        hold_valid = false;
        hold_miss = hold_frames;
        out_coeff = cv::Vec4d(0, 0, 0, 0);
        out_detected = false;
        out_state = LaneState::NODET;
        out_quality = LaneQuality::BAD;
        out_availability = 0U;
        out_lane_start_m = 0.0;
        out_lane_end_m = 0.0;
        out_span_m = 0.0;
        out_view_range_m = 0.0;
        src = "none";
    };

    applyRawSide(det_fit_L, cur_result_.coeffs_fit_L, pixels_L_,
                 cur_result_.coeffs_L, cur_result_.is_detected_L,
                 cur_result_.state_L, cur_result_.quality_L, cur_result_.availability_L,
                 cur_result_.lane_start_m_L, cur_result_.lane_end_m_L, cur_result_.span_m_L, cur_result_.view_range_m_L,
                 raw_hold_coeffs_L_, raw_hold_valid_L_, raw_hold_miss_L_,
                 raw_hold_lane_start_m_L_, raw_hold_lane_end_m_L_, raw_hold_span_m_L_, raw_hold_view_range_m_L_,
                 last_coeffs_L_, src_L);

    applyRawSide(det_fit_R, cur_result_.coeffs_fit_R, pixels_R_,
                 cur_result_.coeffs_R, cur_result_.is_detected_R,
                 cur_result_.state_R, cur_result_.quality_R, cur_result_.availability_R,
                 cur_result_.lane_start_m_R, cur_result_.lane_end_m_R, cur_result_.span_m_R, cur_result_.view_range_m_R,
                 raw_hold_coeffs_R_, raw_hold_valid_R_, raw_hold_miss_R_,
                 raw_hold_lane_start_m_R_, raw_hold_lane_end_m_R_, raw_hold_span_m_R_, raw_hold_view_range_m_R_,
                 last_coeffs_R_, src_R);

    cur_result_.hard_fail_L = false;
    cur_result_.hard_fail_R = false;
    cur_result_.delta_y_m_L = 0.0;
    cur_result_.delta_y_m_R = 0.0;
    cur_result_.delta_kappa_L = 0.0;
    cur_result_.delta_kappa_R = 0.0;

    auto updateStreak = [](LaneState s, int& bad_streak, int& weak_streak) {
        switch (s) {
            case LaneState::NODET:
            case LaneState::BAD:
                bad_streak++;
                weak_streak = 0;
                break;
            case LaneState::WEAK:
                weak_streak++;
                bad_streak = 0;
                break;
            case LaneState::GOOD:
                weak_streak = 0;
                bad_streak = 0;
                break;
        }
    };
    updateStreak(cur_result_.state_L, bad_streak_L_, weak_streak_L_);
    updateStreak(cur_result_.state_R, bad_streak_R_, weak_streak_R_);

    prev_state_L_ = cur_result_.state_L;
    prev_state_R_ = cur_result_.state_R;
    prev_real_L_ = cur_result_.is_detected_L;
    prev_real_R_ = cur_result_.is_detected_R;

    if (config_.log_gate_trace) {
        std::cout << "[track][f=" << frame_idx_ << "][mask_raw]" << std::endl;
        auto printSide = [&](char side, bool pix_found, int px_count, bool det_fit, const std::string& src,
                             LaneState state, uint8_t availability, int hold_miss,
                             double span_m, double vr_m, double rmse_m) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2);
            oss << side << "{pix=" << (pix_found ? "T" : "F")
                << ",px=" << px_count
                << ",fit=" << (det_fit ? "T" : "F")
                << ",src=" << src
                << ",q=" << stateToString(state)
                << ",av=" << ((availability == 1U) ? "T" : "F")
                << ",miss=" << hold_miss << "/" << hold_frames;
            if (det_fit) {
                oss << ",rmse=" << rmse_m;
            }
            if (state != LaneState::NODET) {
                oss << ",span=" << span_m
                    << ",vr=" << vr_m;
            }
            oss << "}";
            std::cout << oss.str() << std::endl;
        };

        printSide('L', pix_found_L, px_count_L, det_fit_L, src_L, cur_result_.state_L, cur_result_.availability_L,
                  raw_hold_miss_L_, cur_result_.span_m_L, cur_result_.view_range_m_L, cur_result_.rmse_fit_m_L);
        printSide('R', pix_found_R, px_count_R, det_fit_R, src_R, cur_result_.state_R, cur_result_.availability_R,
                  raw_hold_miss_R_, cur_result_.span_m_R, cur_result_.view_range_m_R, cur_result_.rmse_fit_m_R);
    }
}

void LaneLineDetector::validateAndSmoothMaskLite()
{
    const cv::Vec4d fit_L = cur_result_.coeffs_fit_L;
    const cv::Vec4d fit_R = cur_result_.coeffs_fit_R;

    auto calcRMSE_m = [this](const std::vector<cv::Point>& pixels, const cv::Vec4d& coeffs) -> double {
        if (pixels.empty()) return 1e9;
        double sum_sq = 0.0;
        for (const auto& p : pixels) {
            const cv::Point2d p_veh = bevPixToVehicle(cv::Point2d((double)p.x, (double)p.y));
            const double x_veh = p_veh.x;
            const double y_veh = p_veh.y;
            const double y_pred = coeffs[0]*x_veh*x_veh*x_veh + coeffs[1]*x_veh*x_veh +
                                  coeffs[2]*x_veh + coeffs[3];
            const double e = y_veh - y_pred;
            sum_sq += e * e;
        }
        return std::sqrt(sum_sq / pixels.size());
    };

    const bool det_fit_L = cur_result_.detected_fit_L;
    const bool det_fit_R = cur_result_.detected_fit_R;

    cur_result_.rmse_fit_m_L = det_fit_L ? calcRMSE_m(pixels_L_, fit_L) : 0.0;
    cur_result_.rmse_fit_m_R = det_fit_R ? calcRMSE_m(pixels_R_, fit_R) : 0.0;

    pred_L_ = kf_initialized_L_ ? kf_L_.predict() : cv::Mat::zeros(8, 1, CV_32F);
    pred_R_ = kf_initialized_R_ ? kf_R_.predict() : cv::Mat::zeros(8, 1, CV_32F);

    auto matToVec4d = [](const cv::Mat& m) -> cv::Vec4d {
        return cv::Vec4d(m.at<float>(0), m.at<float>(1),
                         m.at<float>(2), m.at<float>(3));
    };

    auto kfSeed = [](cv::KalmanFilter& kf, const cv::Vec4d& fit) {
        kf.statePost.at<float>(0) = (float)fit[0];
        kf.statePost.at<float>(1) = (float)fit[1];
        kf.statePost.at<float>(2) = (float)fit[2];
        kf.statePost.at<float>(3) = (float)fit[3];
        kf.statePost.at<float>(4) = 0.0f;
        kf.statePost.at<float>(5) = 0.0f;
        kf.statePost.at<float>(6) = 0.0f;
        kf.statePost.at<float>(7) = 0.0f;
        kf.statePre = kf.statePost.clone();
    };

    auto kfCorrect = [](cv::KalmanFilter& kf, cv::Mat& meas,
                        const cv::Vec4d& fit, double R_val, cv::Mat& out) {
        meas.at<float>(0) = (float)fit[0];
        meas.at<float>(1) = (float)fit[1];
        meas.at<float>(2) = (float)fit[2];
        meas.at<float>(3) = (float)fit[3];
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all((float)R_val));
        out = kf.correct(meas);
    };

    const int hold_frames = std::max(1, config_.max_bad_frames);
    std::string kf_action_L = "none";
    std::string kf_action_R = "none";

    auto processSide = [&](bool det_fit_ok, const cv::Vec4d& fit,
                           cv::KalmanFilter& kf, bool& kf_initialized,
                           cv::Mat& meas, cv::Mat& pred, cv::Mat& estimated,
                           int& miss_count,
                           cv::Vec4d& out_coeff, bool& out_detected,
                           LaneState& out_state, LaneQuality& out_quality,
                           std::string& out_kf_action) {
        if (det_fit_ok) {
            miss_count = 0;
            if (!kf_initialized) {
                kfSeed(kf, fit);
                kf_initialized = true;
                out_coeff = fit;
                out_kf_action = "seed";
            } else {
                kfCorrect(kf, meas, fit, config_.kf_R_weak, estimated);
                out_coeff = matToVec4d(estimated);
                out_kf_action = "correct";
            }
            out_detected = true;
            out_state = LaneState::GOOD;
            out_quality = LaneQuality::GOOD;
            return;
        }

        if (kf_initialized && miss_count < hold_frames) {
            ++miss_count;
            out_coeff = matToVec4d(pred);
            out_detected = true;
            out_state = LaneState::WEAK;
            out_quality = LaneQuality::WEAK;
            out_kf_action = "predict";
            return;
        }

        miss_count = hold_frames;
        kf_initialized = false;
        out_coeff = cv::Vec4d(0, 0, 0, 0);
        out_detected = false;
        out_state = LaneState::NODET;
        out_quality = LaneQuality::BAD;
        out_kf_action = "reset";
    };

    processSide(det_fit_L, fit_L,
                kf_L_, kf_initialized_L_,
                meas_L_, pred_L_, estimated_L_,
                mask_lite_miss_L_,
                cur_result_.coeffs_L, cur_result_.is_detected_L,
                cur_result_.state_L, cur_result_.quality_L, kf_action_L);

    processSide(det_fit_R, fit_R,
                kf_R_, kf_initialized_R_,
                meas_R_, pred_R_, estimated_R_,
                mask_lite_miss_R_,
                cur_result_.coeffs_R, cur_result_.is_detected_R,
                cur_result_.state_R, cur_result_.quality_R, kf_action_R);

    cur_result_.hard_fail_L = false;
    cur_result_.hard_fail_R = false;
    cur_result_.delta_y_m_L = 0.0;
    cur_result_.delta_y_m_R = 0.0;
    cur_result_.delta_kappa_L = 0.0;
    cur_result_.delta_kappa_R = 0.0;
    if (cur_result_.is_detected_L &&
        calcLaneRangeM(pixels_L_, cur_result_.lane_start_m_L, cur_result_.lane_end_m_L, cur_result_.span_m_L)) {
        // populated in calcLaneRangeM
    } else {
        cur_result_.lane_start_m_L = 0.0;
        cur_result_.lane_end_m_L = 0.0;
        cur_result_.span_m_L = 0.0;
    }
    if (cur_result_.is_detected_R &&
        calcLaneRangeM(pixels_R_, cur_result_.lane_start_m_R, cur_result_.lane_end_m_R, cur_result_.span_m_R)) {
        // populated in calcLaneRangeM
    } else {
        cur_result_.lane_start_m_R = 0.0;
        cur_result_.lane_end_m_R = 0.0;
        cur_result_.span_m_R = 0.0;
    }
    cur_result_.view_range_m_L = std::max(0.0, cur_result_.span_m_L);
    cur_result_.view_range_m_R = std::max(0.0, cur_result_.span_m_R);

    auto qualityUsable = [](LaneQuality q) {
        return (q == LaneQuality::GOOD || q == LaneQuality::WEAK);
    };
    cur_result_.availability_L = (det_fit_L && qualityUsable(cur_result_.quality_L)) ? 1U : 0U;
    cur_result_.availability_R = (det_fit_R && qualityUsable(cur_result_.quality_R)) ? 1U : 0U;

    if (cur_result_.state_L == LaneState::GOOD || cur_result_.state_L == LaneState::WEAK) {
        last_coeffs_L_ = cur_result_.coeffs_L;
    }
    if (cur_result_.state_R == LaneState::GOOD || cur_result_.state_R == LaneState::WEAK) {
        last_coeffs_R_ = cur_result_.coeffs_R;
    }

    auto updateStreak = [](LaneState s, int& bad_streak, int& weak_streak) {
        switch (s) {
            case LaneState::NODET:
            case LaneState::BAD:
                bad_streak++;
                weak_streak = 0;
                break;
            case LaneState::WEAK:
                weak_streak++;
                bad_streak = 0;
                break;
            case LaneState::GOOD:
                weak_streak = 0;
                bad_streak = 0;
                break;
        }
    };
    updateStreak(cur_result_.state_L, bad_streak_L_, weak_streak_L_);
    updateStreak(cur_result_.state_R, bad_streak_R_, weak_streak_R_);

    prev_state_L_ = cur_result_.state_L;
    prev_state_R_ = cur_result_.state_R;
    prev_real_L_ = (cur_result_.state_L != LaneState::NODET);
    prev_real_R_ = (cur_result_.state_R != LaneState::NODET);

    if (config_.log_gate_trace) {
        ld_helpers::printGateTraceFrameHeader(true, frame_idx_, ld_helpers::GateTraceProfile::MaskLite);
        ld_helpers::GateTraceSideData trace_L;
        trace_L.side = 'L';
        trace_L.det = det_fit_L;
        trace_L.rmse_known = det_fit_L;
        trace_L.rmse_value = cur_result_.rmse_fit_m_L;
        trace_L.span_g_known = cur_result_.is_detected_L;
        trace_L.span_value = cur_result_.span_m_L;
        trace_L.st = stateToString(cur_result_.state_L);
        trace_L.kf = kf_action_L;
        trace_L.vr_known = cur_result_.is_detected_L;
        trace_L.vr_value = cur_result_.view_range_m_L;
        trace_L.av = (cur_result_.availability_L == 1U);
        ld_helpers::printGateTraceSide(true, trace_L, ld_helpers::GateTraceProfile::MaskLite);

        ld_helpers::GateTraceSideData trace_R;
        trace_R.side = 'R';
        trace_R.det = det_fit_R;
        trace_R.rmse_known = det_fit_R;
        trace_R.rmse_value = cur_result_.rmse_fit_m_R;
        trace_R.span_g_known = cur_result_.is_detected_R;
        trace_R.span_value = cur_result_.span_m_R;
        trace_R.st = stateToString(cur_result_.state_R);
        trace_R.kf = kf_action_R;
        trace_R.vr_known = cur_result_.is_detected_R;
        trace_R.vr_value = cur_result_.view_range_m_R;
        trace_R.av = (cur_result_.availability_R == 1U);
        ld_helpers::printGateTraceSide(true, trace_R, ld_helpers::GateTraceProfile::MaskLite);
    }
}

/* svd vs 누적식+QR 성능검증용 */
cv::Vec4d LaneLineDetector::calcLaneCoeffs_org(const std::vector<cv::Point>& pixels) {
    if (pixels.size() < config_.minpix) return cv::Vec4d(0,0,0,0);

    // 가중 회귀용 시스템 행렬
    cv::Mat Y_mat(pixels.size(), 1, CV_64F);
    cv::Mat X_mat(pixels.size(), 4, CV_64F);

    for (size_t i=0; i<pixels.size(); i++) {
        double u = (double)pixels[i].x; 
        double v = (double)pixels[i].y;

        // 좌표 변환 (BEV px -> rear-axle vehicle 좌표)
        const cv::Point2d p_veh = bevPixToVehicle(cv::Point2d(u, v));
        const double x_veh = p_veh.x;
        const double y_veh = p_veh.y;

        // [핵심] 가중치 계산: 차량에 가까울수록(v가 클수록) 가중치 높게
        // 예: 이미지 바닥(v=720)이면 weight=1.0, 위쪽(v=0)이면 weight=0.1
        double weight = (v / config_.bev_size.height) * 0.9 + 0.1; 
        
        // 행렬 채우기
        // sqrt(weight)를 데이터 항에 곱해 가중 최소제곱 형태로 구성
        
        // (참고: OpenCV solve는 가중치 옵션이 없으므로, 데이터 자체에 가중치를 곱해서 넘겨야 함)
        // Weighted Linear Regression: minimize sum( w_i * (y_i - f(x_i))^2 )
        // -> sqrt(w_i) * y_i = sqrt(w_i) * f(x_i)
        
        double sw = sqrt(weight);

        Y_mat.at<double>(i, 0) = y_veh * sw;

        X_mat.at<double>(i, 0) = (x_veh * x_veh * x_veh) * sw;
        X_mat.at<double>(i, 1) = (x_veh * x_veh) * sw;
        X_mat.at<double>(i, 2) = (x_veh) * sw;
        X_mat.at<double>(i, 3) = 1.0 * sw;
    }

    cv::Mat coeffs;
    if (!cv::solve(X_mat, Y_mat, coeffs, cv::DECOMP_SVD)) return cv::Vec4d(0,0,0,0);

    return cv::Vec4d(
        coeffs.at<double>(0), coeffs.at<double>(1), 
        coeffs.at<double>(2), coeffs.at<double>(3)
    );
}



cv::Vec4d LaneLineDetector::calcLaneCoeffs(const std::vector<cv::Point>& pixels) {
    if (pixels.size() < config_.minpix) return cv::Vec4d(0,0,0,0);

    // 누적식: A = X^T W X (4x4), b = X^T W y (4x1)
    cv::Matx44d A = cv::Matx44d::zeros();
    cv::Vec4d b(0.0, 0.0, 0.0, 0.0);

    for (size_t i=0; i<pixels.size(); i++) {
        double u = (double)pixels[i].x; 
        double v = (double)pixels[i].y;

        // 좌표 변환 (BEV px -> rear-axle vehicle 좌표)
        const cv::Point2d p_veh = bevPixToVehicle(cv::Point2d(u, v));
        const double x_veh = p_veh.x;
        const double y_veh = p_veh.y;

        // 가중치 계산: 차량에 가까울수록(v가 클수록) 가중치 높게
        double weight = (v / config_.bev_size.height) * 0.9 + 0.1; 
        
        const double phi[4] = {
            x_veh * x_veh * x_veh,
            x_veh * x_veh,
            x_veh,
            1.0
        };

        // b += w * phi * y
        for (int r = 0; r < 4; ++r) {
            b[r] += weight * phi[r] * y_veh;
        }
        // A += w * phi * phi^T (상삼각만 누적 후 대칭 복원)
        for (int r = 0; r < 4; ++r) {
            for (int c = r; c < 4; ++c) {
                A(r, c) += weight * phi[r] * phi[c];
            }
        }
    }

    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < r; ++c) {
            A(r, c) = A(c, r);
        }
    }

    // 수치 안정성을 위한 미세 대각 regularization
    constexpr double kLambda = 1e-9;
    for (int i = 0; i < 4; ++i) {
        A(i, i) += kLambda;
    }

    cv::Mat A_mat(4, 4, CV_64F);
    cv::Mat b_mat(4, 1, CV_64F);
    for (int r = 0; r < 4; ++r) {
        b_mat.at<double>(r, 0) = b[r];
        for (int c = 0; c < 4; ++c) {
            A_mat.at<double>(r, c) = A(r, c);
        }
    }

    cv::Mat coeffs;
    if (!cv::solve(A_mat, b_mat, coeffs, cv::DECOMP_QR)) return cv::Vec4d(0,0,0,0);

    return cv::Vec4d(
        coeffs.at<double>(0), coeffs.at<double>(1), 
        coeffs.at<double>(2), coeffs.at<double>(3)
    );
}

void LaneLineDetector::validateAndSmooth()
{
    cv::Vec4d fit_L = cur_result_.coeffs_fit_L;
    cv::Vec4d fit_R = cur_result_.coeffs_fit_R;

    /* ========== KF Predict (초기화된 경우에만 호출) ========== */
    pred_L_ = kf_initialized_L_ ? kf_L_.predict() : cv::Mat::zeros(8, 1, CV_32F);
    pred_R_ = kf_initialized_R_ ? kf_R_.predict() : cv::Mat::zeros(8, 1, CV_32F);

    const bool det_fit_L = cur_result_.detected_fit_L;
    const bool det_fit_R = cur_result_.detected_fit_R;
    bool det_L = det_fit_L;
    bool det_R = det_fit_R;
    bool pass_exp_L = true, pass_exp_R = true;
    bool pass_width_L = true, pass_width_R = true;
    bool width_known = false;
    bool dy_known_L = false, dy_known_R = false;
    bool dk_known_L = false, dk_known_R = false;
    bool pass_dy_L = true, pass_dy_R = true;
    bool pass_dk_L = true, pass_dk_R = true;

    /* ========== AXIS A: Validity Gate (치명 → NODET) ========== */

    // A1: coeffs_fit == (0,0,0,0) → 이미 det=false
    // A2: 발산 검사 비활성화 (pass_exp_*는 true 유지)

    /* 유틸 람다 */
    auto polyEval = [](const cv::Vec4d& c, double x) -> double {
        return c[0]*x*x*x + c[1]*x*x + c[2]*x + c[3];
    };

    auto calcRMSE_m = [this](const std::vector<cv::Point>& pixels, const cv::Vec4d& coeffs) -> double {
        if (pixels.empty()) return 1e9;
        double sum_sq = 0.0;
        for (const auto& p : pixels) {
            const cv::Point2d p_veh = bevPixToVehicle(cv::Point2d((double)p.x, (double)p.y));
            const double x_veh = p_veh.x;
            const double y_veh = p_veh.y;
            double y_pred = coeffs[0]*x_veh*x_veh*x_veh + coeffs[1]*x_veh*x_veh +
                           coeffs[2]*x_veh + coeffs[3];
            double e = y_veh - y_pred;
            sum_sq += e * e;
        }
        return std::sqrt(sum_sq / pixels.size());
    };

    // RMSE (fit 기준, 미터)
    double rmse_L = det_L ? calcRMSE_m(pixels_L_, fit_L) : 0.0;
    double rmse_R = det_R ? calcRMSE_m(pixels_R_, fit_R) : 0.0;
    cur_result_.rmse_fit_m_L = rmse_L;
    cur_result_.rmse_fit_m_R = rmse_R;

    // A3: 폭(Width) 검사 (양쪽 모두 있을 때)
    if (det_L && det_R) {
        width_known = true;
        double yL = polyEval(fit_L, config_.x_check);
        double yR = polyEval(fit_R, config_.x_check);
        double width = yL - yR;
        const bool width_in_range = (width > 0.0 &&
            width >= config_.lane_w_min && width <= config_.lane_w_max);
        pass_width_L = width_in_range;
        pass_width_R = width_in_range;

        if (width <= 0) {
            if (rmse_L > rmse_R) det_L = false;
            else det_R = false;
        } else if (width < config_.lane_w_min || width > config_.lane_w_max) {
            if (rmse_L > rmse_R * 1.2) det_L = false;
            else if (rmse_R > rmse_L * 1.2) det_R = false;
        }
    }

    /* ========== AXIS B: Sufficiency Gate (span 계산) ========== */
    double span_L = 0.0, span_R = 0.0;
    if (det_L && calcLaneRangeM(pixels_L_, cur_result_.lane_start_m_L, cur_result_.lane_end_m_L, span_L)) {
        // populated
    } else {
        cur_result_.lane_start_m_L = 0.0;
        cur_result_.lane_end_m_L = 0.0;
        span_L = 0.0;
    }
    if (det_R && calcLaneRangeM(pixels_R_, cur_result_.lane_start_m_R, cur_result_.lane_end_m_R, span_R)) {
        // populated
    } else {
        cur_result_.lane_start_m_R = 0.0;
        cur_result_.lane_end_m_R = 0.0;
        span_R = 0.0;
    }
    cur_result_.span_m_L = span_L;
    cur_result_.span_m_R = span_R;

    /* ========== AXIS C: Quality Hard Gate (RMSE 치명) ========== */
    bool hard_fail_L = false, hard_fail_R = false;

    if (det_L && rmse_L > config_.rmse_hard_bad_m) hard_fail_L = true;
    if (det_R && rmse_R > config_.rmse_hard_bad_m) hard_fail_R = true;

    /* ========== AXIS D: Temporal Consistency Gate ========== */
    double dy_L = 0.0, dy_R = 0.0;
    double dk_L = 0.0, dk_R = 0.0;

    // Left: last_coeffs가 유효할 때만 연속성 검사
    if (det_L && last_coeffs_L_ != cv::Vec4d(0,0,0,0)) {
        dy_known_L = true;
        dk_known_L = true;
        double y_cur  = polyEval(fit_L, config_.x_check);
        double y_prev = polyEval(last_coeffs_L_, config_.x_check);
        dy_L = std::abs(y_cur - y_prev);
        pass_dy_L = (dy_L <= config_.dy_hard_m);
        if (!pass_dy_L) hard_fail_L = true;

        double k_cur  = calcCurvatureAt(fit_L, config_.x_check);
        double k_prev = calcCurvatureAt(last_coeffs_L_, config_.x_check);
        dk_L = std::abs(k_cur - k_prev);
        pass_dk_L = (dk_L <= config_.dkappa_hard);
        if (!pass_dk_L) hard_fail_L = true;
    }

    // Right
    if (det_R && last_coeffs_R_ != cv::Vec4d(0,0,0,0)) {
        dy_known_R = true;
        dk_known_R = true;
        double y_cur  = polyEval(fit_R, config_.x_check);
        double y_prev = polyEval(last_coeffs_R_, config_.x_check);
        dy_R = std::abs(y_cur - y_prev);
        pass_dy_R = (dy_R <= config_.dy_hard_m);
        if (!pass_dy_R) hard_fail_R = true;

        double k_cur  = calcCurvatureAt(fit_R, config_.x_check);
        double k_prev = calcCurvatureAt(last_coeffs_R_, config_.x_check);
        dk_R = std::abs(k_cur - k_prev);
        pass_dk_R = (dk_R <= config_.dkappa_hard);
        if (!pass_dk_R) hard_fail_R = true;
    }

    cur_result_.delta_y_m_L = dy_L;
    cur_result_.delta_y_m_R = dy_R;
    cur_result_.delta_kappa_L = dk_L;
    cur_result_.delta_kappa_R = dk_R;
    cur_result_.hard_fail_L = hard_fail_L;
    cur_result_.hard_fail_R = hard_fail_R;

    /* ========== State Determination (4축 종합) ========== */
    LaneState state_L = determineLaneState(det_L, hard_fail_L, cur_result_.pixel_count_L, rmse_L, span_L);
    LaneState state_R = determineLaneState(det_R, hard_fail_R, cur_result_.pixel_count_R, rmse_R, span_R);
    cur_result_.state_L = state_L;
    cur_result_.state_R = state_R;

    if (config_.enable_csv_log && csv_initialized_) {
        const size_t px_L = cur_result_.pixel_count_L;
        const size_t px_R = cur_result_.pixel_count_R;

        csv_file_
            << frame_idx_ << ",L,"
            << stateToString(state_L) << ","
            << det_L << ","
            << px_L << ","
            << span_L << ","
            << rmse_L << ","
            << dy_L << ","
            << dk_L << ","
            << hard_fail_L << "\n";

        csv_file_
            << frame_idx_ << ",R,"
            << stateToString(state_R) << ","
            << det_R << ","
            << px_R << ","
            << span_R << ","
            << rmse_R << ","
            << dy_R << ","
            << dk_R << ","
            << hard_fail_R << "\n";
    }

    /* ========== KF 정책 (상태별 분기) ========== */

    // KF correct 헬퍼: 측정값 세팅 + R 조정 + correct
    auto kfCorrect = [](cv::KalmanFilter& kf, cv::Mat& meas,
                        const cv::Vec4d& fit, double R_val, cv::Mat& out) {
        meas.at<float>(0) = (float)fit[0];
        meas.at<float>(1) = (float)fit[1];
        meas.at<float>(2) = (float)fit[2];
        meas.at<float>(3) = (float)fit[3];
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all((float)R_val));
        out = kf.correct(meas);
    };

    auto matToVec4d = [](const cv::Mat& m) -> cv::Vec4d {
        return cv::Vec4d(m.at<float>(0), m.at<float>(1),
                         m.at<float>(2), m.at<float>(3));
    };

    auto kfSeed = [](cv::KalmanFilter& kf, const cv::Vec4d& fit) {
        kf.statePost.at<float>(0) = (float)fit[0];
        kf.statePost.at<float>(1) = (float)fit[1];
        kf.statePost.at<float>(2) = (float)fit[2];
        kf.statePost.at<float>(3) = (float)fit[3];
        // velocity state는 초기 0으로 시작
        kf.statePost.at<float>(4) = 0.0f;
        kf.statePost.at<float>(5) = 0.0f;
        kf.statePost.at<float>(6) = 0.0f;
        kf.statePost.at<float>(7) = 0.0f;
        kf.statePre = kf.statePost.clone();
    };

    std::string kf_action_L = "none";
    std::string kf_action_R = "none";

    // --- Left ---
    switch (state_L) {
        case LaneState::GOOD:
            if (!kf_initialized_L_) {
                kfSeed(kf_L_, fit_L);
                kf_initialized_L_ = true;
                cur_result_.coeffs_L = fit_L;
                kf_action_L = "seed";
            } else {
                kfCorrect(kf_L_, meas_L_, fit_L, config_.kf_R_good, estimated_L_);
                cur_result_.coeffs_L = matToVec4d(estimated_L_);
                kf_action_L = "correct";
            }
            cur_result_.is_detected_L = true;
            break;
        case LaneState::WEAK:
            if (!kf_initialized_L_) {
                kfSeed(kf_L_, fit_L);
                kf_initialized_L_ = true;
                cur_result_.coeffs_L = fit_L;
                kf_action_L = "seed";
            } else {
                kfCorrect(kf_L_, meas_L_, fit_L, config_.kf_R_weak, estimated_L_);
                cur_result_.coeffs_L = matToVec4d(estimated_L_);
                kf_action_L = "correct";
            }
            cur_result_.is_detected_L = true;
            break;
        case LaneState::BAD:
            // KF 미초기화 상태에서는 predict를 쓰지 않음
            if (kf_initialized_L_) {
                cur_result_.coeffs_L = matToVec4d(pred_L_);
                cur_result_.is_detected_L = true;
                kf_action_L = "predict";
            } else {
                cur_result_.coeffs_L = cv::Vec4d(0,0,0,0);
                cur_result_.is_detected_L = false;
                kf_action_L = "none";
            }
            break;
        case LaneState::NODET:
            cur_result_.coeffs_L = cv::Vec4d(0,0,0,0);
            cur_result_.is_detected_L = false;
            kf_action_L = "none";
            break;
    }

    // --- Right ---
    switch (state_R) {
        case LaneState::GOOD:
            if (!kf_initialized_R_) {
                kfSeed(kf_R_, fit_R);
                kf_initialized_R_ = true;
                cur_result_.coeffs_R = fit_R;
                kf_action_R = "seed";
            } else {
                kfCorrect(kf_R_, meas_R_, fit_R, config_.kf_R_good, estimated_R_);
                cur_result_.coeffs_R = matToVec4d(estimated_R_);
                kf_action_R = "correct";
            }
            cur_result_.is_detected_R = true;
            break;
        case LaneState::WEAK:
            if (!kf_initialized_R_) {
                kfSeed(kf_R_, fit_R);
                kf_initialized_R_ = true;
                cur_result_.coeffs_R = fit_R;
                kf_action_R = "seed";
            } else {
                kfCorrect(kf_R_, meas_R_, fit_R, config_.kf_R_weak, estimated_R_);
                cur_result_.coeffs_R = matToVec4d(estimated_R_);
                kf_action_R = "correct";
            }
            cur_result_.is_detected_R = true;
            break;
        case LaneState::BAD:
            if (kf_initialized_R_) {
                cur_result_.coeffs_R = matToVec4d(pred_R_);
                cur_result_.is_detected_R = true;
                kf_action_R = "predict";
            } else {
                cur_result_.coeffs_R = cv::Vec4d(0,0,0,0);
                cur_result_.is_detected_R = false;
                kf_action_R = "none";
            }
            break;
        case LaneState::NODET:
            cur_result_.coeffs_R = cv::Vec4d(0,0,0,0);
            cur_result_.is_detected_R = false;
            kf_action_R = "none";
            break;
    }

    /* ========== quality_L/R 하위호환 동기화 ========== */
    auto stateToQuality = [](LaneState s) -> LaneQuality {
        switch (s) {
            case LaneState::GOOD: return LaneQuality::GOOD;
            case LaneState::WEAK: return LaneQuality::WEAK;
            default: return LaneQuality::BAD;
        }
    };
    cur_result_.quality_L = stateToQuality(state_L);
    cur_result_.quality_R = stateToQuality(state_R);
    cur_result_.view_range_m_L = std::max(0.0, span_L);
    cur_result_.view_range_m_R = std::max(0.0, span_R);

    auto qualityUsable = [](LaneQuality q) {
        return (q == LaneQuality::GOOD || q == LaneQuality::WEAK);
    };
    cur_result_.availability_L =
        (state_L != LaneState::NODET &&
         qualityUsable(cur_result_.quality_L) &&
         !hard_fail_L &&
         cur_result_.view_range_m_L >= config_.valid_view_range) ? 1U : 0U;
    cur_result_.availability_R =
        (state_R != LaneState::NODET &&
         qualityUsable(cur_result_.quality_R) &&
         !hard_fail_R &&
         cur_result_.view_range_m_R >= config_.valid_view_range) ? 1U : 0U;

    /* ========== 트래킹 상태 관리 (디버그용) ========== */
    if (state_L == LaneState::NODET && state_R == LaneState::NODET) {
        bad_frame_count_++;
        if (bad_frame_count_ > config_.max_bad_frames) {
            is_tracking_ = false;
            bad_frame_count_ = 0;
            // 장기 미검출 시 KF 상태 리셋 (잘못된 관성 제거)
            initKalmanFilter();
        }
    } else {
        bad_frame_count_ = 0;
        is_tracking_ = true;
    }

    /* ========== last_coeffs 갱신 (GOOD/WEAK만 갱신, BAD/NODET는 앵커 동결) ========== */
    if (state_L == LaneState::GOOD || state_L == LaneState::WEAK) {
        last_coeffs_L_ = cur_result_.coeffs_L;
    }
    if (state_R == LaneState::GOOD || state_R == LaneState::WEAK) {
        last_coeffs_R_ = cur_result_.coeffs_R;
    }

    /* ========== Streak 갱신 (LaneState 기반) ========== */
    auto updateStreak = [](LaneState s, int& bad_streak, int& weak_streak) {
        switch (s) {
            case LaneState::NODET:
            case LaneState::BAD:
                bad_streak++;
                weak_streak = 0;
                break;
            case LaneState::WEAK:
                weak_streak++;
                bad_streak = 0;
                break;
            case LaneState::GOOD:
                weak_streak = 0;
                bad_streak = 0;
                break;
        }
    };
    updateStreak(state_L, bad_streak_L_, weak_streak_L_); 
    updateStreak(state_R, bad_streak_R_, weak_streak_R_);

    // prev 상태 저장 (다음 프레임 모드 선택용)
    prev_state_L_ = state_L;
    prev_state_R_ = state_R;
    prev_real_L_ = (state_L != LaneState::NODET);
    prev_real_R_ = (state_R != LaneState::NODET);

    if (config_.log_gate_trace) {
        const bool rmse_h_known_L = det_L;
        const bool rmse_h_known_R = det_R;
        const bool pass_rmse_h_L = !rmse_h_known_L || (rmse_L <= config_.rmse_hard_bad_m);
        const bool pass_rmse_h_R = !rmse_h_known_R || (rmse_R <= config_.rmse_hard_bad_m);

        const bool span_g_known_L = det_L;
        const bool span_g_known_R = det_R;
        const bool pass_span_g_L = !span_g_known_L || (span_L >= config_.span_min_m_good);
        const bool pass_span_g_R = !span_g_known_R || (span_R >= config_.span_min_m_good);

        const bool qw_known_L = det_L;
        const bool qw_known_R = det_R;
        const bool pass_qw_L = !qw_known_L ||
            (cur_result_.pixel_count_L >= (size_t)config_.quality_min_pixels_weak &&
             rmse_L <= config_.quality_max_rmse_m_weak);
        const bool pass_qw_R = !qw_known_R ||
            (cur_result_.pixel_count_R >= (size_t)config_.quality_min_pixels_weak &&
             rmse_R <= config_.quality_max_rmse_m_weak);

        const bool qg_known_L = det_L;
        const bool qg_known_R = det_R;
        const bool pass_qg_L = !qg_known_L ||
            (cur_result_.pixel_count_L >= (size_t)config_.quality_min_pixels_good &&
             rmse_L <= config_.quality_max_rmse_m_good);
        const bool pass_qg_R = !qg_known_R ||
            (cur_result_.pixel_count_R >= (size_t)config_.quality_min_pixels_good &&
             rmse_R <= config_.quality_max_rmse_m_good);

        const bool vr_known_L = (state_L != LaneState::NODET);
        const bool vr_known_R = (state_R != LaneState::NODET);
        const bool pass_vr_L = (cur_result_.view_range_m_L >= config_.valid_view_range);
        const bool pass_vr_R = (cur_result_.view_range_m_R >= config_.valid_view_range);

        ld_helpers::printGateTraceFrameHeader(true, frame_idx_);
        ld_helpers::GateTraceSideData trace_L;
        trace_L.side = 'L';
        trace_L.det = det_fit_L;
        trace_L.exp_known = det_fit_L;
        trace_L.exp_pass = pass_exp_L;
        trace_L.width_known = width_known;
        trace_L.width_pass = pass_width_L;
        trace_L.rmse_h_known = rmse_h_known_L;
        trace_L.rmse_h_pass = pass_rmse_h_L;
        trace_L.dy_known = dy_known_L;
        trace_L.dy_pass = pass_dy_L;
        trace_L.dk_known = dk_known_L;
        trace_L.dk_pass = pass_dk_L;
        trace_L.span_g_known = span_g_known_L;
        trace_L.span_g_pass = pass_span_g_L;
        trace_L.qw_known = qw_known_L;
        trace_L.qw_pass = pass_qw_L;
        trace_L.qg_known = qg_known_L;
        trace_L.qg_pass = pass_qg_L;
        trace_L.hf = hard_fail_L;
        trace_L.st = stateToString(state_L);
        trace_L.kf = kf_action_L;
        trace_L.vr_known = vr_known_L;
        trace_L.vr_pass = pass_vr_L;
        trace_L.av = (cur_result_.availability_L == 1U);
        ld_helpers::printGateTraceSide(true, trace_L);

        ld_helpers::GateTraceSideData trace_R;
        trace_R.side = 'R';
        trace_R.det = det_fit_R;
        trace_R.exp_known = det_fit_R;
        trace_R.exp_pass = pass_exp_R;
        trace_R.width_known = width_known;
        trace_R.width_pass = pass_width_R;
        trace_R.rmse_h_known = rmse_h_known_R;
        trace_R.rmse_h_pass = pass_rmse_h_R;
        trace_R.dy_known = dy_known_R;
        trace_R.dy_pass = pass_dy_R;
        trace_R.dk_known = dk_known_R;
        trace_R.dk_pass = pass_dk_R;
        trace_R.span_g_known = span_g_known_R;
        trace_R.span_g_pass = pass_span_g_R;
        trace_R.qw_known = qw_known_R;
        trace_R.qw_pass = pass_qw_R;
        trace_R.qg_known = qg_known_R;
        trace_R.qg_pass = pass_qg_R;
        trace_R.hf = hard_fail_R;
        trace_R.st = stateToString(state_R);
        trace_R.kf = kf_action_R;
        trace_R.vr_known = vr_known_R;
        trace_R.vr_pass = pass_vr_R;
        trace_R.av = (cur_result_.availability_R == 1U);
        ld_helpers::printGateTraceSide(true, trace_R);
    }

    /* ========== 디버그 로그 ========== */
    if (config_.log_frame_debug) {
        printf("[State] L=%s (det=%d px=%zu span=%.1f rmse=%.3f dy=%.2f dk=%.3f hf=%d) | "
               "R=%s (det=%d px=%zu span=%.1f rmse=%.3f dy=%.2f dk=%.3f hf=%d)\n",
            stateToString(state_L).c_str(), cur_result_.is_detected_L,
            cur_result_.pixel_count_L, span_L, rmse_L, dy_L, dk_L, (int)hard_fail_L,
            stateToString(state_R).c_str(), cur_result_.is_detected_R,
            cur_result_.pixel_count_R, span_R, rmse_R, dy_R, dk_R, (int)hard_fail_R);
    }
}

void LaneLineDetector::drawResults(cv::Mat& undist_img) { 
    buf_dbgimg = undist_img.clone();
    
    if (config_.show_undist) drawROI(buf_dbgimg);

    pts_bev_L.clear(); pts_bev_R.clear();
    pts_orig_L.clear(); pts_orig_R.clear();

    const bool draw_state_L = (cur_result_.state_L != LaneState::NODET);
    const bool draw_state_R = (cur_result_.state_R != LaneState::NODET);

    auto getPixelYRange = [](const std::vector<cv::Point>& pixels, int& min_y, int& max_y) -> bool {
        if (pixels.empty()) return false;
        auto [min_it, max_it] = std::minmax_element(
            pixels.begin(), pixels.end(),
            [](const cv::Point& a, const cv::Point& b) { return a.y < b.y; });
        min_y = min_it->y;
        max_y = max_it->y;
        return true;
    };

    auto getResultYRange = [&](double lane_start_m, double lane_end_m, int& min_y, int& max_y) -> bool {
        if (lane_end_m <= lane_start_m) return false;

        const double v_start = vehicleToBevPix(cv::Point2d(lane_start_m, 0.0)).y;
        const double v_end = vehicleToBevPix(cv::Point2d(lane_end_m, 0.0)).y;
        const double v_min = std::min(v_start, v_end);
        const double v_max = std::max(v_start, v_end);

        min_y = std::clamp(static_cast<int>(std::floor(v_min)), 0, config_.bev_size.height - 1);
        max_y = std::clamp(static_cast<int>(std::ceil(v_max)), 0, config_.bev_size.height - 1);
        return min_y <= max_y;
    };

    auto resolveDrawYRange = [&](const std::vector<cv::Point>& pixels,
                                 double lane_start_m, double lane_end_m,
                                 int& min_y, int& max_y) -> bool {
        if (getPixelYRange(pixels, min_y, max_y)) return true;
        return getResultYRange(lane_start_m, lane_end_m, min_y, max_y);
    };

    int min_y_L = 0, max_y_L = -1;
    int min_y_R = 0, max_y_R = -1;
    const bool has_range_L = resolveDrawYRange(
        pixels_L_, cur_result_.lane_start_m_L, cur_result_.lane_end_m_L, min_y_L, max_y_L);
    const bool has_range_R = resolveDrawYRange(
        pixels_R_, cur_result_.lane_start_m_R, cur_result_.lane_end_m_R, min_y_R, max_y_R);

    if ((draw_state_L && has_range_L) || (draw_state_R && has_range_R)) {
        
        // Bottom-Up 루프 (화면 아래 -> 위로 스캔)
        int step = 10; // 10픽셀 간격으로 점을 찍어 선을 만듦
        for (int v = config_.bev_size.height - 1; v >= 0; v -= step) {
            
            // 픽셀 좌표 v -> 차량 좌표계 x_veh (전방 거리) 변환
            const double x_veh = bevPixToVehicle(cv::Point2d(0.0, (double)v)).x;

            double y_L = 0, y_R = 0;
            bool draw_L = false, draw_R = false;

            // --- 왼쪽 차선 계산 ---
            if (draw_state_L && has_range_L) {
                // [조건 1] 실제 픽셀이 존재했던 Y 범위(min~max) 안에서만 그림
                if (v >= min_y_L && v <= max_y_L) {
                    auto cL = cur_result_.coeffs_L;
                    y_L = cL[0]*pow(x_veh,3) + cL[1]*pow(x_veh,2) + cL[2]*x_veh + cL[3];
                    
                    // [조건 2] 발산 방지: 차선이 중심에서 5m 이상 벗어나면 그리지 않음 (튀는 현상 방지)
                    if (std::abs(y_L) < 5.0) {
                        draw_L = true;
                    }
                }
            }

            // --- 오른쪽 차선 계산 ---
            if (draw_state_R && has_range_R) {
                if (v >= min_y_R && v <= max_y_R) {
                    auto cR = cur_result_.coeffs_R;
                    y_R = cR[0]*pow(x_veh,3) + cR[1]*pow(x_veh,2) + cR[2]*x_veh + cR[3];
                    
                    if (std::abs(y_R) < 5.0) {
                        draw_R = true;
                    }
                }
            }

            // --- 좌표 역변환 및 저장 ---
            // 왼쪽
            if (draw_L) {
                const double u_L = vehicleToBevPix(cv::Point2d(x_veh, y_L)).x;
                // 이미지 범위 체크
                if (u_L >= 0 && u_L < config_.bev_size.width) {
                    pts_bev_L.push_back(cv::Point2f((float)u_L, (float)v));
                }
            }
            
            // 오른쪽
            if (draw_R) {
                const double u_R = vehicleToBevPix(cv::Point2d(x_veh, y_R)).x;
                if (u_R >= 0 && u_R < config_.bev_size.width) {
                    pts_bev_R.push_back(cv::Point2f((float)u_R, (float)v));
                }
            }
        }
    }

    // 2. 역투영 및 선 그리기 (Polylines)
    if (!pts_bev_L.empty()) cv::perspectiveTransform(pts_bev_L, pts_orig_L, H_bev_inv);
    if (!pts_bev_R.empty()) cv::perspectiveTransform(pts_bev_R, pts_orig_R, H_bev_inv);

    std::vector<cv::Point> pts_poly_L, pts_poly_R;
    for (auto& p : pts_orig_L) pts_poly_L.push_back(p);
    for (auto& p : pts_orig_R) pts_poly_R.push_back(p);

    // 3. 선 그리기 (undist_img 위에 직접 그림 -> 불투명/진함)
    if (!pts_poly_L.empty())
        cv::polylines(undist_img, pts_poly_L, false, cv::Scalar(255, 0, 0), 8); // Blue
    if (!pts_poly_R.empty())
        cv::polylines(undist_img, pts_poly_R, false, cv::Scalar(0, 0, 255), 8); // Red

    pushDebugImage(undist_img);
    buf_dbgimg = undist_img;
}

int LaneLineDetector::showDebugWindow() {
    if (debug_.empty()) return -1;

    /* 그리드 크기 설정 */
    int cell_w = 270;
    int cell_h = 270;
    cv::Size cell_size(cell_w, cell_h);

    int cols = 4;
    int rows = (debug_.size() + cols - 1) / cols;

    /* 윈도우 준비 */
    int total_w = cell_w * cols;
    int total_h = cell_h * rows;

    if (display_window_.cols != total_w || display_window_.rows != total_h) {
        display_window_ = cv::Mat::zeros(total_h, total_w, CV_8UC3);
    } else {
        display_window_.setTo(cv::Scalar(0,0,0));
    }

    /* ROI에 복사 */
    for (size_t i=0; i<debug_.size(); ++i) {
        int r = i / cols;
        int c = i % cols;

        cv::Rect roi(c * cell_w, r * cell_h, cell_w, cell_h);
        cv::Mat target_region = display_window_(roi);

        if (debug_[i].img.channels()==1) {
            cv::cvtColor(debug_[i].img, buff_bgr_, cv::COLOR_GRAY2BGR);
            cv::resize(buff_bgr_, target_region, cell_size);
        } else {
            cv::resize(debug_[i].img, target_region, cell_size);
        }

        cv::putText(target_region, debug_[i].label,
                    cv::Point(10, cell_h - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 255), 1
                );
    }

    /* 띄우기 */
    cv::imshow("Lane Detection Process", display_window_);
    return cv::waitKey(config_.waitkey);
}
/* USE
    int key = LaneLineDetector.showDebugWindow();
    if (key != -1) break;
*/

cv::Mat LaneLineDetector::getDebugImage() {
    if (debug_.empty()) return cv::Mat();

    const int cell_w = 320;
    const int cell_h = 240;
    const int cols = 4;
    const int rows = (static_cast<int>(debug_.size()) + cols - 1) / cols;
    const int total_w = cell_w * cols;
    const int total_h = cell_h * rows;

    cv::Mat out = cv::Mat::zeros(total_h, total_w, CV_8UC3);
    cv::Mat tmp_bgr, scaled;

    for (size_t i = 0; i < debug_.size(); ++i) {
        const int r = static_cast<int>(i) / cols;
        const int c = static_cast<int>(i) % cols;
        cv::Mat cell = out(cv::Rect(c * cell_w, r * cell_h, cell_w, cell_h));

        if (debug_[i].img.channels() == 1) {
            cv::cvtColor(debug_[i].img, tmp_bgr, cv::COLOR_GRAY2BGR);
        } else {
            tmp_bgr = debug_[i].img;
        }

        const float scale = std::min(static_cast<float>(cell_w) / tmp_bgr.cols,
                                     static_cast<float>(cell_h) / tmp_bgr.rows);
        const int sw = static_cast<int>(tmp_bgr.cols * scale);
        const int sh = static_cast<int>(tmp_bgr.rows * scale);
        const int xo = (cell_w - sw) / 2;
        const int yo = (cell_h - sh) / 2;
        cv::resize(tmp_bgr, scaled, cv::Size(sw, sh));
        scaled.copyTo(cell(cv::Rect(xo, yo, sw, sh)));

        cv::putText(cell, debug_[i].label,
                    cv::Point(10, cell_h - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 255), 1);
    }

    return out;
}

void LaneLineDetector::drawROI(cv::Mat& img) {
    if (roi_pixel_pts_.size() != 4) return;

    std::vector<cv::Point> pts_int;
    for (auto& p : roi_pixel_pts_) pts_int.push_back(p); // 좌표가 int로 변환됨

    cv::line(img, pts_int[0], pts_int[1], config_.color_roi, config_.thickness_roi); 
    cv::line(img, pts_int[1], pts_int[2], config_.color_roi, config_.thickness_roi); 
    cv::line(img, pts_int[2], pts_int[3], config_.color_roi, config_.thickness_roi); 
    cv::line(img, pts_int[3], pts_int[0], config_.color_roi, config_.thickness_roi); 
}

void LaneLineDetector::drawPolyBev(cv::Mat& img, const cv::Scalar& color,
    const cv::Vec4d& cL, const cv::Vec4d& cR, bool det_L, bool det_R)
{
    if (img.empty()) return;

    std::vector<cv::Point> pts_L, pts_R;
    int step = 5;

    for (int v = 0; v < img.rows; v += step) {
        const double x_veh = bevPixToVehicle(cv::Point2d(0.0, (double)v)).x;

        if (det_L) {
            double y_veh = cL[0]*pow(x_veh, 3) + cL[1]*pow(x_veh, 2) + cL[2]*x_veh + cL[3];
            int u = (int)vehicleToBevPix(cv::Point2d(x_veh, y_veh)).x;
            if (u >= 0 && u < img.cols) {
                pts_L.push_back(cv::Point(u, v));
            }
        }

        if (det_R) {
            double y_veh = cR[0]*pow(x_veh, 3) + cR[1]*pow(x_veh, 2) + cR[2]*x_veh + cR[3];
            int u = (int)vehicleToBevPix(cv::Point2d(x_veh, y_veh)).x;
            if (u >= 0 && u < img.cols) {
                pts_R.push_back(cv::Point(u, v));
            }
        }
    }

    if (!pts_L.empty()) {
        const cv::Point* pts = &pts_L[0];
        int npts = (int)pts_L.size();
        cv::polylines(img, &pts, &npts, 1, false, color, 2);
    }

    if (!pts_R.empty()) {
        const cv::Point* pts = &pts_R[0];
        int npts = (int)pts_R.size();
        cv::polylines(img, &pts, &npts, 1, false, color, 2);
    }
}

void LaneLineDetector::applyRoiMask(cv::Mat& bev_bin)
{
    if (!config_.roi_mask_enabled) return;
    if (bev_bin.empty()) return;

    const int w = bev_bin.cols;
    const int h = bev_bin.rows;

    // 중앙 기준점: SW/SA와 동일한 mid_img 사용
    const int mid_img = w / 2 + config_.roi_mid_offset_px;

    // 사다리꼴 반폭 비율 (상단=원거리=넓음, 하단=근거리=좁음)
    const float top_ratio = config_.roi_half_top_ratio;
    const float bot_ratio = config_.roi_half_bot_ratio;

    // 마스크 생성
    cv::Mat mask = cv::Mat::zeros(bev_bin.size(), CV_8U);

    for (int y = 0; y < h; ++y) {
        // t: 0.0(상단/원거리) → 1.0(하단/근거리)
        const float t = static_cast<float>(y) / static_cast<float>(h);

        // 선형 보간: 상단에서 하단으로 갈수록 반폭 비율 감소
        const float half_ratio = top_ratio + (bot_ratio - top_ratio) * t;
        const int half_width = static_cast<int>(w * half_ratio);

        // 좌/우 영역 경계 계산 (클램프 적용)
        const int left_end = std::max(0, mid_img - half_width);
        const int right_start = std::min(w - 1, mid_img + half_width);

        // 좌측 영역 (0 ~ left_end) 흰색으로 채움
        cv::line(mask, cv::Point(0, y), cv::Point(left_end, y), 255, 1);

        // 우측 영역 (right_start ~ w-1) 흰색으로 채움
        cv::line(mask, cv::Point(right_start, y), cv::Point(w - 1, y), 255, 1);
    }

    // 마스크 적용: 중앙 영역 제외, 좌/우 가장자리만 남김
    cv::bitwise_and(bev_bin, mask, bev_bin);

    // 디버그: 마스크 시각화
    if (config_.show_contour) {
        cv::Mat mask_vis;
        cv::cvtColor(mask, mask_vis, cv::COLOR_GRAY2BGR);
        cv::addWeighted(bev, 0.7, mask_vis, 0.3, 0, mask_vis);
        // 중앙선 표시
        cv::line(mask_vis, cv::Point(mid_img, 0), cv::Point(mid_img, h - 1),
                 cv::Scalar(0, 255, 255), 1);
        pushDebugImage(mask_vis);
    }
}

void LaneLineDetector::pushDebugImageExec(const cv::Mat& img, std::string label)
{
    if (img.empty()) return;

    debug_.push_back({img.clone(), label});
}

/* ============================================
   픽셀 분리 유틸 함수 구현
   ============================================ */

std::pair<double, double> LaneLineDetector::enforceMinGap(double uL, double uR, double min_gap_px)
{
    // uR > uL이 정상 (왼쪽이 u 작은 쪽)
    if (uR <= uL) {
        std::swap(uL, uR);
    }

    double gap = uR - uL;
    if (gap < min_gap_px) {
        double mid = 0.5 * (uL + uR);
        uL = mid - 0.5 * min_gap_px;
        uR = mid + 0.5 * min_gap_px;
    }

    return {uL, uR};
}

PixelSide LaneLineDetector::classifyPixelToSide(double nx, double uL, double uR, double margin_px)
{
    double dist_L = std::abs(nx - uL);
    double dist_R = std::abs(nx - uR);

    bool in_L = (dist_L <= margin_px);
    bool in_R = (dist_R <= margin_px);

    if (!in_L && !in_R) {
        return PixelSide::NONE; // 둘 다 margin 밖
    }

    if (in_L && in_R) {
        // 둘 다 margin 내: 더 가까운 쪽 단일 선택
        return (dist_L < dist_R) ? PixelSide::LEFT : PixelSide::RIGHT;
    }

    // 하나만 margin 내
    return in_L ? PixelSide::LEFT : PixelSide::RIGHT;
}

LaneQuality LaneLineDetector::evaluateLaneQuality(size_t pixel_count, double rmse_m)
{
    /*
     * 차선 품질 평가: 픽셀 수와 RMSE(미터) 기반 3단계 등급
     *
     * GOOD: 픽셀 충분 + RMSE 작음 (신뢰도 높음)
     * WEAK: 픽셀 부족하거나 RMSE 중간 (부분 신뢰)
     * BAD:  픽셀 너무 적거나 RMSE 큼 (신뢰 불가)
     *
     * 입력: rmse_m은 미터 단위 (fit 기준)
     */

    // 픽셀 수 체크
    if (pixel_count < (size_t)config_.quality_min_pixels_weak) {
        return LaneQuality::BAD;
    }

    // RMSE 체크 (미터 단위)
    if (rmse_m > config_.quality_max_rmse_m_weak) {
        return LaneQuality::BAD;
    }

    // GOOD 조건: 픽셀 충분 + RMSE 작음
    if (pixel_count >= (size_t)config_.quality_min_pixels_good &&
        rmse_m <= config_.quality_max_rmse_m_good) {
        return LaneQuality::GOOD;
    }

    // 나머지는 WEAK
    return LaneQuality::WEAK;
}

/* ============================================
   4축 게이팅 헬퍼 함수
   ============================================ */

double LaneLineDetector::calcSpanM(const std::vector<cv::Point>& pixels) const
{
    double start_m = 0.0, end_m = 0.0, span_m = 0.0;
    if (!calcLaneRangeM(pixels, start_m, end_m, span_m)) {
        return 0.0;
    }
    return span_m;
}

bool LaneLineDetector::calcLaneRangeM(const std::vector<cv::Point>& pixels,
                                      double& start_m, double& end_m, double& span_m) const
{
    start_m = 0.0;
    end_m = 0.0;
    span_m = 0.0;

    if (pixels.size() < 2) return false;

    int v_min = pixels[0].y;
    int v_max = pixels[0].y;
    for (const auto& p : pixels) {
        if (p.y < v_min) v_min = p.y;
        if (p.y > v_max) v_max = p.y;
    }

    // v_max(아래쪽) -> near, v_min(위쪽) -> far
    const double x_near = bevPixToVehicle(cv::Point2d(0.0, (double)v_max)).x;
    const double x_far  = bevPixToVehicle(cv::Point2d(0.0, (double)v_min)).x;

    start_m = std::min(x_near, x_far);
    end_m = std::max(x_near, x_far);
    span_m = end_m - start_m;
    return true;
}

double LaneLineDetector::calcCurvatureAt(const cv::Vec4d& c, double x) const
{
    // y  = c[0]*x³ + c[1]*x² + c[2]*x + c[3]
    // y' = 3*c[0]*x² + 2*c[1]*x + c[2]
    // y''= 6*c[0]*x + 2*c[1]
    // κ  = y'' / (1 + y'²)^(3/2)
    double yp  = 3.0 * c[0] * x * x + 2.0 * c[1] * x + c[2];
    double ypp = 6.0 * c[0] * x + 2.0 * c[1];
    double denom = std::pow(1.0 + yp * yp, 1.5);
    if (denom < 1e-9) return 0.0;
    return ypp / denom;
}

LaneState LaneLineDetector::determineLaneState(
    bool validity_pass, bool hard_fail,
    size_t pixel_count, double rmse_m, double span_m)
{
    /*
     * 4축 결과 종합 → 최종 LaneState 결정
     *
     * 순서:
     *   1) Validity 실패 → NODET (검출 성립 실패)
     *   2) Hard gate 위반 → BAD (검출 성립, 치명 위반)
     *   3) 소프트 기준으로 GOOD/WEAK/BAD 결정
     *      - span cap: span 부족 시 GOOD 불가
     *      - pixel/rmse 기반 등급
     */

    // (1) Validity gate
    if (!validity_pass) return LaneState::NODET;

    // (2) Hard gate (품질/연속성 치명 위반)
    if (hard_fail) return LaneState::BAD;

    // (3) Span cap: span 부족 시 최대 등급 제한
    LaneState max_grade = LaneState::GOOD;
    if (span_m < config_.span_min_m_good) {
        max_grade = LaneState::WEAK;
    }

    // (4) 기존 pixel + rmse 기반 등급
    LaneState base_grade;
    if (pixel_count < (size_t)config_.quality_min_pixels_weak ||
        rmse_m > config_.quality_max_rmse_m_weak) {
        base_grade = LaneState::BAD;
    } else if (pixel_count >= (size_t)config_.quality_min_pixels_good &&
               rmse_m <= config_.quality_max_rmse_m_good) {
        base_grade = LaneState::GOOD;
    } else {
        base_grade = LaneState::WEAK;
    }

    // (5) Cap 적용: base_grade가 max_grade보다 높으면 제한
    auto rank = [](LaneState s) -> int {
        switch (s) {
            case LaneState::GOOD:  return 3;
            case LaneState::WEAK:  return 2;
            case LaneState::BAD:   return 1;
            case LaneState::NODET: return 0;
        }
        return 0;
    };

    return (rank(base_grade) > rank(max_grade)) ? max_grade : base_grade;
}
