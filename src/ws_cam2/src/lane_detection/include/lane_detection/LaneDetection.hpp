#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include <fstream>
#include <string>
#include <cstdint>
#include <iomanip>
#include <sstream>

// #include "CalibData.hpp"
#include "utils/CalibData.hpp"

#define pushDebugImage(mat) pushDebugImageExec(mat, #mat)

/* 픽셀 분류 결과 */
enum class PixelSide { LEFT, RIGHT, NONE };

/* 차선 품질 등급 */
enum class LaneQuality { GOOD, WEAK, BAD };

/* 차선 최종 상태 (4-state: Validity/Sufficiency/Quality/Temporal 게이트 결과) */
enum class LaneState { NODET, BAD, WEAK, GOOD };
enum class LaneSideMask { Left, Right };
enum class LaneTrackMode { BLOB_SEEDED, PREV_SEEDED };

struct LaneGeom {
    double y0 = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double kappa_dot = 0.0;
    bool valid = false;
};

struct LaneConfig {


    /* debug images */
    int waitkey = 10;

    bool show_org = false;
    bool show_undist = false;
    bool show_bev = false;
    bool show_gamma = false;
    bool show_sobel = false;
    bool show_bev_bin = false;
    bool show_sld_win = false;
    bool show_hls = false;
    bool show_contour = false;
    bool log_detect_timing = false; // Detect() 단계별 처리 시간 로그 on/off
    bool log_process_time = false;  // DetectFromMask() 1줄 처리 시간 로그 on/off (YAML: log_process_time)
    bool log_gate_trace = false;    // validateAndSmooth 계열 게이트 추적 로그 on/off (YAML: log_gate_trace)
    bool log_sample_kf_trace = false; // trackLaneSamplesKF() sample-vector 로그 on/off (YAML: log_sample_kf_trace)
    bool log_frame_debug = false;  // 프레임 단위 상세 로그 on/off
    bool log_lr_separation = false; // DetectFromMask LR 분리 보정 로그 on/off
    
    /* debug images (yolopv2) */
    bool show_lane_mask = false;
    bool show_preprocessed = false;
    bool show_start_blob = false;
    bool show_sw = false;
    bool show_bev_pix = false;
    bool show_fitpoly = false;

    /* DetectFromMask() 전처리 */
    int mask_sobel_thresh = 30; // Sobel X 이진화 임계값 (세로 엣지 추출, 낮을수록 민감)
    int mask_close_kh = 9;     // MORPH_CLOSE 수직 커널 높이 (끊긴 엣지 연결)

    /* 지면 ROI - 차량 좌표계(지면, m) X+ 전방, Y+ 오른쪽 */
    double roi_xmin = 2.0;
    double roi_xmax = 70.0;
    double roi_ymin = -10.0;
    double roi_ymax = 10.0;

    cv::Size bev_size = cv::Size(640, 640);

    /* preprocess() */
    cv::Size ksize_vert = cv::Size(1, 7); // 세로 성분 커널 -> Open
    cv::Size ksize_noise = cv::Size(3, 3); // 노이즈 성분 커널 -> 얘도 Open
    int sobel_thres = 40; // sobel_abs 이진화 임계값
    int adaptive_block_size = 15; // adaptive threshold block size (홀수, >=3)
    double adaptive_C = -10.0; // adaptive threshold C
    double gamma = 2.0; // 1.5 ~ 2.5 사이 추천 (값이 클수록 어두운 곳이 밝아짐)

    int hls_white_hmin = 0;
    int hls_white_hmax = 180;
    int hls_white_lmin = 70;
    int hls_white_lmax = 255;
    int hls_white_smin = 0;
    int hls_white_smax = 100;

    int hls_yellow_hmin = 15;
    int hls_yellow_hmax = 35;
    int hls_yellow_lmin = 80;
    int hls_yellow_lmax = 255;
    int hls_yellow_smin = 80;
    int hls_yellow_smax = 255;

    /* findPixelsSearchAround() */
    int margin_sa = 30;
    double safe_gap_multiplier = 2.0; // SearchAround에서 safe_gap = margin_sa * multiplier (fallback)
    bool debug_pixel_overlap = false; // 픽셀 침범 디버그 로그 on/off
    bool show_sa_anchor = false; // SA(anchor) 디버그 시각화 on/off
    double sa_anchor_x_start_m = 2.0;
    double sa_anchor_x_end_m = 30.0;
    double sa_anchor_x_step_m = 2.0;
    int sa_anchor_win_w_px = 28;
    int sa_anchor_win_h_px = 30;
    int sa_anchor_strip_count = 5;
    int sa_anchor_strip_minpix = 3;
    int sa_anchor_min_valid_points = 12;
    bool sa_anchor_fallback_to_sw = true;
    bool sa_seed_use_prev_start = true;   // runSearchAroundMaskPath()
    double sa_seed_track_length_m = 8.0; // runSearchAroundMaskPath(), buildSaSeedCentersFromPrevCoeffs()
    int sa_seed_count_max = 4;           // runSearchAroundMaskPath()
    int sa_init_win_w_px = 49;           // computeSaWindowSizeForStep(), runSaFromSeeds()
    int sa_init_win_h_px = 30;           // computeSaWindowSizeForStep(), runSaFromSeeds()
    int sa_win_shrink_w_px_per_step = 1; // computeSaWindowSizeForStep()
    int sa_win_shrink_h_px_per_step = 1; // computeSaWindowSizeForStep()
    int sa_min_win_w_px = 10;            // computeSaWindowSizeForStep()
    int sa_min_win_h_px = 10;            // computeSaWindowSizeForStep()
    int lane_edge_min_run_px = 2;        // findHorizontalInnerEdgeFromCentroid()
    int lane_edge_wall_margin_px = 0;    // findHorizontalInnerEdgeFromCentroid()
    bool lane_edge_enable_run_hop = true;      // findHorizontalInnerEdgeFromCentroid()
    int lane_edge_max_hop_gap_px = 12;         // findHorizontalInnerEdgeFromCentroid()
    int lane_edge_min_hop_run_width_px = 2;    // findHorizontalInnerEdgeFromCentroid()
    int lane_edge_max_hops_per_row = 1;        // findHorizontalInnerEdgeFromCentroid()
    int prev_seed_start_prior_half_width_px = 25; // buildLaneTrackStartState()
    bool use_edge_sample_in_sw = false;  // runSwV2()

    /* findPixelsSlidingWindow() */
    int n_windows = 9;
    int sw_hist_bottom_divisor = 3; // 히스토그램 기준 영역: 하단 1/N
    int margin_sw = 35;
    int minpix = 30;
    int win_h = 30;            // 슬라이딩 윈도우 높이 (px)
    int win_w = 35;            // 슬라이딩 윈도우 폭 (px)
    double sw_forward_limit_m = 25.0; // 슬라이딩 윈도우 탐색 최대 전방 거리 (m, <=0 이면 비활성)
    int min_window_gap = 150; // px, 양쪽 윈도우 간 최소 간격

    // int expected_w = 187; // 차선 폭 3.5m으로 간주
    int expected_w = 300; // 차선 폭 3.5m으로 간주
    int tolerance = 30;     // 허용 오차 
    int max_shift = 50;

    /* strip 기반 복구 탐색(SW 대체) */
    bool sw_use_strip_guided = true;             // true면 SW 분기에서 strip 기반 탐색 사용
    int strip_count = 12;                        // BEV 세로 분할 개수
    int strip_empty_stop = 5;                    // 연속 무검출 strip 개수 초과 시 조기 종료
    int strip_peak_min_value = 2550;             // strip histogram peak 최소값
    int strip_noise_min_pixels = 300;            // 노이즈 판정 최소 픽셀 수
    double strip_noise_growth_ratio = 2.2;       // 직전 strip 대비 픽셀 급증 배율
    double strip_noise_max_occupancy_ratio = 0.38; // 히스토그램 점유 폭 비율 상한
    int strip_min_guide_points = 4;              // 2차 가이드 피팅 최소 샘플 수
    int strip_band_margin = 35;                  // 가이드 곡선 주변 밴드 반폭(px)

    // fitPolynomial()
    double lane_expected_w = 3.6; // 차선 폭(m)




    // validateAndSmooth()
    double x_check = 5.0;
    double lane_w_min = 2.5; // m
    double lane_w_max = 5.0; // m
    double min_rmse_ratio = 0.8; // RMSE 비교 시 신뢰도 비율 (작을수록 엄격)

    int max_bad_frames = 10; // 칼만 필터 믿고 버티는 프레임 수
    double valid_view_range = 5.0; // Cam2LD view_range availability 판정 임계 (m)
    double lane_sep_min_m = 1.5;   // DetectFromMask LR 최소 간격 게이트 (m)
    bool lr_slot_by_top_endpoint = true; // 상단 끝점(BEV) 기준 좌/우 슬롯 재배치
    bool lr_keep_closer_to_mid = true;   // 같은 쪽 충돌 시 중앙에 더 가까운 후보만 유지

    /* RViz lane marker (yolopv2_node) */
    bool enable_lane_marker = true;
    std::string lane_marker_frame_id = "camera_link";
    double lane_marker_forward_min_m = 0.0;
    double lane_marker_forward_max_m = 30.0;
    double lane_marker_dx_m = 0.5;
    double lane_marker_line_width_m = 0.12;
    double lane_marker_origin_radius_m = 0.25;
    double lane_marker_z_m = 0.02;

    /* 차선 품질 평가 (evaluateLaneQuality) - 미터 단위 기준 */
    int quality_min_pixels_good = 300;       // GOOD 판정 최소 픽셀 수
    int quality_min_pixels_weak = 100;       // WEAK 판정 최소 픽셀 수
    double quality_max_rmse_m_good = 0.05;   // GOOD 판정 최대 RMSE (미터)
    double quality_max_rmse_m_weak = 0.15;   // WEAK 판정 최대 RMSE (미터)

    /* 품질 기반 모드 선택 (SA/SW) */
    int weak_to_sw = 8;           // 연속 WEAK 횟수 -> SW 전환 임계값
    int bad_to_sw = 3;            // 연속 BAD 횟수 -> SW 전환 임계값
    bool debug_mode_select = false;  // 모드 선택 디버그 로그 on/off

    /* StartBlob 필터 (has_prev_start 있을 때만 적용) */
    float blob_max_angle_deg    = 15.0f; // 이전 프레임 대비 방향벡터 각도 변화 허용 최대값 (도)
    float blob_max_pos_delta_px = 20.0f; // 이전 프레임 대비 시작점 이동 허용 최대값 (px)
    bool enable_csv_log = false;     // CSV 로그 저장 on/off

    /* 4축 게이팅 (validateAndSmooth) */
    double span_min_m_weak = 5.0;       // WEAK 판정 최소 span (미터, 미만→WEAK 이하 제한)
    double span_min_m_good = 10.0;      // GOOD 판정 최소 span (미터)
    double rmse_hard_bad_m = 0.5;       // RMSE 치명 하드게이트 (미터, 초과→즉시 BAD)
    double dy_hard_m = 1.0;             // Δy 치명 하드게이트 (미터 @ x_check)
    double dkappa_hard = 0.05;          // Δκ 치명 하드게이트 (곡률 변화량)

    /* KF 측정 노이즈 (상태별) */
    double kf_R_good = 0.1;            // GOOD: measurementNoiseCov 스칼라
    double kf_R_weak = 1.0;            // WEAK: measurementNoiseCov 스칼라 (약하게 반영)

    /* MaskLite sample-vector KF */
    bool use_geometry_kf = false;
    int geom_kf_hold_frames = 5;
    double geom_kf_process_noise = 1e-4;
    double geom_kf_measurement_noise = 1e-2;
    bool use_sample_kf_masklite = true;
    int rawfit_hold_frames = 5;
    int kf_sample_hold_frames = 5;
    std::vector<double> kf_sample_x_m = {5.0, 8.0, 12.0, 16.0};
    double kf_sample_band_half_width_m = 0.5;
    int kf_sample_min_points_per_slot = 1;
    std::vector<double> kf_sample_Q_pos_list = {1e-3, 1e-3, 1e-3, 1e-3};
    std::vector<double> kf_sample_Q_vel_list = {1e-2, 1e-2, 1e-2, 1e-2};
    std::vector<double> kf_sample_R_diag_list = {0.3, 0.3, 0.8, 1.2};
    double kf_sample_invalid_R = 100.0;

    /* 전처리 옵션 (기본 OFF, YAML로 제어) */
    bool preprocess_adaptive_th = false;     // adaptive threshold on sobel_abs
    bool preprocess_morph_close = false;     // morphology close (끊긴 차선 연결)
    bool preprocess_contour_filter = false;  // contour 기반 소형 블롭 제거
    cv::Size ksize_close = cv::Size(5, 5);   // morph close 커널 크기

    /* 사다리꼴 ROI 마스크 (applyRoiMask) */
    int roi_mid_offset_px = 0;         // 중심 오프셋 (양수=오른쪽, 음수=왼쪽)
    float roi_half_top_ratio = 0.45f;  // 상단(원거리) 반폭 비율 (넓게)
    float roi_half_bot_ratio = 0.35f;  // 하단(근거리) 반폭 비율 (좁게)
    bool roi_mask_enabled = true;      // ROI 마스크 적용 여부

    /* Contour Filtering */
    double contour_min_area = 20;   // 컨투어 최소 넓이 (픽셀²)
    double contour_min_ratio = 1.5; // 컨투어 최소 종횡비 (h/w)
    int contour_merge_dist_x = 40;  // 컨투어 병합 가로 거리 (픽셀)
    int contour_merge_dist_y = 20;  // 컨투어 병합 세로 거리 (픽셀)

    /* drawROI */
    cv::Scalar color_roi = cv::Scalar(0,255,0); // 녹색
    int thickness_roi = 2;


    bool loadConfig(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);

        if (!fs.isOpened()) {
            std::cerr << "ERROR: Failed to open config file: " << filename << std::endl;
            return false;
        }


        
        // 1. 기본 타입 읽기
        if (!fs["waitkey"].empty()) fs["waitkey"] >> waitkey;
        std::cout << "[DEBUG] waitkey: "  << waitkey << std::endl;

        int temp_bool;
        if (!fs["show_org"].empty()) { fs["show_org"] >> temp_bool; show_org = (bool)temp_bool; }
        if (!fs["show_lane_mask"].empty()) { fs["show_lane_mask"] >> temp_bool; show_lane_mask = (bool)temp_bool; }
        if (!fs["show_preprocessed"].empty()) { fs["show_preprocessed"] >> temp_bool; show_preprocessed = (bool)temp_bool; }
        if (!fs["show_sw"].empty()) { fs["show_sw"] >> temp_bool; show_sw = (bool)temp_bool; }
        if (!fs["show_bev_pix"].empty()) { fs["show_bev_pix"] >> temp_bool; show_bev_pix = (bool)temp_bool; }
        if (!fs["show_fitpoly"].empty()) { fs["show_fitpoly"] >> temp_bool; show_fitpoly = (bool)temp_bool; }
        if (!fs["mask_sobel_thresh"].empty()) fs["mask_sobel_thresh"] >> mask_sobel_thresh;
        if (!fs["mask_close_kh"].empty()) fs["mask_close_kh"] >> mask_close_kh;
        if (!fs["show_undist"].empty()) { fs["show_undist"] >> temp_bool; show_undist = (bool)temp_bool; }
        if (!fs["show_bev"].empty()) { fs["show_bev"] >> temp_bool; show_bev = (bool)temp_bool; }
        if (!fs["show_gamma"].empty()) { fs["show_gamma"] >> temp_bool; show_gamma = (bool)temp_bool; }
        if (!fs["show_sobel"].empty()) { fs["show_sobel"] >> temp_bool; show_sobel = (bool)temp_bool; }
        if (!fs["show_bev_bin"].empty()) { fs["show_bev_bin"] >> temp_bool; show_bev_bin = (bool)temp_bool; }
        if (!fs["show_sld_win"].empty()) { fs["show_sld_win"] >> temp_bool; show_sld_win = (bool)temp_bool; }
        if (!fs["show_hls"].empty()) { fs["show_hls"] >> temp_bool; show_hls = (bool)temp_bool; }
        if (!fs["show_contour"].empty()) { fs["show_contour"] >> temp_bool; show_contour = (bool)temp_bool; }
        if (!fs["log_detect_timing"].empty()) {
            fs["log_detect_timing"] >> temp_bool;
            log_detect_timing = (bool)temp_bool;
            // Backward compatibility: keep legacy key behavior unless overridden by log_process_time.
            log_process_time = log_detect_timing;
        }
        if (!fs["log_process_time"].empty()) {
            fs["log_process_time"] >> temp_bool;
            log_process_time = (bool)temp_bool;
        }
        if (!fs["log_gate_trace"].empty()) {
            fs["log_gate_trace"] >> temp_bool;
            log_gate_trace = (bool)temp_bool;
        }
        if (!fs["log_sample_kf_trace"].empty()) {
            fs["log_sample_kf_trace"] >> temp_bool;
            log_sample_kf_trace = (bool)temp_bool;
        }
        if (!fs["log_lr_separation"].empty()) {
            fs["log_lr_separation"] >> temp_bool;
            log_lr_separation = (bool)temp_bool;
        }
        if (!fs["log_frame_debug"].empty()) { fs["log_frame_debug"] >> temp_bool; log_frame_debug = (bool)temp_bool; }

        // 2. Double / Int 읽기
        if (!fs["roi_xmin"].empty()) fs["roi_xmin"] >> roi_xmin;
        if (!fs["roi_xmax"].empty()) fs["roi_xmax"] >> roi_xmax;
        if (!fs["roi_ymin"].empty()) fs["roi_ymin"] >> roi_ymin;
        if (!fs["roi_ymax"].empty()) fs["roi_ymax"] >> roi_ymax;

        if (!fs["gamma"].empty()) fs["gamma"] >> gamma;

        
        // 3. OpenCV 타입 읽기 (YAML의 리스트 [w, h]를 자동으로 변환해줍니다)
        if (!fs["bev_size"].empty()) fs["bev_size"] >> bev_size;
        if (!fs["ksize_vert"].empty()) fs["ksize_vert"] >> ksize_vert;
        if (!fs["ksize_noise"].empty()) fs["ksize_noise"] >> ksize_noise;
        if (!fs["sobel_thres"].empty()) fs["sobel_thres"] >> sobel_thres;
        if (!fs["adaptive_block_size"].empty()) fs["adaptive_block_size"] >> adaptive_block_size;
        if (!fs["adaptive_C"].empty()) fs["adaptive_C"] >> adaptive_C;
        if (!fs["color_roi"].empty()) fs["color_roi"] >> color_roi;
        
        if (!fs["hls_white_hmin"].empty()) fs["hls_white_hmin"] >> hls_white_hmin;
        if (!fs["hls_white_hmax"].empty()) fs["hls_white_hmax"] >> hls_white_hmax;
        if (!fs["hls_white_lmin"].empty()) fs["hls_white_lmin"] >> hls_white_lmin;
        if (!fs["hls_white_lmax"].empty()) fs["hls_white_lmax"] >> hls_white_lmax;
        if (!fs["hls_white_smin"].empty()) fs["hls_white_smin"] >> hls_white_smin;
        if (!fs["hls_white_smax"].empty()) fs["hls_white_smax"] >> hls_white_smax;

        if (!fs["hls_yellow_hmin"].empty()) fs["hls_yellow_hmin"] >> hls_yellow_hmin;
        if (!fs["hls_yellow_hmax"].empty()) fs["hls_yellow_hmax"] >> hls_yellow_hmax;
        if (!fs["hls_yellow_lmin"].empty()) fs["hls_yellow_lmin"] >> hls_yellow_lmin;
        if (!fs["hls_yellow_lmax"].empty()) fs["hls_yellow_lmax"] >> hls_yellow_lmax;
        if (!fs["hls_yellow_smin"].empty()) fs["hls_yellow_smin"] >> hls_yellow_smin;
        if (!fs["hls_yellow_smax"].empty()) fs["hls_yellow_smax"] >> hls_yellow_smax;

        // 4. 계산이 필요한 변수 업데이트 (YAML에 없는 파생 변수들)
        if (!fs["margin_sa"].empty()) fs["margin_sa"] >> margin_sa;
        if (!fs["safe_gap_multiplier"].empty()) fs["safe_gap_multiplier"] >> safe_gap_multiplier;
        if (!fs["debug_pixel_overlap"].empty()) { fs["debug_pixel_overlap"] >> temp_bool; debug_pixel_overlap = (bool)temp_bool; }
        if (!fs["show_sa_anchor"].empty()) { fs["show_sa_anchor"] >> temp_bool; show_sa_anchor = (bool)temp_bool; }
        if (!fs["sa_anchor_x_start_m"].empty()) fs["sa_anchor_x_start_m"] >> sa_anchor_x_start_m;
        if (!fs["sa_anchor_x_end_m"].empty()) fs["sa_anchor_x_end_m"] >> sa_anchor_x_end_m;
        if (!fs["sa_anchor_x_step_m"].empty()) fs["sa_anchor_x_step_m"] >> sa_anchor_x_step_m;
        if (!fs["sa_anchor_win_w_px"].empty()) fs["sa_anchor_win_w_px"] >> sa_anchor_win_w_px;
        if (!fs["sa_anchor_win_h_px"].empty()) fs["sa_anchor_win_h_px"] >> sa_anchor_win_h_px;
        if (!fs["sa_anchor_strip_count"].empty()) fs["sa_anchor_strip_count"] >> sa_anchor_strip_count;
        if (!fs["sa_anchor_strip_minpix"].empty()) fs["sa_anchor_strip_minpix"] >> sa_anchor_strip_minpix;
        if (!fs["sa_anchor_min_valid_points"].empty()) fs["sa_anchor_min_valid_points"] >> sa_anchor_min_valid_points;
        if (!fs["sa_anchor_fallback_to_sw"].empty()) { fs["sa_anchor_fallback_to_sw"] >> temp_bool; sa_anchor_fallback_to_sw = (bool)temp_bool; }
        if (!fs["sa_seed_use_prev_start"].empty()) { fs["sa_seed_use_prev_start"] >> temp_bool; sa_seed_use_prev_start = (bool)temp_bool; }
        if (!fs["sa_seed_track_length_m"].empty()) fs["sa_seed_track_length_m"] >> sa_seed_track_length_m;
        if (!fs["sa_seed_count_max"].empty()) fs["sa_seed_count_max"] >> sa_seed_count_max;
        if (!fs["sa_init_win_w_px"].empty()) fs["sa_init_win_w_px"] >> sa_init_win_w_px;
        if (!fs["sa_init_win_h_px"].empty()) fs["sa_init_win_h_px"] >> sa_init_win_h_px;
        if (!fs["sa_win_shrink_w_px_per_step"].empty()) fs["sa_win_shrink_w_px_per_step"] >> sa_win_shrink_w_px_per_step;
        if (!fs["sa_win_shrink_h_px_per_step"].empty()) fs["sa_win_shrink_h_px_per_step"] >> sa_win_shrink_h_px_per_step;
        if (!fs["sa_min_win_w_px"].empty()) fs["sa_min_win_w_px"] >> sa_min_win_w_px;
        if (!fs["sa_min_win_h_px"].empty()) fs["sa_min_win_h_px"] >> sa_min_win_h_px;
        if (!fs["lane_edge_min_run_px"].empty()) fs["lane_edge_min_run_px"] >> lane_edge_min_run_px;
        if (!fs["lane_edge_wall_margin_px"].empty()) fs["lane_edge_wall_margin_px"] >> lane_edge_wall_margin_px;
        if (!fs["lane_edge_enable_run_hop"].empty()) { fs["lane_edge_enable_run_hop"] >> temp_bool; lane_edge_enable_run_hop = (bool)temp_bool; }
        if (!fs["lane_edge_max_hop_gap_px"].empty()) fs["lane_edge_max_hop_gap_px"] >> lane_edge_max_hop_gap_px;
        if (!fs["lane_edge_min_hop_run_width_px"].empty()) fs["lane_edge_min_hop_run_width_px"] >> lane_edge_min_hop_run_width_px;
        if (!fs["lane_edge_max_hops_per_row"].empty()) fs["lane_edge_max_hops_per_row"] >> lane_edge_max_hops_per_row;
        if (!fs["prev_seed_start_prior_half_width_px"].empty()) fs["prev_seed_start_prior_half_width_px"] >> prev_seed_start_prior_half_width_px;
        if (!fs["use_edge_sample_in_sw"].empty()) { fs["use_edge_sample_in_sw"] >> temp_bool; use_edge_sample_in_sw = (bool)temp_bool; }
        if (!fs["n_windows"].empty()) fs["n_windows"] >> n_windows;
        if (!fs["sw_hist_bottom_divisor"].empty()) fs["sw_hist_bottom_divisor"] >> sw_hist_bottom_divisor;
        if (!fs["margin_sw"].empty()) fs["margin_sw"] >> margin_sw;
        if (!fs["minpix"].empty()) fs["minpix"] >> minpix;
        if (!fs["win_h"].empty()) fs["win_h"] >> win_h;
        if (!fs["win_w"].empty()) fs["win_w"] >> win_w;
        if (!fs["sw_forward_limit_m"].empty()) fs["sw_forward_limit_m"] >> sw_forward_limit_m;
        if (!fs["min_window_gap"].empty()) fs["min_window_gap"] >> min_window_gap;
        if (!fs["expected_w"].empty()) fs["expected_w"] >> expected_w;
        if (!fs["tolerance"].empty()) fs["tolerance"] >> tolerance;
        if (!fs["max_shift"].empty()) fs["max_shift"] >> max_shift;
        if (!fs["sw_use_strip_guided"].empty()) { fs["sw_use_strip_guided"] >> temp_bool; sw_use_strip_guided = (bool)temp_bool; }
        if (!fs["strip_count"].empty()) fs["strip_count"] >> strip_count;
        if (!fs["strip_empty_stop"].empty()) fs["strip_empty_stop"] >> strip_empty_stop;
        if (!fs["strip_peak_min_value"].empty()) fs["strip_peak_min_value"] >> strip_peak_min_value;
        if (!fs["strip_noise_min_pixels"].empty()) fs["strip_noise_min_pixels"] >> strip_noise_min_pixels;
        if (!fs["strip_noise_growth_ratio"].empty()) fs["strip_noise_growth_ratio"] >> strip_noise_growth_ratio;
        if (!fs["strip_noise_max_occupancy_ratio"].empty()) fs["strip_noise_max_occupancy_ratio"] >> strip_noise_max_occupancy_ratio;
        if (!fs["strip_min_guide_points"].empty()) fs["strip_min_guide_points"] >> strip_min_guide_points;
        if (!fs["strip_band_margin"].empty()) fs["strip_band_margin"] >> strip_band_margin;

        if (!fs["lane_expected_w"].empty()) fs["lane_expected_w"] >> lane_expected_w;
        if (!fs["x_check"].empty()) fs["x_check"] >> x_check;
        if (!fs["lane_w_min"].empty()) fs["lane_w_min"] >> lane_w_min;
        if (!fs["lane_w_max"].empty()) fs["lane_w_max"] >> lane_w_max;
        if (!fs["min_rmse_ratio"].empty()) fs["min_rmse_ratio"] >> min_rmse_ratio;
        if (!fs["max_bad_frames"].empty()) fs["max_bad_frames"] >> max_bad_frames;
        if (!fs["valid_view_range"].empty()) fs["valid_view_range"] >> valid_view_range;
        if (!fs["lane_sep_min_m"].empty()) fs["lane_sep_min_m"] >> lane_sep_min_m;
        if (!fs["lr_slot_by_top_endpoint"].empty()) { fs["lr_slot_by_top_endpoint"] >> temp_bool; lr_slot_by_top_endpoint = (bool)temp_bool; }
        if (!fs["lr_keep_closer_to_mid"].empty()) { fs["lr_keep_closer_to_mid"] >> temp_bool; lr_keep_closer_to_mid = (bool)temp_bool; }
        if (!fs["enable_lane_marker"].empty()) { fs["enable_lane_marker"] >> temp_bool; enable_lane_marker = (bool)temp_bool; }
        if (!fs["lane_marker_frame_id"].empty()) fs["lane_marker_frame_id"] >> lane_marker_frame_id;
        if (!fs["lane_marker_forward_min_m"].empty()) fs["lane_marker_forward_min_m"] >> lane_marker_forward_min_m;
        if (!fs["lane_marker_forward_max_m"].empty()) fs["lane_marker_forward_max_m"] >> lane_marker_forward_max_m;
        if (!fs["lane_marker_dx_m"].empty()) fs["lane_marker_dx_m"] >> lane_marker_dx_m;
        if (!fs["lane_marker_line_width_m"].empty()) fs["lane_marker_line_width_m"] >> lane_marker_line_width_m;
        if (!fs["lane_marker_origin_radius_m"].empty()) fs["lane_marker_origin_radius_m"] >> lane_marker_origin_radius_m;
        if (!fs["lane_marker_z_m"].empty()) fs["lane_marker_z_m"] >> lane_marker_z_m;

        // 품질 평가 파라미터 (미터 단위)
        if (!fs["quality_min_pixels_good"].empty()) fs["quality_min_pixels_good"] >> quality_min_pixels_good;
        if (!fs["quality_min_pixels_weak"].empty()) fs["quality_min_pixels_weak"] >> quality_min_pixels_weak;
        if (!fs["quality_max_rmse_m_good"].empty()) fs["quality_max_rmse_m_good"] >> quality_max_rmse_m_good;
        if (!fs["quality_max_rmse_m_weak"].empty()) fs["quality_max_rmse_m_weak"] >> quality_max_rmse_m_weak;

        // 품질 기반 모드 선택 파라미터
        if (!fs["weak_to_sw"].empty()) fs["weak_to_sw"] >> weak_to_sw;
        if (!fs["bad_to_sw"].empty()) fs["bad_to_sw"] >> bad_to_sw;
        if (!fs["debug_mode_select"].empty()) { fs["debug_mode_select"] >> temp_bool; debug_mode_select = (bool)temp_bool; }
        if (!fs["blob_max_angle_deg"].empty()) fs["blob_max_angle_deg"] >> blob_max_angle_deg;
        if (!fs["blob_max_pos_delta_px"].empty()) fs["blob_max_pos_delta_px"] >> blob_max_pos_delta_px;
        if (!fs["enable_csv_log"].empty()) { fs["enable_csv_log"] >> temp_bool; enable_csv_log = (bool)temp_bool; }

        // 4축 게이팅 파라미터
        if (!fs["span_min_m_weak"].empty()) fs["span_min_m_weak"] >> span_min_m_weak;
        if (!fs["span_min_m_good"].empty()) fs["span_min_m_good"] >> span_min_m_good;
        if (!fs["rmse_hard_bad_m"].empty()) fs["rmse_hard_bad_m"] >> rmse_hard_bad_m;
        if (!fs["dy_hard_m"].empty()) fs["dy_hard_m"] >> dy_hard_m;
        if (!fs["dkappa_hard"].empty()) fs["dkappa_hard"] >> dkappa_hard;
        if (!fs["kf_R_good"].empty()) fs["kf_R_good"] >> kf_R_good;
        if (!fs["kf_R_weak"].empty()) fs["kf_R_weak"] >> kf_R_weak;
        if (!fs["use_geometry_kf"].empty()) { fs["use_geometry_kf"] >> temp_bool; use_geometry_kf = (bool)temp_bool; }
        if (!fs["geom_kf_hold_frames"].empty()) fs["geom_kf_hold_frames"] >> geom_kf_hold_frames;
        if (!fs["geom_kf_process_noise"].empty()) fs["geom_kf_process_noise"] >> geom_kf_process_noise;
        if (!fs["geom_kf_measurement_noise"].empty()) fs["geom_kf_measurement_noise"] >> geom_kf_measurement_noise;
        if (!fs["use_sample_kf_masklite"].empty()) { fs["use_sample_kf_masklite"] >> temp_bool; use_sample_kf_masklite = (bool)temp_bool; }
        if (!fs["rawfit_hold_frames"].empty()) fs["rawfit_hold_frames"] >> rawfit_hold_frames;
        if (!fs["kf_sample_hold_frames"].empty()) fs["kf_sample_hold_frames"] >> kf_sample_hold_frames;
        if (!fs["kf_sample_x_m"].empty()) fs["kf_sample_x_m"] >> kf_sample_x_m;
        if (!fs["kf_sample_band_half_width_m"].empty()) fs["kf_sample_band_half_width_m"] >> kf_sample_band_half_width_m;
        if (!fs["kf_sample_min_points_per_slot"].empty()) fs["kf_sample_min_points_per_slot"] >> kf_sample_min_points_per_slot;
        if (!fs["kf_sample_Q_pos_list"].empty()) fs["kf_sample_Q_pos_list"] >> kf_sample_Q_pos_list;
        if (!fs["kf_sample_Q_vel_list"].empty()) fs["kf_sample_Q_vel_list"] >> kf_sample_Q_vel_list;
        if (!fs["kf_sample_R_diag_list"].empty()) fs["kf_sample_R_diag_list"] >> kf_sample_R_diag_list;
        if (!fs["kf_sample_invalid_R"].empty()) fs["kf_sample_invalid_R"] >> kf_sample_invalid_R;

        // 전처리 옵션
        if (!fs["preprocess_adaptive_th"].empty()) { fs["preprocess_adaptive_th"] >> temp_bool; preprocess_adaptive_th = (bool)temp_bool; }
        if (!fs["preprocess_morph_close"].empty()) { fs["preprocess_morph_close"] >> temp_bool; preprocess_morph_close = (bool)temp_bool; }
        if (!fs["preprocess_contour_filter"].empty()) { fs["preprocess_contour_filter"] >> temp_bool; preprocess_contour_filter = (bool)temp_bool; }
        if (!fs["ksize_close"].empty()) fs["ksize_close"] >> ksize_close;

        if (!fs["contour_min_area"].empty()) fs["contour_min_area"] >> contour_min_area;
        if (!fs["contour_min_ratio"].empty()) fs["contour_min_ratio"] >> contour_min_ratio;
        if (!fs["contour_merge_dist_x"].empty()) fs["contour_merge_dist_x"] >> contour_merge_dist_x;
        if (!fs["contour_merge_dist_y"].empty()) fs["contour_merge_dist_y"] >> contour_merge_dist_y;

        // ROI 마스크 파라미터
        if (!fs["roi_mid_offset_px"].empty()) fs["roi_mid_offset_px"] >> roi_mid_offset_px;
        if (!fs["roi_half_top_ratio"].empty()) fs["roi_half_top_ratio"] >> roi_half_top_ratio;
        if (!fs["roi_half_bot_ratio"].empty()) fs["roi_half_bot_ratio"] >> roi_half_bot_ratio;
        if (!fs["roi_mask_enabled"].empty()) { fs["roi_mask_enabled"] >> temp_bool; roi_mask_enabled = (bool)temp_bool; }

        if (!fs["thickness_roi"].empty()) fs["thickness_roi"] >> thickness_roi;

        /* print 해서 파싱 검증 */


        fs.release();
        std::cout << "Config loaded successfully from " << filename << std::endl;
        
        return true;
    }

};

struct LaneResult {
    /*
     * 계수 분리 저장:
     *   - coeffs_fit_*: 피팅 직후 raw 계수 (품질 평가 기준)
     *   - coeffs_L/R:   KF 보정 후 최종 계수 (그리기/출력용)
     */
    cv::Vec4d coeffs_fit_L, coeffs_fit_R;  // 피팅 직후 raw 계수
    cv::Vec4d coeffs_L, coeffs_R;          // KF 보정 후 최종 계수

    bool detected_fit_L = false, detected_fit_R = false;  // 피팅 성공 여부

    /*
     * 검출 여부 (NODET vs 검출됨)
     *   - is_detected = true:  검증 통과, coeffs 유효, quality는 GOOD/WEAK/BAD 중 하나
     *   - is_detected = false: 미검출(NODET), coeffs=(0,0,0,0)
     */
    bool is_detected_L = false;
    bool is_detected_R = false;

    /*
     * 품질 평가 (fit 기준):
     *   - rmse_fit_m: 미터 단위 RMSE (피팅 품질 직접 반영)
     *   - quality: GOOD/WEAK/BAD 등급 (is_detected=true일 때만 의미 있음)
     */
    double rmse_fit_m_L = 0.0, rmse_fit_m_R = 0.0;  // 미터 단위 RMSE (fit 기준)
    size_t pixel_count_L = 0, pixel_count_R = 0;    // 픽셀 개수
    LaneQuality quality_L = LaneQuality::BAD;       // 좌측 차선 품질 (하위호환)
    LaneQuality quality_R = LaneQuality::BAD;       // 우측 차선 품질 (하위호환)

    /*
     * 4-state 최종 상태 (NODET/BAD/WEAK/GOOD)
     *   - (is_detected, quality) 조합 대신 단일 state로 사용
     *   - NODET: 검출 성립 실패, BAD: 검출 성립 but 치명 위반 or 소프트 미달
     */
    LaneState state_L = LaneState::NODET;
    LaneState state_R = LaneState::NODET;

    bool hard_fail_L = false, hard_fail_R = false;     // 치명 하드게이트 위반 여부
    double lane_start_m_L = 0.0, lane_start_m_R = 0.0; // 차선 시작점 x (m, near)
    double lane_end_m_L = 0.0, lane_end_m_R = 0.0;     // 차선 끝점 x (m, far)
    double span_m_L = 0.0, span_m_R = 0.0;             // 검출 커버리지 (x방향, 미터)
    double view_range_m_L = 0.0, view_range_m_R = 0.0; // Cam2LD view range (m)
    uint8_t availability_L = 0, availability_R = 0;    // Cam2LD availability (0/1)
    double delta_y_m_L = 0.0, delta_y_m_R = 0.0;       // 이전 프레임 대비 y 변화량 @ x_check
    double delta_kappa_L = 0.0, delta_kappa_R = 0.0;    // 곡률 변화량 (3차식 기반)
};

struct DebugFrame {
    cv::Mat img;
    std::string label;
};

class LaneLineDetector {
public:
    LaneLineDetector(const CalibData& calib, std::string& cfg);
    ~LaneLineDetector() = default;

    /* main pipeline */
    LaneResult Detect(const cv::Mat& frame);
    LaneResult DetectFromMask(const cv::Mat& lane_mask, const cv::Mat& frame_org = cv::Mat());
    double GetValidViewRange() const { return config_.valid_view_range; }
    bool GetEnableLaneMarker() const { return config_.enable_lane_marker; }
    const std::string& GetLaneMarkerFrameId() const { return config_.lane_marker_frame_id; }
    double GetLaneMarkerForwardMinM() const { return config_.lane_marker_forward_min_m; }
    double GetLaneMarkerForwardMaxM() const { return config_.lane_marker_forward_max_m; }
    double GetLaneMarkerDxM() const { return config_.lane_marker_dx_m; }
    double GetLaneMarkerLineWidthM() const { return config_.lane_marker_line_width_m; }
    double GetLaneMarkerOriginRadiusM() const { return config_.lane_marker_origin_radius_m; }
    double GetLaneMarkerZ() const { return config_.lane_marker_z_m; }

    /* show debug images */
    int showDebugWindow();
    cv::Mat getDebugImage();

    /* BEV(IPM) 원근변환 정보 노출 — 노드에서 BEV 뷰 시각화용 */
    cv::Mat GetBevHomography() const { return H_bev; }   // 원본(rect) → BEV homography
    cv::Size GetBevSize() const { return config_.bev_size; }

    /* 설정 노출(정보 패널용) */
    const LaneConfig& GetConfig() const { return config_; }

    // 파이프라인 중간단계(show_*) 런타임 토글 — 관리 패널의 stage 버튼용.
    //   key: mask/preprocessed/bev/gamma/sobel/bev_bin/sld_win/hls/contour
    //   getDebugImage() 모자이크에 해당 단계가 나타날지 여부를 켜고 끈다.
    bool GetShowFlag(const std::string& key) const {
        if (key == "mask")        return config_.show_lane_mask;
        if (key == "preprocessed")return config_.show_preprocessed;
        if (key == "bev")         return config_.show_bev;
        if (key == "gamma")       return config_.show_gamma;
        if (key == "sobel")       return config_.show_sobel;
        if (key == "bev_bin")     return config_.show_bev_bin;
        if (key == "sld_win")     return config_.show_sld_win;
        if (key == "hls")         return config_.show_hls;
        if (key == "contour")     return config_.show_contour;
        return false;
    }
    void SetShowFlag(const std::string& key, bool on) {
        if (key == "mask")        config_.show_lane_mask   = on;
        else if (key == "preprocessed") config_.show_preprocessed = on;
        else if (key == "bev")    config_.show_bev         = on;
        else if (key == "gamma")  config_.show_gamma       = on;
        else if (key == "sobel")  config_.show_sobel       = on;
        else if (key == "bev_bin")config_.show_bev_bin     = on;
        else if (key == "sld_win")config_.show_sld_win     = on;
        else if (key == "hls")    config_.show_hls         = on;
        else if (key == "contour")config_.show_contour     = on;
    }

public:
    cv::Mat buf_dbgimg;


private: // ld logics

    void initIPM();

    void remap(const cv::Mat& src, cv::Mat& dst); // ros2에서 왜곡보정된 이미지 받으면 제외 가능
    void initKalmanFilter();
    void initSampleKalmanFilter();
    void initGeometryKalmanFilter();

    void preprocess(cv::Mat& src, cv::Mat& dst_bev_bin);

    // 사다리꼴 ROI 마스크 적용: 중앙 영역 제외하고 좌/우 가장자리만 남김
    void applyRoiMask(cv::Mat& bev_bin);

    // findPixelsFromMask() 전용 내부 구조체
    struct StartBlob {
        cv::Point2f end_pt  = {0.0f, 0.0f};
        cv::Point2f dir_vec = {0.0f, -1.0f};
        bool valid = false;
    };
    struct WindowPixelStats {
        bool has_enough_pixels = false;
        int pixel_count = 0;
        cv::Point2f centroid = {0.0f, 0.0f};
        cv::Point2f pca_dir = {0.0f, -1.0f};
    };
    struct HorizontalEdgeSample {
        bool valid = false;
        cv::Point centroid_px = {0, 0};
        cv::Point edge_px = {0, 0};
        bool touched_window_wall = false;
        int horizontal_span_px = 0;
    };
    struct WindowTrackDebug {
        cv::Rect rect;
        cv::Point centroid_px = {0, 0};
        cv::Point edge_px = {0, 0};
        cv::Point next_center_px = {0, 0};
        bool valid = false;
    };
    struct LaneTrackStart {
        cv::Point2f center = {0.0f, 0.0f};
        cv::Point2f dir = {0.0f, -1.0f};
        bool valid = false;
    };

    void findPixels(const cv::Mat& bev_bin);
    bool findPixelsFromMask(const cv::Mat& lane_mask); // DetectFromMask() 전용, true=lane_lost
    bool runSwV2Path(const cv::Mat& lane_mask);        // SW v2 전체 경로: 시작점탐색+시각화+SW수집
    bool runSearchAroundMaskPath(const cv::Mat& lane_mask); // SA(mask) 경로: 예측 중심 기반 픽셀 수집
    std::vector<StartBlob> findStartBlobsInRoi(const cv::Mat& lane_mask, const cv::Rect& roi_rect) const;
    // ROI 탐색 + L/R 후보 선택 + lane_lost 처리: true=lane_lost
    bool selectStartBlobs(const cv::Mat& lane_mask, StartBlob& out_L, StartBlob& out_R);
    bool selectStartBlobsForTracking(const cv::Mat& lane_mask,
                                     StartBlob& out_L, StartBlob& out_R,
                                     bool commit_state);
    void runSwV2(const StartBlob& blob, const cv::Mat& lane_mask, std::vector<cv::Point>& out,
                 int stop_line_y = -1, std::vector<cv::Rect>* debug_wins = nullptr);
    int computeSwForwardLimitY(const cv::Size& mask_size, const cv::Size& rect_size) const;
    void updateSwForwardLimitCache(const cv::Size& mask_size, const cv::Size& rect_size);
    void sampleAnchorPointsVehicle(const cv::Vec4d& coeffs, std::vector<cv::Point2d>& out_veh) const;
    void projectVehicleAnchorsToMask(const std::vector<cv::Point2d>& in_veh,
                                     const cv::Mat& H_bev2mask,
                                     const cv::Size& mask_size,
                                     std::vector<cv::Point2f>& out_mask) const;
    void collectCentroidsFromAnchorWindows(const cv::Mat& lane_mask,
                                           const std::vector<cv::Point2f>& anchors,
                                           std::vector<cv::Point>& out_pts,
                                           std::vector<cv::Rect>* debug_wins = nullptr) const;
    void buildMaskPredictionLutFromCoeffs(const cv::Vec4d& coeffs,
                                          const cv::Mat& H_bev2mask,
                                          int mask_h, int mask_w,
                                          std::vector<int>& out_u) const;
    WindowPixelStats computeWindowPixelStats(const cv::Mat& lane_mask,
                                             const cv::Rect& win_rect,
                                             int min_required_pixels) const;
    HorizontalEdgeSample findHorizontalInnerEdgeFromCentroid(const cv::Mat& lane_mask,
                                                             const cv::Rect& win_rect,
                                                             const WindowPixelStats& stats,
                                                             LaneSideMask side) const;
    bool processTrackingWindow(const cv::Mat& lane_mask,
                               LaneSideMask side,
                               const cv::Rect& win_rect,
                               int min_required_pixels,
                               WindowPixelStats& out_stats,
                               HorizontalEdgeSample& out_sample) const;
    cv::Size computeSaWindowSizeForStep(int step_idx) const;
    cv::Point2f computeNextWindowBottomCenterForSA(const cv::Rect& cur_rect,
                                                   const HorizontalEdgeSample& sample,
                                                   int next_win_h) const;
    bool projectSaNearStartFromCoeff(const cv::Vec4d& coeffs,
                                     const cv::Mat& H_bev2mask,
                                     const cv::Size& mask_size,
                                     cv::Point2f& out_center,
                                     cv::Point2f& out_dir) const;
    bool buildSaStartState(const cv::Mat& lane_mask,
                           LaneSideMask side,
                           const cv::Mat& H_bev2mask,
                           cv::Point2f& out_center,
                           cv::Point2f& out_dir) const;
    LaneTrackMode decideLaneTrackMode(LaneSideMask side) const;
    bool buildLaneTrackStartState(const cv::Mat& lane_mask,
                                  LaneSideMask side,
                                  LaneTrackMode mode,
                                  const cv::Mat& H_bev2mask,
                                  const StartBlob& blob_L,
                                  const StartBlob& blob_R,
                                  LaneTrackStart& out_start) const;
    void runLaneTrackContinuous(const cv::Mat& lane_mask,
                                LaneSideMask side,
                                LaneTrackMode mode,
                                const LaneTrackStart& start,
                                std::vector<cv::Point>& out_samples,
                                int stop_line_y = -1,
                                std::vector<WindowTrackDebug>* debug_steps = nullptr,
                                LaneTrackStart* out_first_valid = nullptr) const;
    void buildSaSeedCentersFromPrevCoeffs(const cv::Vec4d& coeffs,
                                          const cv::Mat& H_bev2mask,
                                          const cv::Size& mask_size,
                                          std::vector<cv::Point2f>& out_mask_pts) const;
    void runSaTrackContinuous(const cv::Mat& lane_mask,
                              LaneSideMask side,
                              const cv::Point2f& start_center,
                              const cv::Point2f& start_dir,
                              std::vector<cv::Point>& out_samples,
                              int stop_line_y = -1,
                              std::vector<WindowTrackDebug>* debug_steps = nullptr) const;
    void runSaFromSeeds(const cv::Mat& lane_mask,
                        LaneSideMask side,
                        const std::vector<cv::Point2f>& seed_centers,
                        std::vector<cv::Point>& out_samples,
                        int stop_line_y = -1,
                        std::vector<WindowTrackDebug>* debug_steps = nullptr) const;
    void drawTrackingWindowDebug(cv::Mat& vis,
                                 const std::vector<WindowTrackDebug>& steps,
                                 LaneSideMask side,
                                 LaneTrackMode mode) const;
    void runSearchAround(const cv::Mat& bev_bin, const std::vector<cv::Point>& nonzero);
    void runStripHistogramGuidedSearch(const cv::Mat& bev_bin, const std::vector<cv::Point>& nonzero);
    void runSlidingWindow(const cv::Mat& bev_bin, const std::vector<cv::Point>& nonzero);
    void computeSlidingWindowBase(const cv::Mat& bev_bin,
                                  int& base_xl, int& base_xr,
                                  double& peak_l_val, double& peak_r_val);

    void fitPolynomial();

    void validateAndSmooth();
    void validateAndSmoothMaskLite();
    void trackLaneSamplesKF();
    void trackLaneGeometryKF();
    void resultsFromRawFit();
    void normalizeMaskLaneSlotsByTopEndpoint(std::vector<cv::Point>& pts_L, std::vector<cv::Point>& pts_R);
    void applyMaskLiteSeparationGate(bool& hard_fail_L, bool& hard_fail_R,
                                     std::string& kf_action_L, std::string& kf_action_R);

    // void drawResults(); // TODO: debug 함수 모듈 불러온 곳에서 호출할 수 있게, 이미지 개수만큼 adaptive하게 창 띄우는 구조로
    // void pushDebugImage(const cv::Mat& img);

    void drawROI(cv::Mat& img);

    cv::Vec4d calcLaneCoeffs(const std::vector<cv::Point>& pixels);
    cv::Vec4d calcLaneCoeffs_org(const std::vector<cv::Point>& pixels);

    void drawResults(cv::Mat& undist_img);

    void drawPolyBev(cv::Mat& img, const cv::Scalar& color,
                     const cv::Vec4d& cL, const cv::Vec4d& cR,
                     bool det_L, bool det_R);
    
    void pushDebugImageExec(const cv::Mat& img, std::string label);

    // DetectFromMask 경로: mask 픽셀 -> BEV 픽셀 homography 구축/적용
    void buildMaskToBevHomography(const cv::Size& mask_size, const cv::Size& rect_size);
    void transformMaskCentroidsToBev(std::vector<cv::Point>& pts) const;

    // BEV 픽셀 <-> 차량 좌표계(X+: 전방, Y+: 좌측, 원점: BEV 하단 중앙)
    cv::Point2d bevPixToVehicle(const cv::Point2d& p_bev) const;
    cv::Point2d vehicleToBevPix(const cv::Point2d& p_veh) const;
    std::vector<cv::Point2f> bevToVehicleLocal(const std::vector<cv::Point>& bev_pts) const;
    void pushBevVehicleDebugImage(const std::vector<cv::Point>& bev_L, const std::vector<cv::Point>& bev_R);

    /* 픽셀 분리 유틸 함수 */
    // uL, uR 사이 간격이 min_gap_px 미만이면 중앙 기준 강제 벌림
    std::pair<double, double> enforceMinGap(double uL, double uR, double min_gap_px);

    // nx 픽셀을 uL, uR 중 더 가까운 쪽에 단일 귀속 (margin_px 이내만)
    PixelSide classifyPixelToSide(double nx, double uL, double uR, double margin_px);

    // 차선 품질 평가 (픽셀 수 + RMSE_m 기반, rmse는 미터 단위) - 하위호환용 유지
    LaneQuality evaluateLaneQuality(size_t pixel_count, double rmse_m);

    // 4축 게이팅 결과 종합 → 최종 LaneState 결정
    LaneState determineLaneState(bool validity_pass, bool hard_fail,
                                 size_t pixel_count, double rmse_m, double span_m);

    std::string stateToString(LaneState s) const;
    std::string makeCsvName() const;

    // 픽셀 span 계산 (x방향 미터)
    double calcSpanM(const std::vector<cv::Point>& pixels) const;
    bool calcLaneRangeM(const std::vector<cv::Point>& pixels,
                        double& start_m, double& end_m, double& span_m) const;

    // 곡률(curvature) 계산 @ x (3차식 기반)
    double calcCurvatureAt(const cv::Vec4d& coeffs, double x) const;
    bool buildLaneSampleMeasurementFromPoints(const std::vector<cv::Point2f>& pts_vehicle,
                                              cv::Mat& z_out,
                                              std::vector<uint8_t>& valid_mask) const;
    bool buildLaneSampleMeasurementFromAccumulator(const std::vector<std::vector<float>>& slot_y_accum,
                                                   cv::Mat& z_out,
                                                   std::vector<uint8_t>& valid_mask) const;
    bool fillMissingLaneSampleSlots(cv::Mat& z_io,
                                    const std::vector<uint8_t>& valid_mask) const;
    cv::Vec4d fitPolynomialFromSamples(const cv::Mat& state_or_meas) const;
    LaneGeom fitLaneGeometry(const std::vector<cv::Point2d>& pts) const;
    cv::Vec4d geometryToPolynomial(const LaneGeom& g) const;
    void setSampleMeasurementNoiseWithValidity(cv::KalmanFilter& kf,
                                               const std::vector<uint8_t>& valid_mask) const;
    void resetSampleSlotAccumulator(LaneSideMask side) const;
    void resetAllSampleSlotAccumulators() const;
    void accumulateSampleSlotFromWindow(LaneSideMask side,
                                        const cv::Rect& win_rect,
                                        const cv::Point& sample_pt) const;
    void drawLaneSampleKfDebug(const cv::Mat& filtered_L, bool filtered_valid_L,
                               const cv::Mat& filtered_R, bool filtered_valid_R);
    void drawLaneGeometryKfDebug(const cv::Vec4d& raw_L, bool raw_valid_L,
                                 const cv::Vec4d& raw_R, bool raw_valid_R,
                                 const cv::Vec4d& kf_L, bool kf_valid_L,
                                 const cv::Vec4d& kf_R, bool kf_valid_R);
    int sampleCount() const { return static_cast<int>(config_.kf_sample_x_m.size()); }
    bool hasValidSampleKfNoiseConfig() const;
    void warnSampleKfNoiseConfigFallback(const char* context);


private: // member variants
    CalibData calib_;    
    LaneConfig config_;
    LaneResult cur_result_;
    cv::Vec4d last_coeffs_L_ = cv::Vec4d(0, 0, 0, 0);
    cv::Vec4d last_coeffs_R_ = cv::Vec4d(0, 0, 0, 0);

    /* 이미지 버퍼 */ //TODO -> 그냥 했던 거랑 버퍼 관리해서 한 거랑 fps 비교해보자.
    cv::Mat undist;
    cv::Mat bev_bin;
    cv::Mat bev;
    cv::Mat bev_gamma;
    cv::Mat gray;
    cv::Mat vis;
    cv::Mat half, hist;
    cv::Mat sld_win;


    /* initIPM() */
    double roi_x_dist, roi_y_dist; // 지면 상의 roi 전방/좌우 길이
    // double bev_mpp_x, bev_mpp_y;   // bev meters per pixel x/y
    double bev_mpp_x_, bev_mpp_y_;   // bev meters per pixel x/y
    std::vector<cv::Point2f> roi_pixel_pts_; // initIPM() 에서 계산된 ROI 픽셀 좌표

    std::vector<cv::Point> pixels_L_, pixels_R_; // detected lane pixels

    // std::deque<cv::Vec4d> prev_L_, prev_R_;
    bool is_tracking_ = false;

    /* 상태 기반 모드 선택 (SA/SW) */
    LaneState prev_state_L_ = LaneState::NODET;  // 직전 프레임 상태
    LaneState prev_state_R_ = LaneState::NODET;
    bool prev_real_L_ = false;   // 직전 프레임에서 검출 성립 여부
    bool prev_real_R_ = false;
    int weak_streak_L_ = 0;      // 연속 WEAK 횟수
    int weak_streak_R_ = 0;
    int bad_streak_L_ = 0;       // 연속 BAD 횟수
    int bad_streak_R_ = 0;
    
    cv::Mat H_bev;     // org -> bev
    cv::Mat H_bev_inv; // bev -> org
    cv::Mat H_mask2bev_; // mask(px) -> bev(px), DetectFromMask 경로 전용
    cv::Size mask_ref_size_;
    cv::Size rect_ref_size_;
    int sw_forward_limit_y_cache_ = -1;
    bool sw_forward_limit_cache_valid_ = false;
    cv::Size sw_forward_limit_mask_size_cache_;
    cv::Size sw_forward_limit_rect_size_cache_;
    double sw_forward_limit_m_cache_ = 0.0;
    
    /* preprocess() */
    cv::Mat kernel_vert = cv::getStructuringElement(cv::MORPH_RECT, config_.ksize_vert);
    cv::Mat kernel_noise = cv::getStructuringElement(cv::MORPH_RECT, config_.ksize_noise);
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, config_.ksize_close);

    /* findPixelsSlidingWindow() */
    int momentum_l = 0;
    int momentum_r = 0;
    int min_window_gap;
    
    /* showDebugWindow() */
    std::vector<DebugFrame> debug_; // debug용 이미지 리스트
    bool is_paused_ = false;

    cv::Mat display_window_;
    cv::Mat buff_bgr_;

    /* validateAndSmooth() */
    cv::KalmanFilter kf_L_;
    cv::KalmanFilter kf_R_;
    bool kf_initialized_L_ = false;
    bool kf_initialized_R_ = false;
    cv::KalmanFilter kf_geom_L_;
    cv::KalmanFilter kf_geom_R_;
    bool kf_geom_initialized_L_ = false;
    bool kf_geom_initialized_R_ = false;
    cv::KalmanFilter kf_L_samples_;
    cv::KalmanFilter kf_R_samples_;
    bool kf_samples_initialized_L_ = false;
    bool kf_samples_initialized_R_ = false;
    
    cv::Mat meas_L_, meas_R_;     // 측정값(detection 결과)을 담을 행렬
    cv::Mat pred_L_, pred_R_; // 예측 결과 저장
    cv::Mat estimated_L_, estimated_R_; // 보정 결과 저장
    cv::Mat meas_samples_L_, meas_samples_R_;
    cv::Mat pred_samples_L_, pred_samples_R_;
    cv::Mat estimated_samples_L_, estimated_samples_R_;
    mutable std::vector<std::vector<float>> sample_slot_y_accum_L_;
    mutable std::vector<std::vector<float>> sample_slot_y_accum_R_;

    double lane_w_min, lane_w_max;

    int bad_frame_count_ = 0;
    int max_bad_frames_; // 칼만 필터 믿고 버티는 프레임 수
    int mask_lite_miss_L_ = 0;
    int mask_lite_miss_R_ = 0;
    int geom_miss_L_ = 0;
    int geom_miss_R_ = 0;
    bool geom_hold_valid_L_ = false;
    bool geom_hold_valid_R_ = false;
    double geom_hold_lane_start_m_L_ = 0.0;
    double geom_hold_lane_end_m_L_ = 0.0;
    double geom_hold_span_m_L_ = 0.0;
    double geom_hold_view_range_m_L_ = 0.0;
    double geom_hold_lane_start_m_R_ = 0.0;
    double geom_hold_lane_end_m_R_ = 0.0;
    double geom_hold_span_m_R_ = 0.0;
    double geom_hold_view_range_m_R_ = 0.0;
    int sample_miss_L_ = 0;
    int sample_miss_R_ = 0;
    bool sample_kf_noise_config_warned_ = false;
    cv::Vec4d raw_hold_coeffs_L_ = cv::Vec4d(0, 0, 0, 0);
    cv::Vec4d raw_hold_coeffs_R_ = cv::Vec4d(0, 0, 0, 0);
    bool raw_hold_valid_L_ = false;
    bool raw_hold_valid_R_ = false;
    int raw_hold_miss_L_ = 0;
    int raw_hold_miss_R_ = 0;
    double raw_hold_lane_start_m_L_ = 0.0;
    double raw_hold_lane_end_m_L_ = 0.0;
    double raw_hold_span_m_L_ = 0.0;
    double raw_hold_view_range_m_L_ = 0.0;
    double raw_hold_lane_start_m_R_ = 0.0;
    double raw_hold_lane_end_m_R_ = 0.0;
    double raw_hold_span_m_R_ = 0.0;
    double raw_hold_view_range_m_R_ = 0.0;

    /* drawResults() */
    std::vector<cv::Point2f> pts_bev_L, pts_bev_R;
    std::vector<cv::Point2f> pts_orig_L, pts_orig_R;

    bool csv_initialized_ = false;
    std::ofstream csv_file_;
    uint64_t frame_idx_ = 0;

    /* findPixelsFromMask()용 이전 프레임 시작점 / 방향 (perspective 픽셀) */
    cv::Point2f prev_start_L_ = {0.0f, 0.0f};
    cv::Point2f prev_start_R_ = {0.0f, 0.0f};
    bool has_prev_start_L_    = false;
    bool has_prev_start_R_    = false;
    cv::Point2f prev_dir_L_;  // 이전 프레임 L blob 방향벡터 (has_prev_start_L_=true일 때만 유효)
    cv::Point2f prev_dir_R_;  // 이전 프레임 R blob 방향벡터 (has_prev_start_R_=true일 때만 유효)



};
