#ifndef LANE_DETECTION_HPP
#define LANE_DETECTION_HPP

/* basics */
#include <iostream>
#include <vector>
#include <utility>
#include <tuple>
#include <vector>
#include <random>
#include <numeric>
#include <cmath>

/* args 파싱 */
#include <getopt.h>


/* opencv */
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgcodecs.hpp>

/*  */
#include "fo_cam2/camera_geometry.hpp"

/* 구조체 정의 */
struct LaneDetectionResult {
    cv::Mat processed_frame;           // 차선이 그려진 프레임
    std::vector<double> coeffs_l;      // 왼쪽 차선 4차 다항식 계수 (a,b,c,d)
    std::vector<double> coeffs_r;      // 오른쪽 차선 4차 다항식 계수 (a,b,c,d)
    bool is_detection_successful = false; // 검출 성공 여부 플래그
};

////// 
struct SlidingWindowConfig {
    int nwindows = 10;
    int margin = 20;
    int minpix_weak = 5;
    int minpix_strong = 15;
    int minpix_for_fitting = 8;         // 국소 피팅용
    double R2_threshold = 0.6;          // 피팅 신뢰도
    double momentum = 0.7;              // 예측 가중치
    int max_lateral_jump = 15;          // 변화 제한
    int consecutive_fail_threshold = 3; // 단절 판단
};

struct WindowState {
    int currentX;
    double prevSlope = 0.0;           // 이전 기울기
    int consecutive_fails = 0;        // 연속 실패
    bool is_broken = false;
    int break_y_position = -1;
    
    WindowState(int startX) : currentX(startX) {}
};

struct LocalFitResult {
    bool success = false;
    double slope = 0.0;
    double intercept = 0.0;
    double R2 = 0.0;
};

struct LaneBreakInfo {
    bool is_broken = false;
    int valid_end_y = -1;
};

struct WindowStateExtended {
    int currentX;
    double prevSlope = 0.0;
    int consecutive_fails = 0;
    bool is_broken = false;
    int break_y_position = -1;
    
    std::vector<LocalFitResult> local_fits;  // 🆕 각 윈도우별 피팅 결과
    
    WindowStateExtended(int startX) : currentX(startX) {}
};



class LaneDetector
{
public:
    cv::VideoCapture cap_;
    bool flag_debug_enable = false;
    bool flag_imshow_enable = false;
    bool flag_log_enable = false;

    LaneDetector(const CamCalib& c);
    void run();
    cv::Mat create_dbg_img();
    std::pair<cv::Vec4d, cv::Vec4d> outCam2LD(cv::Mat &frame);
    
private:
    void pre_process();
    void detect();
    std::pair<cv::Vec4d, cv::Vec4d> polyFit();
    int visualization();

    void setup_transform();
    // void setup_transform_org();
    
    cv::Mat makeCleanLaneMask_SAFE(const cv::Mat &bin_bgr);
    std::vector<cv::Point> slidingWindow(
                            const cv::Mat &binary_warped, 
                            int baseX, 
                            int nWindows = 15, 
                            int margin = 30, 
                            int minpix = 25
                        );
    // std::pair<std::vector<cv::Point>, WindowState> slidingWindow2(const cv::Mat& binary,
    //                                                                     int baseX,
    //                                                                     const SlidingWindowConfig& cfg
    //                                                                 );
    std::pair<std::vector<cv::Point>, WindowStateExtended> slidingWindow2(const cv::Mat& binary,
                                                                        int baseX,
                                                                        const SlidingWindowConfig& cfg);
    LocalFitResult fitLocalLine(const std::vector<cv::Point>& points,
                                int offset_x, 
                                int offset_y
                            );
    
    int computeMeanX(
        const std::vector<cv::Point>& points,
        int offset_x);
    bool fitLaneRANSAC(const std::vector<cv::Point>& lanePoints, cv::Vec4d& out_coeffs,
                    int iterations, double distance_threshold, double min_inlier_ratio);
    bool fitLaneRANSAC_gnd(const std::vector<cv::Point2f>& lanePoints, cv::Vec4d& out_coeffs, // y=f(x)
                    int iterations, double distance_threshold, double min_inlier_ratio);


    /* visualization */
    cv::Mat visualize_lane_lines_gnd(const cv::Mat& bev_image, const cv::Vec4d& left_coeffs, const cv::Vec4d& right_coeffs);
    cv::Point2d bevPixelToGround(const cv::Point2d& pixel_uv) const;

    cv::Mat visualize_lane_lines(const cv::Mat& bev_image, const cv::Vec4d& left_coeffs, const cv::Vec4d& right_coeffs);
    std::pair<int,int> findLaneBasePositionsGated(
                            const cv::Mat &binary, 
                            double meters_per_px_x, double meters_per_px_y,
                            int    prev_leftX  ,
                            int    prev_rightX ,
                            double lane_width_m,
                            double band_m      ,
                            double bottom_ratio,
                            int    smooth_ksize,
                            int    peak_min_sum,
                            double prev_band_m );
    // std::pair<int,int> trackBasePositions();

    cv::Mat draw_lane_on_original(const cv::Mat& frame_original, 
                               const cv::Vec4d& coeffs_L, 
                               const cv::Vec4d& coeffs_R,
                               bool fill_area = true);

    cv::Mat draw_lane_points_on_original(const cv::Mat& frame_original,
                                                     const std::vector<cv::Point>& lanePoints_L,
                                                     const std::vector<cv::Point>& lanePoints_R);

    
    cv::Mat visualize_sliding_windows(const cv::Mat& bev_image,
                                   const std::vector<cv::Point>& lanePoints_L,
                                   const std::vector<cv::Point>& lanePoints_R,
                                   int baseX_L, int baseX_R,
                                   int nwindows = 9, int margin = 10);

    cv::Mat visualize_sliding_windows_binary(const cv::Mat& binary_image,
                                            const std::vector<cv::Point>& lanePoints_L,
                                            const std::vector<cv::Point>& lanePoints_R,
                                            int baseX_L, 
                                            int baseX_R,
                                            const SlidingWindowConfig& cfg,
                                            const std::vector<LocalFitResult>& local_fits_L,  // 🆕 국소 피팅 결과들
                                            const std::vector<LocalFitResult>& local_fits_R
                                        );
    cv::Mat visualize_final_lanes_color(const cv::Mat& color_bev,
                                        const std::vector<cv::Point>& lanePoints_L,
                                        const std::vector<cv::Point>& lanePoints_R,
                                        const cv::Vec4d& coeffs_L,
                                        const cv::Vec4d& coeffs_R,
                                        bool success_L,
                                        bool success_R
                                    );

                                   
                                   
    /* variables */
    LaneDetectionResult result;
    
    cv::Mat frame_org;
    cv::Mat frame_undist;
    cv::Mat frame_bev;
    cv::Mat frame_res;
    cv::Mat bev_gray;
    // cv::Mat bev_bin;
    cv::Mat bev_bin_sobel;
    cv::Mat bev_bin_ridge;

    int w_org;
    int h_org;
    

    // const double gnd_x1_cm = 2500.0; // cm, 전방 최대 범위
    // const double gnd_x2_cm = 500.0;  // cm, 전방 최소 범위
    // const double gnd_y1_cm = 300.0;  // cm, 좌측 범위
    // const double gnd_y2_cm = -300.0; // cm, 우측 범위
    const double gnd_x1_m = 43.0;    // m, 전방 최대 범위
    const double gnd_x2_m = 3.0;     // m, 전방 최소 범위
    const double gnd_y1_m = 5.5;     // m, 좌측 범위
    const double gnd_y2_m = -5.5;    // m, 우측 범위
    // const double METERS_PER_PIXEL_X = 0.05;
    // const double METERS_PER_PIXEL_Y = 0.025;
    // const int w_bev = (gnd_y1_m - gnd_y2_m) / METERS_PER_PIXEL_Y;
    // const int h_bev = (gnd_x1_m - gnd_x2_m) / METERS_PER_PIXEL_X;

    /* bev 사이즈 정해놓을 때? */
    const int w_bev = 320; //px, 1280/4
    const int h_bev = 720;
    const double METERS_PER_PIXEL_X = (gnd_x1_m - gnd_x2_m) / h_bev; // 0.056
    const double METERS_PER_PIXEL_Y = (gnd_y1_m - gnd_y2_m) / w_bev; // 0.034
    
    const int NUM_POINTS = 4;                // 점의 개수를 상수로 정의
    // std::vector<cv::Point2d> image_rect_pts; // BEV 사다리꼴 원본 이미지 위치
    std::vector<cv::Point2f> image_rect_pts; // BEV 사다리꼴 원본 이미지 위치

    CamCalib calib;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat K_;
    cv::Mat D_;
    cv::Mat R_w2c;
    cv::Mat t_w2c;
    cv::Mat R_c2w;
    cv::Mat t_c2w;
    cv::Mat R_refined;
    cv::Mat t_refined;

    cv::Mat cam2bev;
    cv::Mat bev2cam;

    // cv::Mat extrinsic_R;
    // cv::Mat extrinsic_t;
    cv::Mat gnd2cam_4x4_ = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat cam2gnd_4x4_ = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat mapx_, mapy_; // 왜곡 보정용 룩업 테이블 (LUT)
    cv::Mat H_gnd2cam;    // 정방향 BEV 변환 행렬 H (gnd -> cam)
    cv::Mat H_cam2gnd;    // 역방향 BEV 변환 행렬 H^-1 (cam -> gnd)

    /*  */
    std::pair<int,int> baseX_LR;
    
    /* sliding window */
    int nWindows = 15;
    int margin = 30;
    int minPixels = 25;
    std::vector<cv::Point> lanePoints_L;
    std::vector<cv::Point> lanePoints_R;
    std::vector<cv::Point2f> lanePointsGnd_L, lanePointsGnd_R;
    // AdaptiveWindow sw_cfg;
    SlidingWindowConfig sw_cfg_;
    LaneBreakInfo break_info_L;
    LaneBreakInfo break_info_R;
    std::vector<LocalFitResult> local_fits_L_;
    std::vector<LocalFitResult> local_fits_R_;
    /* hough */
    std::vector<cv::Vec4i> laneLines;

    /* polyfit : RANSAC */
    int ransac_iterations = 100;      // 반복 횟수
    double ransac_threshold = 5.0;    // Inlier 임계값 (단위: 픽셀)
    int ransac_min_inliers = 50;      // 최소 Inlier 수

    cv::Vec4d coeffs_L;
    cv::Vec4d coeffs_R;
    bool success_L;
    bool success_R;

    /* buffers, kernels, y-caches */
    cv::Mat tmp_gray, tmp_blur, gx, gy, mag, mag_mask, dir_ratio, dir_mask_u8;
    cv::Mat bin_dir, horiz, no_horiz, clean, tmp8, overlay;
    
    cv::Mat k_hor, k_ver;

    std::vector<double> y_, y2_, y3_;
    bool kernels_ready = false;

    double kThetaDeg      = 40.0;   // 방향 게이트
    double kMinMag        = 25.0;   // SobelX 임계
    double kMinBright     = 140.0;  // 밝기 하한
    double kHorizLen_m    = 0.45;   // 가로 스트라이프 제거 커널 길이
    double kVertClose_m   = 0.45;   // 세로 close 길이
    bool   kUseCenterMask = false;  // 중앙 배타 임시 비활성

    /* for tracking */
    cv::Vec4d prev_coeff_L, prev_coeff_R;
    bool prev_ok_L = false, prev_ok_R = false;
    std::vector<cv::Point> left_lane_points;
    std::vector<cv::Point> right_lane_points;
    int prev_lX = -1, prev_rX = -1;

    // state_vec sv;
    // sv.y0 = 0.2;
    // sv.th5 = 0.00; // rad
    // sv.th15 = 0.00; // rad
    // sv.th25 = 0.00; // rad
    // sv.width = 3.5;

    // cv::KalmanFilter kfL, kfR; // [y0, theta5, theta15, theta25, w]
    // cv::Mat xL, zL, xR, zR;
    // bool kfL_init = false, kfR_init=false;

    // void initKF5(cv::KalmanFilter& kf, cv::Mat& x, cv::Mat& z, 
    //              double q=1e-3, double r=1e-2);
    // static cv::Vec4d cubicCenterFromState(const cv::Mat& x);
    // static cv::Mat   measFromLR(const cv::Vec4d& kL, const cv::Vec4d& kR); // [y0, theta5, theta15, theta25, w]
    // static cv::Vec4d offsetFitCubic(const cv::Vec4d& kC, double W, bool left,
    //                                 double xmin=0.0, double xmax=35.0, double xstep=0.25);


};

#endif // LANE_DETECTION_HPP