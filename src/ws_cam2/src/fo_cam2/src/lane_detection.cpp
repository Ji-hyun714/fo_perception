#include "fo_cam2/lane_detection.hpp"
// #include "fo_cam2/lane_tracking_KF.hpp"

// LaneDetector::LaneDetector()
LaneDetector::LaneDetector(const CamCalib& c)

    // : cap_(pipeline_str, cv::CAP_GSTREAMER)
{
    w_org = 1280;
    h_org = 720;

    gnd2cam_4x4_ = cv::Mat::eye(4, 4, CV_64F);
    cam2gnd_4x4_ = cv::Mat::eye(4, 4, CV_64F);
    
    // setup_transform_org();
    calib = c;
    setup_transform();
}

// std::pair<cv::Vec4d, cv::Vec4d> LaneDetector::outCam2LD(cv::Mat &frame)
std::pair<cv::Vec4d, cv::Vec4d> LaneDetector::outCam2LD(cv::Mat &frame)
{
    frame_org = frame;

    pre_process();
    detect();

    return polyFit();
}

cv::Mat LaneDetector::create_dbg_img()
{
    
    /* 원본 프레임 */
    // std::string window_name_org = " Original frame Size: " + std::to_string(w_org) + " x " + std::to_string(h_org);
    // cv::imshow(window_name_org, frame_org);
    
    cv::Mat result_with_lane = draw_lane_on_original(frame_org, coeffs_L, coeffs_R, true);
    result_with_lane = draw_lane_points_on_original(result_with_lane, lanePoints_L, lanePoints_R);

    
    /* 왜곡보정, BEV 범위 표시 */
    frame_undist = frame_org.clone();
    std::vector<cv::Point> points_int(image_rect_pts.begin(), image_rect_pts.end());
    const cv::Point *ppt[1] = {points_int.data()};
    int npt[] = {(int)points_int.size()};
    cv::polylines(result_with_lane, ppt, npt, 1, true, cv::Scalar(0, 255, 0), 2);
    // draw_guidelines_on_image(frame_undist);
    // std::string window_name_undist = "Undistorted frame";
    // cv::imshow(window_name_undist, frame_undist);
    cv::imshow("Original with Lane Overlay", result_with_lane);

    // /* BEV */
    // std::string window_name_bev = " BEV frame Size: " + std::to_string(w_bev) + " x " + std::to_string(h_bev);
    // // cv::imshow(window_name_bev, frame_bev);

    // // 라인 시작점 표시
    // cv::circle(frame_bev, cv::Point(baseX_LR.first, h_bev - 1), 10, cv::Scalar(0, 0, 255), -1);
    // cv::circle(frame_bev, cv::Point(baseX_LR.second, h_bev - 1), 10, cv::Scalar(0, 0, 255), -1); 
    
    // // 슬라이딩 윈도우로 얻은 차선 픽셀 표시 
    // cv::Mat laneMask(frame_bev.size(), CV_8U, cv::Scalar(0));
    // for (const auto& pl:lanePoints_L) {
    //     laneMask.at<uchar>(pl) = 255;
    // }
    // for (const auto& pr:lanePoints_R) {
    //     laneMask.at<uchar>(pr) = 255;
    // }
    // frame_bev.setTo(cv::Scalar(0,255,255), laneMask); 

    // // frame_bev = visualize_lane_lines(frame_bev, coeffs_L, coeffs_R);
    // frame_bev = visualize_lane_lines_gnd(frame_bev, coeffs_L, coeffs_R);
    // frame_bev = visualize_sliding_windows(frame_bev, lanePoints_L, lanePoints_R,
    //                                   baseX_LR.first, baseX_LR.second,
    //                                   nWindows, margin);
    // cv::imshow("BEV", frame_bev);
    // cv::imshow("binary_sobel", bev_bin_sobel);

   /* === 🆕 이진 BEV 시각화 (윈도우 + 픽셀 + 국소 피팅) === */
   cv::Mat bev_binary_debug = visualize_sliding_windows_binary(
       bev_bin_sobel,
       lanePoints_L,
       lanePoints_R,
       baseX_LR.first,
       baseX_LR.second,
       sw_cfg_,
       local_fits_L_,
       local_fits_R_
   );
   cv::imshow("BEV Binary with Windows", bev_binary_debug);
   
   /* === 🆕 컬러 BEV 시각화 (픽셀 + 최종 곡선만) === */
   cv::Mat bev_color_result = visualize_final_lanes_color(
       frame_bev,
       lanePoints_L,
       lanePoints_R,
       coeffs_L,
       coeffs_R,
       success_L,
       success_R
   );
   cv::imshow("BEV Color Final", bev_color_result);
   
   /* === 참고: 순수 이진화 결과 === */
   cv::imshow("binary_sobel", bev_bin_sobel);

    /* lane points on original image */
    std::vector<cv::Point2f> left_lane_points_bev;  // Use Point2f for perspectiveTransform
    std::vector<cv::Point2f> right_lane_points_bev;

    if (success_L && success_R) // Only draw if both lanes were successfully fitted
    {
        double la = coeffs_L[0], lb = coeffs_L[1], lc = coeffs_L[2], ld = coeffs_L[3];
        double ra = coeffs_R[0], rb = coeffs_R[1], rc = coeffs_R[2], rd = coeffs_R[3];

        for (int y = 0; y < h_bev; ++y) {
            double X = gnd_x1_m - y * METERS_PER_PIXEL_X;
            
            // lane L
            double Yl = la*X*X*X + lb*X*X + lc*X + ld;
            double uL = (gnd_y1_m - Yl) / METERS_PER_PIXEL_Y;
            if (uL >= 0 && uL < w_bev) {
                left_lane_points_bev.emplace_back(static_cast<float>(uL), static_cast<float>(y));
            }

            // lane R
            double Yr = ra*X*X*X + rb*X*X + rc*X + rd;
            double uR = (gnd_y1_m - Yr) / METERS_PER_PIXEL_Y;
            if (uR >= 0 && uR < w_bev) {
                right_lane_points_bev.emplace_back(static_cast<float>(uR), static_cast<float>(y));
            }
        }

        // 2. Combine points into a single polygon.
        std::vector<cv::Point2f> bev_lane_polygon;
        bev_lane_polygon.insert(bev_lane_polygon.end(), left_lane_points_bev.begin(), left_lane_points_bev.end());
        bev_lane_polygon.insert(bev_lane_polygon.end(), right_lane_points_bev.rbegin(), right_lane_points_bev.rend());

        // 3. Transform the polygon from BEV back to original image coordinates.
        // We use H_cam2gnd, which you already calculated.
        if (bev_lane_polygon.size() > 3)
        {
            std::vector<cv::Point2f> image_lane_polygon;
            cv::perspectiveTransform(bev_lane_polygon, image_lane_polygon, H_cam2gnd);

            // 4. Draw the projected lane area as a transparent overlay.
            cv::Mat overlay = frame_undist.clone();
            std::vector<cv::Point> image_lane_polygon_int(image_lane_polygon.begin(), image_lane_polygon.end());
            
            cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{image_lane_polygon_int}, cv::Scalar(0, 255, 0), cv::LINE_AA);

            // Blend the overlay with the original frame for a transparent effect
            double alpha = 0.4;
            cv::addWeighted(overlay, alpha, frame_undist, 1.0 - alpha, 0, frame_undist);
        }
    }
    // Display the final result with the lane overlay
    // cv::imshow("Final Result with Lane Overlay", frame_undist);

    /* waitKey() */
    char key = (char)cv::waitKey(40);
    if (key == 'q' || key == 27)
    { // 'q' 키 또는 ESC 키
        if (flag_debug_enable)
        {
            std::cout << "[DEBUG] Quitting by user request." << std::endl;
        }
        // break;
        cv::destroyAllWindows();
        // return -1;
        return bev_color_result;
    }

    return bev_color_result;
}

cv::Mat LaneDetector::visualize_sliding_windows(const cv::Mat& bev_image,
                                                 const std::vector<cv::Point>& lanePoints_L,
                                                 const std::vector<cv::Point>& lanePoints_R,
                                                 int baseX_L, int baseX_R,
                                                 int nwindows, int margin)
{
    cv::Mat result = bev_image.clone();
    int height = result.rows;
    int width = result.cols;
    int windowHeight = height / nwindows;
    
    // 왼쪽 차선 윈도우 그리기
    if (!lanePoints_L.empty()) {
        int currentX = baseX_L;
        
        for (int window = 1; window < nwindows; ++window) {
            int win_y_low = height - (window + 1) * windowHeight;
            int win_y_high = height - window * windowHeight;
            if (win_y_low < 0) win_y_low = 0;
            
            int win_x_low = currentX - margin;
            int win_x_high = currentX + margin;
            if (win_x_low < 0) win_x_low = 0;
            if (win_x_high >= width) win_x_high = width - 1;
            
            // 윈도우 사각형 그리기 (초록색)
            cv::rectangle(result, 
                         cv::Point(win_x_low, win_y_low),
                         cv::Point(win_x_high, win_y_high),
                         cv::Scalar(0, 255, 0), 2);
            
            // 다음 윈도우 중심 계산 (현재 윈도우 내 포인트들의 평균)
            std::vector<cv::Point> window_points;
            for (const auto& pt : lanePoints_L) {
                if (pt.y >= win_y_low && pt.y < win_y_high &&
                    pt.x >= win_x_low && pt.x <= win_x_high) {
                    window_points.push_back(pt);
                }
            }
            
            if (window_points.size() > 0) {
                long sumX = 0;
                for (const auto& pt : window_points) {
                    sumX += pt.x;
                }
                currentX = sumX / window_points.size();
            }
        }
    }
    
    // 오른쪽 차선 윈도우 그리기
    if (!lanePoints_R.empty()) {
        int currentX = baseX_R;
        
        for (int window = 1; window < nwindows; ++window) {
            int win_y_low = height - (window + 1) * windowHeight;
            int win_y_high = height - window * windowHeight;
            if (win_y_low < 0) win_y_low = 0;
            
            int win_x_low = currentX - margin;
            int win_x_high = currentX + margin;
            if (win_x_low < 0) win_x_low = 0;
            if (win_x_high >= width) win_x_high = width - 1;
            
            // 윈도우 사각형 그리기 (파란색)
            cv::rectangle(result,
                         cv::Point(win_x_low, win_y_low),
                         cv::Point(win_x_high, win_y_high),
                         cv::Scalar(255, 0, 0), 2);
            
            // 다음 윈도우 중심 계산
            std::vector<cv::Point> window_points;
            for (const auto& pt : lanePoints_R) {
                if (pt.y >= win_y_low && pt.y < win_y_high &&
                    pt.x >= win_x_low && pt.x <= win_x_high) {
                    window_points.push_back(pt);
                }
            }
            
            if (window_points.size() > 0) {
                long sumX = 0;
                for (const auto& pt : window_points) {
                    sumX += pt.x;
                }
                currentX = sumX / window_points.size();
            }
        }
    }
    
    return result;
}

cv::Mat LaneDetector::draw_lane_on_original(const cv::Mat& frame_original,
                                             const cv::Vec4d& coeffs_L,
                                             const cv::Vec4d& coeffs_R,
                                             bool fill_area)
{
    if (!success_L || !success_R) {
        return frame_original.clone();
    }

    cv::Mat result = frame_original.clone();
    
    // 1. BEV 좌표계에서 차선 포인트 생성
    std::vector<cv::Point2f> left_lane_bev;
    std::vector<cv::Point2f> right_lane_bev;
    
    double la = coeffs_L[0], lb = coeffs_L[1], lc = coeffs_L[2], ld = coeffs_L[3];
    double ra = coeffs_R[0], rb = coeffs_R[1], rc = coeffs_R[2], rd = coeffs_R[3];
    
    // BEV 이미지의 각 y 픽셀에 대해 x 좌표 계산
    for (int v = 0; v < h_bev; ++v) {
        // BEV 픽셀 좌표를 지면 좌표로 변환
        double X = gnd_x1_m - v * METERS_PER_PIXEL_X;
        
        // 좌측 차선
        double Yl = la * X * X * X + lb * X * X + lc * X + ld;
        double uL = (gnd_y1_m - Yl) / METERS_PER_PIXEL_Y;
        if (uL >= 0 && uL < w_bev) {
            left_lane_bev.emplace_back(static_cast<float>(uL), static_cast<float>(v));
        }
        
        // 우측 차선
        double Yr = ra * X * X * X + rb * X * X + rc * X + rd;
        double uR = (gnd_y1_m - Yr) / METERS_PER_PIXEL_Y;
        if (uR >= 0 && uR < w_bev) {
            right_lane_bev.emplace_back(static_cast<float>(uR), static_cast<float>(v));
        }
    }
    
    // 2. BEV → 원본 이미지 좌표 변환 (H_cam2gnd 사용)
    std::vector<cv::Point2f> left_lane_img, right_lane_img;
    if (!left_lane_bev.empty()) {
        cv::perspectiveTransform(left_lane_bev, left_lane_img, H_cam2gnd);
    }
    if (!right_lane_bev.empty()) {
        cv::perspectiveTransform(right_lane_bev, right_lane_img, H_cam2gnd);
    }
    
    // 3. 원본 이미지에 차선 그리기
    cv::Mat overlay = result.clone();
    
    // 차선 영역 채우기
    if (fill_area && !left_lane_img.empty() && !right_lane_img.empty()) {
        std::vector<cv::Point2f> lane_polygon;
        lane_polygon.insert(lane_polygon.end(), left_lane_img.begin(), left_lane_img.end());
        lane_polygon.insert(lane_polygon.end(), right_lane_img.rbegin(), right_lane_img.rend());
        
        std::vector<cv::Point> lane_polygon_int(lane_polygon.begin(), lane_polygon.end());
        cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{lane_polygon_int}, 
                     cv::Scalar(0, 255, 0), cv::LINE_AA);
    }
    
    // 차선 라인 그리기
    if (!left_lane_img.empty()) {
        std::vector<cv::Point> left_int(left_lane_img.begin(), left_lane_img.end());
        cv::polylines(overlay, left_int, false, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
    }
    if (!right_lane_img.empty()) {
        std::vector<cv::Point> right_int(right_lane_img.begin(), right_lane_img.end());
        cv::polylines(overlay, right_int, false, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }
    
    // 4. 블렌딩
    double alpha = 0.4;
    cv::addWeighted(overlay, alpha, result, 1.0 - alpha, 0, result);
    
    return result;
}

cv::Mat LaneDetector::draw_lane_points_on_original(const cv::Mat& frame_original,
                                                     const std::vector<cv::Point>& lanePoints_L,
                                                     const std::vector<cv::Point>& lanePoints_R)
{
    cv::Mat result = frame_original.clone();
    
    // BEV 픽셀 좌표를 원본 이미지 좌표로 변환
    if (!lanePoints_L.empty()) {
        std::vector<cv::Point2f> points_L_f(lanePoints_L.begin(), lanePoints_L.end());
        std::vector<cv::Point2f> points_L_img;
        cv::perspectiveTransform(points_L_f, points_L_img, H_cam2gnd);
        
        // 각 포인트를 작은 원으로 표시
        for (const auto& pt : points_L_img) {
            cv::circle(result, cv::Point(pt), 2, cv::Scalar(255, 255, 0), -1);
        }
    }
    
    if (!lanePoints_R.empty()) {
        std::vector<cv::Point2f> points_R_f(lanePoints_R.begin(), lanePoints_R.end());
        std::vector<cv::Point2f> points_R_img;
        cv::perspectiveTransform(points_R_f, points_R_img, H_cam2gnd);
        
        for (const auto& pt : points_R_img) {
            cv::circle(result, cv::Point(pt), 2, cv::Scalar(0, 255, 255), -1);
        }
    }
    
    return result;
}

void LaneDetector::setup_transform()
{
    /* int,ext 정의 */
    // cv::Mat K = (cv::Mat_<double>(3, 3) << 
    //             596.242804413754, 0, 640,
    //             0, 606.212049838454, 360,
    //             0.0, 0.0, 1.0
    //         );
    // cv::Mat D = (cv::Mat_<double>(1, 4) << 
    //             0.145233418368253, -0.448981613840834, 0.348017752004174, -0.0460532929321028
    //         );
    // cv::Mat K_rect;
    // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
    //         K, D, cv::Size(W_ORG,H_ORG), cv::Mat::eye(3,3,CV_64F), K_rect, 0.0
    // ); // 여기부턴 K_rect가 K를 대신함
    
    // cv::Mat R = (cv::Mat_<double>(3, 3) << 
    //         0.02006879253265631, -0.9997020415116865, -0.01389502657959448,
    //         0.1828656803129355, 0.01733366206390713, -0.9829850899800778,
    //         0.9829330529237322, 0.01718640034506079, 0.1831590595990568
    //         );
    // cv::Mat t = (cv::Mat_<double>(3, 1) << 
    //             0.4657723062176374, 1.774409539061654, 2.652951619441843
    //         );
    // calib = CamCalib::Make(K_rect, D, R, t); // 입력 이미지, 픽셀 좌표 모두 왜곡 보정 된 상태

    /* LUT 계산 */
    cv::fisheye::initUndistortRectifyMap(calib.K, calib.D, cv::Mat(), camera_matrix_,
                                cv::Size(w_org, h_org), CV_16SC2,
                                mapx_, mapy_);

    /* BEV 변환 행렬 계산 */
    // std::vector<cv::Point3d> world_rect_pts;                        // 차량 좌표계
    std::vector<cv::Point2d> world_rect_pts;                        // 차량 좌표계(m)
    world_rect_pts.push_back(cv::Point2d(gnd_x1_m, gnd_y1_m)); // Top-Left(시계방향)
    world_rect_pts.push_back(cv::Point2d(gnd_x1_m, gnd_y2_m)); // Top-Right
    world_rect_pts.push_back(cv::Point2d(gnd_x2_m, gnd_y2_m)); // Bottom-Right
    world_rect_pts.push_back(cv::Point2d(gnd_x2_m, gnd_y1_m)); // Bottom-Left

    for (const auto &world_pt : world_rect_pts)
    {
        image_rect_pts.push_back(cv::Point2f(gndToPix(world_pt, calib)));
    }

    /* new roi points -> failed */
    // image_rect_pts.push_back(cv::Point2f(0.0, 462.0));
    // image_rect_pts.push_back(cv::Point2f(1280.0, 462.0));
    // image_rect_pts.push_back(cv::Point2f(1280.0, 720.0));
    // image_rect_pts.push_back(cv::Point2f(0.0, 720.0));

    std::vector<cv::Point2f> bev_rect_pts; // 픽셀 좌표
    bev_rect_pts.push_back(cv::Point2f(0, 0));
    bev_rect_pts.push_back(cv::Point2f(w_bev, 0));
    bev_rect_pts.push_back(cv::Point2f(w_bev, h_bev));
    bev_rect_pts.push_back(cv::Point2f(0, h_bev));

    H_gnd2cam = cv::getPerspectiveTransform(image_rect_pts, bev_rect_pts);
    H_cam2gnd = cv::getPerspectiveTransform(bev_rect_pts, image_rect_pts);

    if (flag_debug_enable)
    {
        std::cout << "[INFO] Setup phase complete.\n";
        std::cout << "-> Calculated BEV source points on image (사다리꼴):\n"
                    << image_rect_pts << std::endl;
        // std::cout << "-> BEV Matrix (H_gnd2cam) is ready.\n";
    }
}

cv::Mat LaneDetector::makeCleanLaneMask_SAFE(const cv::Mat& bev_bgr) {
    CV_Assert(bev_bgr.type()==CV_8UC3);
    const double mppX = METERS_PER_PIXEL_X;  // 세로축(전방)
    const double mppY = METERS_PER_PIXEL_Y;  // 가로축(좌우)

    // 0) gray + blur
    tmp_gray.create(bev_bgr.rows, bev_bgr.cols, CV_8U);
    cv::cvtColor(bev_bgr, tmp_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(tmp_gray, tmp_blur, cv::Size(5,5), 0);

    // 1) Sobel X(세로선 강조) + 낮은 임계
    cv::Sobel(tmp_blur, gx, CV_16S, 1,0,3);
    cv::convertScaleAbs(gx, tmp8);                 // |Gx| in 8U
    double Tedge = std::max(kMinMag,
        cv::threshold(tmp8, mag_mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU));
    if (Tedge < kMinMag) Tedge = kMinMag;
    cv::threshold(tmp8, mag_mask, Tedge, 255, cv::THRESH_BINARY);

    // 2) 밝기 하한(너무 어두운 에지 컷)
    cv::threshold(tmp_gray, dir_mask_u8, kMinBright, 255, cv::THRESH_BINARY);
    cv::bitwise_and(mag_mask, dir_mask_u8, bin_dir);   // edge AND bright

    // 3) 방향 게이트(|Gy|/|Gx| < tan θ), θ 완화(40°)
    cv::Sobel(tmp_blur, gy, CV_16S, 0,1,3);
    cv::Mat absgx, absgy; cv::convertScaleAbs(gx, absgx); cv::convertScaleAbs(gy, absgy);
    cv::divide(absgy + 1, absgx + 1, dir_ratio, 1.0, CV_32F);
    dir_mask_u8 = (dir_ratio < (float)std::tan(kThetaDeg*CV_PI/180.0));
    dir_mask_u8.convertTo(dir_mask_u8, CV_8U, 255);
    cv::bitwise_and(bin_dir, dir_mask_u8, bin_dir);

    // 4) 가로 스트라이프 제거(커널 축소)
    const int k_hor_len = std::max(9, (int)std::round(kHorizLen_m / mppY)); // px
    if ((int)k_hor.cols != k_hor_len) k_hor = cv::getStructuringElement(cv::MORPH_RECT, {k_hor_len,3});
    cv::morphologyEx(bin_dir, horiz, cv::MORPH_OPEN, k_hor);  // 가로만 남김
    cv::bitwise_and(bin_dir, ~horiz, no_horiz);               // 가로 제거

    // 5) 세로 연결성 복원
    const int k_ver_len = std::max(9, (int)std::round(kVertClose_m / mppY));
    if ((int)k_ver.rows != k_ver_len) k_ver = cv::getStructuringElement(cv::MORPH_RECT, {3,k_ver_len});
    cv::morphologyEx(no_horiz, clean, cv::MORPH_CLOSE, k_ver);

    // 6) 중앙/사이드/탑 마스킹(중앙은 일단 OFF로 진단 후 켜기)
    int W=clean.cols, H=clean.rows, cx=W/2;
    if (kUseCenterMask) {
        int excl = (int)std::round(0.6 / mppY); // 0.6 m만 먼저 시도
        cv::rectangle(clean, {cx-excl,0}, {cx+excl,H-1}, 0, cv::FILLED);
    }
    int side = std::max(4, (int)std::round(0.2 / mppY));
    cv::rectangle(clean, {0,0}, {side,H-1}, 0, cv::FILLED);
    cv::rectangle(clean, {W-1-side,0}, {W-1,H-1}, 0, cv::FILLED);
    cv::rectangle(clean, {0,0}, {W-1, (int)std::round(0.3/mppX)}, 0, cv::FILLED);

    return clean;
}

std::vector<cv::Point> LaneDetector::slidingWindow(const cv::Mat& binary, int baseX, 
                                // int nwindows=9, int margin=10, int minpix=50) {
                                int nwindows, int margin, int minpix) {
    CV_Assert(binary.type() == CV_8UC1);
    int width = binary.cols;
    int height = binary.rows;
    int windowHeight = height / nwindows;  // 윈도우 하나의 세로 높이
    
    int currentX = baseX;
    std::vector<cv::Point> lanePoints;
    lanePoints.reserve(1000);
    
    // 하단부터 윈도우를 위로 올려가며 반복
    for (int window = 0; window < nwindows; ++window) {
    // for (int window = 1; window < nwindows; ++window) { //왜 맨 밑에 윈도우 뺐지?
        // 현재 윈도우의 세로 범위 (y좌표 하한/상한)
        int win_y_low = height - (window + 1) * windowHeight;
        int win_y_high = height - window * windowHeight;
        if (win_y_low < 0) win_y_low = 0;
        
        // 현재 윈도우의 가로 범위 (x좌표 범위, currentX 중심으로 margin 폭)
        int win_x_low = currentX - margin;
        int win_x_high = currentX + margin;
        if (win_x_low < 0) win_x_low = 0;
        if (win_x_high >= width) win_x_high = width - 1;
        
        // ROI 설정 및 내부의 흰 픽셀 좌표 찾기
        cv::Rect windowRect(win_x_low, win_y_low, win_x_high - win_x_low + 1, win_y_high - win_y_low);
        cv::Mat roi = binary(windowRect);
        std::vector<cv::Point> nonZeroLocs;
        cv::findNonZero(roi, nonZeroLocs);
        
        // ROI내 흰 픽셀들을 lanePoints에 추가 (전역 좌표계로 변환)
        for (const cv::Point& pt : nonZeroLocs) {
            int global_x = pt.x + win_x_low;
            int global_y = pt.y + win_y_low;
            lanePoints.emplace_back(global_x, global_y);
        }
        
        // 충분한 픽셀이 있으면, 그 x 평균으로 다음 윈도우 중심 이동
        if ((int)nonZeroLocs.size() > minpix) {
            long sumX = 0;
            for (const cv::Point& pt : nonZeroLocs) {
                sumX += pt.x;
            }
            int meanX = win_x_low + static_cast<int>( sumX / nonZeroLocs.size() );
            currentX = meanX;
        }
    }
    
    return lanePoints;
}

// std::vector<cv::Point> LaneDetector::slidingWindowAdaptive(
//                                         const cv::Mat& binary, 
//                                         int baseX, 
//                                         const AdaptiveWindow& cfg, 
//                                         int nwindows)
// {
//     int height = binary.rows;
//     int width = binary.cols;
//     int windowHeight = height / nwindows;
    
//     int currentX = baseX;
//     int currentMargin = cfg.margin_min;

//     std::vector<cv::Point> lanePoints;
//     lanePoints.reserve(2000);
    
//     for (int window = 0; window < nwindows; ++window) {
//         int win_y_low = height - (window + 1) * windowHeight;
//         int win_y_high = height - window * windowHeight;
//         if (win_y_low < 0) win_y_low = 0;
        
//         // ✅ 적응형 마진 (상단으로 갈수록 확장)
//         int adaptive_margin = currentMargin + (window * 3);
//         if (adaptive_margin > cfg.margin_max) adaptive_margin = cfg.margin_max;
        
//         int win_x_low = std::max(0, currentX - adaptive_margin);
//         int win_x_high = std::min(width - 1, currentX + adaptive_margin);
        
//         // ROI 내 픽셀 수집
//         cv::Rect roi_rect(win_x_low, win_y_low, 
//                           win_x_high - win_x_low + 1, win_y_high - win_y_low);
//         cv::Mat roi = binary(roi_rect);
        
//         std::vector<cv::Point> nonZeroLocs;
//         cv::findNonZero(roi, nonZeroLocs);
        
//         // 전역 좌표로 변환
//         for (const auto& pt : nonZeroLocs) {
//             lanePoints.emplace_back(pt.x + win_x_low, pt.y + win_y_low);
//         }
        
//         // ✅ 강한 신호면 중심 업데이트 + 마진 축소
//         if (nonZeroLocs.size() >= cfg.minpix_strong) {
//             long sumX = 0;
//             for (const auto& pt : nonZeroLocs) sumX += pt.x;
//             currentX = win_x_low + (sumX / nonZeroLocs.size());
//             currentMargin = cfg.margin_min;  // 신뢰도 높으면 좁게
//         }
//         // ✅ 약한 신호면 중심만 업데이트, 마진 유지
//         else if (nonZeroLocs.size() >= cfg.minpix_weak) {
//             long sumX = 0;
//             for (const auto& pt : nonZeroLocs) sumX += pt.x;
//             currentX = win_x_low + (sumX / nonZeroLocs.size());
//             // 마진은 점진적 확장 유지
//         }
//         // else: currentX와 margin 변경 없음
//     }
    
//     return lanePoints;
// }

LocalFitResult LaneDetector::fitLocalLine(const std::vector<cv::Point>& points,
                                          int offset_x,
                                          int offset_y)
{
    LocalFitResult result;

    if (points.size() < 3) {
        return result;
    }

    // 최소자승법: y=ax+b
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    int n = points.size();

    for (const auto& pt : points) {
        double x = pt.x + offset_x;
        double y = pt.y + offset_y;
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    double denom = n * sum_x2 - sum_x * sum_x;
    if (std::abs(denom) < 1e-6) {
        return result;
    }

    result.slope = (n * sum_xy - sum_x * sum_y) / denom;
    result.intercept = (sum_y - result.slope * sum_x) / n;

    // R² 계산
    double mean_y = sum_y / n;
    double ss_total = 0.0;
    double ss_residual = 0.0;
    
    for (const auto& pt : points) {
        double x = pt.x + offset_x;
        double y = pt.y + offset_y;
        double y_pred = result.slope * x + result.intercept;
        
        ss_total += (y - mean_y) * (y - mean_y);
        ss_residual += (y - y_pred) * (y - y_pred);
    }
    
    result.R2 = (ss_total > 1e-6) ? (1.0 - ss_residual / ss_total) : 0.0;
    result.success = true;

    return result;
}

int LaneDetector::computeMeanX(const std::vector<cv::Point>& points,
                               int offset_x)
{
    if (points.empty()) return 0;
    
    long sum_x = 0;
    for (const auto& pt : points) {
        sum_x += pt.x;
    }
    
    return offset_x + (sum_x / points.size());
}

std::pair<std::vector<cv::Point>, WindowStateExtended> LaneDetector::slidingWindow2(const cv::Mat& binary,
                                                                            int baseX,
                                                                            const SlidingWindowConfig& cfg)
{
    CV_Assert(binary.type() == CV_8UC1);
    
    int height = binary.rows;
    int width = binary.cols;
    int windowHeight = height / cfg.nwindows;
    
    WindowStateExtended state(baseX);
    std::vector<cv::Point> lanePoints;
    lanePoints.reserve(2000);
    
    for (int window = 0; window < cfg.nwindows; ++window) { // 하단->상단
        
        // 윈도우 영역 계산
        int win_y_low = height - (window + 1) * windowHeight;
        int win_y_high = height - window * windowHeight;
        if (win_y_low < 0) win_y_low = 0;
        
        // 이전 기울기 활용
        int searchX = state.currentX;
        if (std::abs(state.prevSlope) > 1e-6) {
            int predictedDeltaX = (int)(state.prevSlope * windowHeight * cfg.momentum);
            searchX = state.currentX + predictedDeltaX;
            searchX = std::clamp(searchX, 0, width - 1);
        }
        
        // ROI 설정
        int win_x_low = std::max(0, searchX - cfg.margin);
        int win_x_high = std::min(width - 1, searchX + cfg.margin);
        
        cv::Rect windowRect(win_x_low, win_y_low, 
                            win_x_high - win_x_low + 1, 
                            win_y_high - win_y_low);
        cv::Mat roi = binary(windowRect);
        
        std::vector<cv::Point> nonZeroLocs;
        cv::findNonZero(roi, nonZeroLocs);
        
        // 단절 감지
        if ((int)nonZeroLocs.size() < cfg.minpix_weak) {
            state.consecutive_fails++;
            
            if (state.consecutive_fails >= cfg.consecutive_fail_threshold 
                && !state.is_broken) {
                state.is_broken = true;
                state.break_y_position = win_y_high;
                break;  // 탐색 중단
            }
            
            // 예측 위치 유지하고 계속
            state.currentX = searchX;
            state.prevSlope *= 0.8;
            continue;
        }
        state.consecutive_fails = 0; // 픽셀 발견되면 초기화
        
        // 전역 좌표로 변환하여 저장
        for (const auto& pt : nonZeroLocs) {
            int global_x = pt.x + win_x_low;
            int global_y = pt.y + win_y_low;
            lanePoints.emplace_back(global_x, global_y);
        }
        
        // 다음 중심 결정 (국소 피팅 vs 평균)
        int nextX;
        if ((int)nonZeroLocs.size() >= cfg.minpix_for_fitting) {
            auto fit_result = fitLocalLine(nonZeroLocs, win_x_low, win_y_low);
            state.local_fits.push_back(fit_result);

            if (fit_result.success && fit_result.R2 >= cfg.R2_threshold) {
                // 피팅 성공 → 기울기 사용
                double dy = windowHeight;
                nextX = state.currentX + (int)(fit_result.slope * dy);
                
                // 급격한 변화 제한
                int deltaX = nextX - state.currentX;
                if (std::abs(deltaX) > cfg.max_lateral_jump) {
                    deltaX = (deltaX > 0) ? cfg.max_lateral_jump : -cfg.max_lateral_jump;
                    nextX = state.currentX + deltaX;
                }
                state.prevSlope = fit_result.slope;

            } else {
                state.local_fits.push_back(LocalFitResult{});
                // 피팅 실패 → 평균 사용
                nextX = computeMeanX(nonZeroLocs, win_x_low);
                state.prevSlope = (nextX - state.currentX) / (double)windowHeight;
            }
        } else {
            // 픽셀 부족 → 평균만 사용
            nextX = computeMeanX(nonZeroLocs, win_x_low);
            state.prevSlope = (nextX - state.currentX) / (double)windowHeight;
        }
        
        state.currentX = std::clamp(nextX, 0, width - 1);
    }
    
    return {lanePoints, state};
}


bool LaneDetector::fitLaneRANSAC_gnd(
                    const std::vector<cv::Point2f>& lanePoints, 
                    cv::Vec4d& out_coeffs, // y=f(x)=aX³ + bX² + cX + d
                    int iterations = 150, double threshold_gnd_m = 0.25, double min_inlier_ratio = 0.35)
    {
        int min_points_for_model = 4;
        if (lanePoints.size() < 4) return false; // 3차 다항식 모델을 만들 최소 포인트 개수 4개

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<size_t> dist(0, lanePoints.size() - 1);

        int best_inlier_count = 0;
        std::vector<cv::Point2f> best_inliers;

        for (int i = 0; i < iterations; ++i)
        {
            // 1. 무작위로 4개의 고유한 포인트 샘플링
            std::vector<cv::Point2f> sample_points;
            std::vector<int> indices;
            while (indices.size() < min_points_for_model) {
                int idx = dist(gen);
                // 중복되지 않는 인덱스 선택
                if (std::find(indices.begin(), indices.end(), idx) == indices.end()) {
                    indices.push_back(idx);
                    sample_points.push_back(lanePoints[idx]);
                }
            }

            // 2. 모델 추정: 4개의 점으로 연립방정식 풀기 (Ax = B)
            // x = ay^3 + by^2 + cy + d(x)
            cv::Mat A(min_points_for_model, 4, CV_64F);
            cv::Mat B(min_points_for_model, 1, CV_64F);

            for (int j = 0; j < min_points_for_model; ++j) {
                double X = static_cast<double>(sample_points[j].x);
                double Y = static_cast<double>(sample_points[j].y);
                A.at<double>(j, 0) = std::pow(X, 3);
                A.at<double>(j, 1) = std::pow(X, 2);
                A.at<double>(j, 2) = X;
                A.at<double>(j, 3) = 1.0;

                B.at<double>(j, 0) = Y;
            }

            cv::Vec4d current_coeffs;
            if (!cv::solve(A, B, current_coeffs, cv::DECOMP_LU)) {
                // 행렬이 특이(singular)하여 해를 구할 수 없는 경우 건너뜀
                continue;
            }

            double a = current_coeffs[0], b = current_coeffs[1], c = current_coeffs[2], d = current_coeffs[3];

            // 3. 인라이어 탐색
            std::vector<cv::Point2f> current_inliers;
            for (const auto& p : lanePoints) {
                double x_p = static_cast<double>(p.x);
                double y_p = static_cast<double>(p.y);
                double y_pred = a * std::pow(x_p, 3) + b * std::pow(x_p, 2) + c * x_p + d;
                double error = std::abs(y_p - y_pred);

                if (error < threshold_gnd_m) {
                    current_inliers.push_back(p);
                }
            }
            
            // 4. 최고의 모델 업데이트
            if (current_inliers.size() > best_inlier_count) {
                best_inlier_count = current_inliers.size();
                best_inliers = current_inliers;
            }
        }

        // 5. 모델 재추정 (Refitting) - 최고의 인라이어 집합으로 다시 피팅
        if (best_inliers.size() > min_points_for_model && 
            (static_cast<double>(best_inliers.size()) / lanePoints.size()) > min_inlier_ratio)
        {
            cv::Mat A_refit(best_inliers.size(), 4, CV_64F);
            cv::Mat B_refit(best_inliers.size(), 1, CV_64F);

            for (size_t j = 0; j < best_inliers.size(); ++j) {
                // double y = static_cast<double>(best_inliers[j].y);
                // A_refit.at<double>(j, 0) = std::pow(y, 3);
                // A_refit.at<double>(j, 1) = std::pow(y, 2);
                // A_refit.at<double>(j, 2) = y;
                // A_refit.at<double>(j, 3) = 1.0;

                // B_refit.at<double>(j, 0) = static_cast<double>(best_inliers[j].x);

                double x = static_cast<double>(best_inliers[j].x);
                double y = static_cast<double>(best_inliers[j].y);
                A_refit.at<double>(j, 0) = std::pow(x, 3);
                A_refit.at<double>(j, 1) = std::pow(x, 2);
                A_refit.at<double>(j, 2) = x;
                A_refit.at<double>(j, 3) = 1.0;

                B_refit.at<double>(j, 0) = y;
            }

            // Overdetermined system이므로 SVD를 사용하여 최소자승법 해를 구함
            if (cv::solve(A_refit, B_refit, out_coeffs, cv::DECOMP_SVD)) {
                return true;
            }
        }

        return false;
    }
    
cv::Point2d LaneDetector::bevPixelToGround(const cv::Point2d& pixel_uv) const {
    // BEV 픽셀 좌표 -> 지면 좌표 변환
    // BEV 이미지: u(가로)=좌우, v(세로)=전후
    // 지면 좌표: X=전방, Y=좌우

    double X = gnd_x1_m - pixel_uv.y * METERS_PER_PIXEL_X;  // 전방
    double Y = gnd_y1_m - pixel_uv.x * METERS_PER_PIXEL_Y;  // 좌우

    return {X, Y};
}


cv::Mat LaneDetector::visualize_lane_lines_gnd(const cv::Mat& bev_image, const cv::Vec4d& left_coeffs, const cv::Vec4d& right_coeffs)
{
    cv::Mat overlay = cv::Mat::zeros(bev_image.size(), bev_image.type());
    int height = bev_image.rows;
    int width = bev_image.cols;

    std::vector<cv::Point> left_lane_points;
    std::vector<cv::Point> right_lane_points;

    double la = left_coeffs[0], lb = left_coeffs[1], lc = left_coeffs[2], ld = left_coeffs[3];
    double ra = right_coeffs[0], rb = right_coeffs[1], rc = right_coeffs[2], rd = right_coeffs[3];

    const double mppX = METERS_PER_PIXEL_X;
    const double mppY = METERS_PER_PIXEL_Y;
    for (int y = 0; y < height; ++y) {
        double X = gnd_x1_m - y * mppX;
        
        // lane L
        double Yl = la*X*X*X + lb*X*X + lc*X + ld;
        double uL = (gnd_y1_m - Yl) / mppY;
        if (uL>=0 && uL<width) {
            left_lane_points.emplace_back((int)std::lround(uL), y);
        }

        // lane R
        double Yr = ra*X*X*X + rb*X*X + rc*X + rd;
        double uR = (gnd_y1_m - Yr) / mppY;
        if (uR>=0 && uR<width) {
            right_lane_points.emplace_back((int)std::lround(uR), y);
        }
    }
    
    if (!left_lane_points.empty()) {
        cv::polylines(overlay, left_lane_points, false, cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }
    if (!right_lane_points.empty()) {
        cv::polylines(overlay, right_lane_points, false, cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }

    // 4. 차선 영역 채우기 (cv::fillPoly)
    if (!left_lane_points.empty() && !right_lane_points.empty()) {
        std::vector<cv::Point> lane_area_pts;
        lane_area_pts.insert(lane_area_pts.end(), left_lane_points.begin(), left_lane_points.end());
        // 오른쪽 차선은 아래에서 위로 올라가도록 역순으로 추가해야 다각형이 꼬이지 않음
        lane_area_pts.insert(lane_area_pts.end(), right_lane_points.rbegin(), right_lane_points.rend());
        
        cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{lane_area_pts}, cv::Scalar(0, 255, 0), cv::LINE_AA); // 초록색 주행 영역
    }

    // 5. 원본 이미지와 오버레이 합성
    cv::Mat result;
    cv::addWeighted(bev_image, 0.7, overlay, 0.3, 0.0, result);
    
    return result;
}


cv::Mat LaneDetector::visualize_lane_lines(const cv::Mat& bev_image, const cv::Vec4d& left_coeffs, const cv::Vec4d& right_coeffs)
{
    // 1. 시각화 요소를 그릴 빈 오버레이 이미지 생성
    cv::Mat overlay = cv::Mat::zeros(bev_image.size(), bev_image.type());
    
    int height = bev_image.rows;
    int width = bev_image.cols;

    // 2. 곡선 포인트를 저장할 벡터 생성
    std::vector<cv::Point> left_lane_points;
    std::vector<cv::Point> right_lane_points;

    double la = left_coeffs[0], lb = left_coeffs[1], lc = left_coeffs[2], ld = left_coeffs[3];
    double ra = right_coeffs[0], rb = right_coeffs[1], rc = right_coeffs[2], rd = right_coeffs[3];

    // 이미지 상단부터 하단까지 y값을 순회하며 x값 계산
    for (int y = 0; y < height; ++y) {
        double y_d = static_cast<double>(y);

        // 왼쪽 차선의 x좌표 계산
        double left_x = la * std::pow(y_d, 3) + lb * std::pow(y_d, 2) + lc * y_d + ld;
        
        // 오른쪽 차선의 x좌표 계산
        double right_x = ra * std::pow(y_d, 3) + rb * std::pow(y_d, 2) + rc * y_d + rd;
        
        // 유효한 좌표만 추가
        if (left_x >= 0 && left_x < width) {
            left_lane_points.push_back(cv::Point(static_cast<int>(left_x), y));
        }
        if (right_x >= 0 && right_x < width) {
            right_lane_points.push_back(cv::Point(static_cast<int>(right_x), y));
        }
    }

    // 3. 곡선 그리기 (cv::polyline)
    // isClosed=false, color, thickness
    if (!left_lane_points.empty()) {
        cv::polylines(overlay, left_lane_points, false, cv::Scalar(0, 0, 255), 3, cv::LINE_AA); // 빨간색 왼쪽 차선
    }
    if (!right_lane_points.empty()) {
        cv::polylines(overlay, right_lane_points, false, cv::Scalar(0, 0, 255), 3, cv::LINE_AA); // 파란색 오른쪽 차선
    }

    // 4. 차선 영역 채우기 (cv::fillPoly)
    // 왼쪽 차선 포인터와, 역순으로 정렬된 오른쪽 차선 포인트를 합쳐 다각형을 만듦
    if (!left_lane_points.empty() && !right_lane_points.empty()) {
        std::vector<cv::Point> lane_area_pts;
        lane_area_pts.insert(lane_area_pts.end(), left_lane_points.begin(), left_lane_points.end());
        // 오른쪽 차선은 아래에서 위로 올라가도록 역순으로 추가해야 다각형이 꼬이지 않음
        lane_area_pts.insert(lane_area_pts.end(), right_lane_points.rbegin(), right_lane_points.rend());
        
        cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{lane_area_pts}, cv::Scalar(0, 255, 0), cv::LINE_AA); // 초록색 주행 영역
    }

    // 5. 원본 이미지와 오버레이 합성
    cv::Mat result;
    cv::addWeighted(bev_image, 0.7, overlay, 0.3, 0.0, result);
    
    return result;
}

std::pair<int,int> LaneDetector::findLaneBasePositionsGated(
    const cv::Mat& binary,          // 0/255 BEV 이진 마스크
    double meters_per_px_x,         // 전후(m)→px (y축 샘플 간격) - 여기선 사용 안함
    double meters_per_px_y,         // 좌우(m)→px (가로 픽셀 환산)  = METERS_PER_PIXEL_Y
    int    prev_leftX     = -1,     // (선택) 이전 프레임 좌 예측
    int    prev_rightX    = -1,     // (선택) 이전 프레임 우 예측
    double lane_width_m   = 3.5,    // 차로폭 가정
    double band_m         = 0.8,    // 좌/우 탐색 밴드 반폭
    double bottom_ratio   = 0.35,   // 하단 몇 %만 히스토그램
    int    smooth_ksize   = 21,     // 히스토그램 스무딩 커널(홀수)
    int    peak_min_sum   = 1500,   // 피크 최소 강도(데이터에 맞게 튜닝)
    double prev_band_m    = 0.4     // (선택) 이전 예측 주변 좁은 밴드
    )
{
    (void)meters_per_px_x;

    CV_Assert(binary.type()==CV_8UC1);
    int W = binary.cols, H = binary.rows;

    // 1) 하단 영역만 사용
    int y0 = std::max(0, H - (int)std::round(H*bottom_ratio));
    cv::Mat lower = binary.rowRange(y0, H);

    // 2) 열 합 (히스토그램) + 스무딩
    cv::Mat colsum; cv::reduce(lower, colsum, 0, cv::REDUCE_SUM, CV_32S); // 1 x W
    if (smooth_ksize > 1) {
        cv::blur(colsum, colsum, cv::Size(smooth_ksize,1));
    }

    // 3) 기본 밴드 계산(차량 중심선 기준 좌/우)
    int cx = W/2;
    int half_lane_px = (int)std::round(0.5*lane_width_m / meters_per_px_y);
    int band_px      = (int)std::round(band_m        / meters_per_px_y);
    auto clamp = [&](int v){ return std::max(0, std::min(W-1, v)); };

    auto find_peak_in_range = [&](int x1, int x2)->std::pair<int,double>{
        x1 = clamp(x1); x2 = clamp(x2);
        if (x2 <= x1) return {-1, 0.0};
        cv::Mat roi = colsum.colRange(x1, x2);
        double minv,maxv; cv::Point minp,maxp;
        cv::minMaxLoc(roi, &minv, &maxv, &minp, &maxp);
        int x = x1 + maxp.x;
        return {x, maxv};
    };

    // 4) (옵션) 이전 프레임 예측 있으면 좁은 밴드 우선
    std::pair<int,double> L{-1,0}, R{-1,0};
    if (prev_leftX >= 0 && prev_rightX >= 0) {
        int pband = (int)std::round(prev_band_m / meters_per_px_y);
        L = find_peak_in_range(prev_leftX - pband,  prev_leftX + pband);
        R = find_peak_in_range(prev_rightX - pband, prev_rightX + pband);
    }

    // 5) 없거나 약하면 기본 밴드로 탐색
    if (L.first < 0 || L.second < peak_min_sum) {
        int lx1 = cx - half_lane_px - band_px;
        int lx2 = cx - half_lane_px + band_px;
        L = find_peak_in_range(lx1, lx2);
    }
    if (R.first < 0 || R.second < peak_min_sum) {
        int rx1 = cx + half_lane_px - band_px;
        int rx2 = cx + half_lane_px + band_px;
        R = find_peak_in_range(rx1, rx2);
    }

    // 6) 그래도 약하면 밴드 확장(점진적 폴백)
    if (L.first < 0 || L.second < peak_min_sum) {
        int lx1 = cx - half_lane_px - 2*band_px;
        int lx2 = cx - half_lane_px + 2*band_px;
        L = find_peak_in_range(lx1, lx2);
    }
    if (R.first < 0 || R.second < peak_min_sum) {
        int rx1 = cx + half_lane_px - 2*band_px;
        int rx2 = cx + half_lane_px + 2*band_px;
        R = find_peak_in_range(rx1, rx2);
    }

    // 7) 최종 보정/검증: 좌<우, 최소 간격 확보
    int leftX  = (L.first >= 0) ? L.first : clamp(cx - half_lane_px);
    int rightX = (R.first >= 0) ? R.first : clamp(cx + half_lane_px);
    if (leftX >= rightX) {
        // 간단 폴백: 중심 기준으로 재설정
        leftX  = clamp(cx - half_lane_px);
        rightX = clamp(cx + half_lane_px);
    }
    return {leftX, rightX};
}

void LaneDetector::pre_process()
{
    // if (flag_debug_enable) { std::cout << "[INFO] pre_process() starts.\n"; }

    /* 왜곡 보정 및 BEV 변환 */
    // cv::remap(frame_org, frame_undist, mapx_, mapy_, cv::INTER_LINEAR);
    // cv::warpPerspective(frame_undist, frame_bev, H_gnd2cam, cv::Size(w_bev, h_bev), cv::INTER_LINEAR);
    cv::warpPerspective(frame_org, frame_bev, H_gnd2cam, cv::Size(w_bev, h_bev), cv::INTER_LINEAR);

    /* grayscale, 이진화*/
    // cv::cvtColor(frame_bev, bev_gray, cv::COLOR_BGR2GRAY);
    // bev_bin_sobel = binarize_sobel(bev_gray);
    // bev_bin_ridge = binarize_ridge(bev_gray);
    // bev_bin_sobel = binarize_sobel_clean(frame_bev);
    bev_bin_sobel = makeCleanLaneMask_SAFE(frame_bev);
}

void LaneDetector::detect()
{
    /* TODO: Add ROI tracker for sliding window start points */
    // baseX_LR = findLaneBasePositions(bev_bin_ridge); 
    // baseX_LR = findLaneBasePositions(bev_bin_sobel); 
    baseX_LR = findLaneBasePositionsGated(bev_bin_sobel, METERS_PER_PIXEL_X, METERS_PER_PIXEL_Y,
                                            // prev_lX, prev_rX, 3.5, 0.8);
                                            prev_lX, prev_rX, 3.5, 0.4);
    prev_lX = baseX_LR.first;
    prev_rX = baseX_LR.second;
    // trackedX_LR = trackBasePositions(baseX_LR);

    // lanePoints_L = slidingWindow(bev_bin_sobel, baseX_LR.first, nWindows, margin, minPixels);
    // lanePoints_R = slidingWindow(bev_bin_sobel, baseX_LR.second, nWindows, margin, minPixels);
    // lanePoints_L = slidingWindowAdaptive(bev_bin_sobel, baseX_LR.first,  sw_cfg);
    // lanePoints_R = slidingWindowAdaptive(bev_bin_sobel, baseX_LR.second, sw_cfg);

    sw_cfg_.nwindows = 10;
    sw_cfg_.margin = 20;
    sw_cfg_.minpix_weak = 5;
    sw_cfg_.minpix_strong = 15;
    sw_cfg_.minpix_for_fitting = 8;
    sw_cfg_.R2_threshold = 0.6;
    sw_cfg_.momentum = 0.7;
    sw_cfg_.max_lateral_jump = 15;
    sw_cfg_.consecutive_fail_threshold = 3;
    
    auto [points_L, state_L] = slidingWindow2(bev_bin_sobel, baseX_LR.first, sw_cfg_);
    auto [points_R, state_R] = slidingWindow2(bev_bin_sobel, baseX_LR.second, sw_cfg_);
    
    lanePoints_L = std::move(points_L);
    lanePoints_R = std::move(points_R);

    local_fits_L_ = state_L.local_fits;
    local_fits_R_ = state_R.local_fits;

    // 🆕 단절 정보 업데이트
    break_info_L.is_broken = state_L.is_broken;
    break_info_L.valid_end_y = state_L.break_y_position;
    
    break_info_R.is_broken = state_R.is_broken;
    break_info_R.valid_end_y = state_R.break_y_position;
    
    // 기존 코드 유지
    prev_lX = state_L.currentX;
    prev_rX = state_R.currentX;

    if (flag_debug_enable) {
        std::cout << "slidinwindowL: " << lanePoints_L.size() << std::endl;
        std::cout << "slidinwindowR: " << lanePoints_R.size() << std::endl;
    }
}

std::pair<cv::Vec4d, cv::Vec4d> LaneDetector::polyFit()
{
    lanePointsGnd_L.clear(); 
    lanePointsGnd_R.clear();
    lanePointsGnd_L.reserve(lanePoints_L.size());
    lanePointsGnd_R.reserve(lanePoints_R.size());

    for (auto& p: lanePoints_L)
    {
        auto m = bevPixelToGround(p);
        lanePointsGnd_L.emplace_back(cv::Point2d((float)m.x, (float)m.y));
    }
    for (auto& p: lanePoints_R)
    {
        auto m = bevPixelToGround(p);
        lanePointsGnd_R.emplace_back(cv::Point2d((float)m.x, (float)m.y));
    }

    success_L = fitLaneRANSAC_gnd(lanePointsGnd_L, coeffs_L);
    success_R = fitLaneRANSAC_gnd(lanePointsGnd_R, coeffs_R); 

    if (flag_debug_enable) {
        std::cout << "L Lane Fit [a,b,c,d]: " << coeffs_L << std::endl;
        std::cout << "R Lane Fit [a,b,c,d]: " << coeffs_R << std::endl;
    }

    return {coeffs_L, coeffs_R};
}

int LaneDetector::visualization()
{

    /* 원본 프레임 */
    // std::string window_name_org = " Original frame Size: " + std::to_string(w_org) + " x " + std::to_string(h_org);
    // cv::imshow(window_name_org, frame_org);

    /* 왜곡보정, BEV 범위 표시 */
    std::vector<cv::Point> points_int(image_rect_pts.begin(), image_rect_pts.end());
    const cv::Point *ppt[1] = {points_int.data()};
    int npt[] = {(int)points_int.size()};
    cv::polylines(frame_undist, ppt, npt, 1, true, cv::Scalar(0, 255, 0), 2);
    // draw_guidelines_on_image(frame_undist);
    std::string window_name_undist = "Undistorted frame";
    // cv::imshow(window_name_undist, frame_undist);

    /* BEV */
    // std::string window_name_bev = " BEV frame Size: " + std::to_string(w_bev) + " x " + std::to_string(h_bev);
    // cv::imshow(window_name_bev, frame_bev);

    // 라인 시작점 표시
    cv::circle(frame_bev, cv::Point(baseX_LR.first, h_bev - 1), 10, cv::Scalar(0, 0, 255), -1);
    cv::circle(frame_bev, cv::Point(baseX_LR.second, h_bev - 1), 10, cv::Scalar(0, 0, 255), -1); 
    
    // 슬라이딩 윈도우로 얻은 차선 픽셀 표시 
    cv::Mat laneMask(frame_bev.size(), CV_8U, cv::Scalar(0));
    for (const auto& pl:lanePoints_L) {
        laneMask.at<uchar>(pl) = 255;
    }
    for (const auto& pr:lanePoints_R) {
        laneMask.at<uchar>(pr) = 255;
    }
    frame_bev.setTo(cv::Scalar(0,255,255), laneMask); 



    // frame_bev = visualize_lane_lines(frame_bev, coeffs_L, coeffs_R);
    frame_bev = visualize_lane_lines_gnd(frame_bev, coeffs_L, coeffs_R);
    cv::imshow("BEV", frame_bev);
    // cv::imshow("BIN", bev_bin);
    cv::imshow("binary_sobel", bev_bin_sobel);
    // cv::imshow("binary_ridge", bev_bin_ridge);

    /* waitKey() */
    char key = (char)cv::waitKey(25); // 약 40 FPS
    if (key == 'q' || key == 27)
    { // 'q' 키 또는 ESC 키
        if (flag_debug_enable)
        {
            std::cout << "[DEBUG] Quitting by user request." << std::endl;
        }
        // break;
        cv::destroyAllWindows();
        return -1;
    }

    return 0;
}

cv::Mat LaneDetector::visualize_sliding_windows_binary(
    const cv::Mat& binary_image,
    const std::vector<cv::Point>& lanePoints_L,
    const std::vector<cv::Point>& lanePoints_R,
    int baseX_L, 
    int baseX_R,
    const SlidingWindowConfig& cfg,
    const std::vector<LocalFitResult>& local_fits_L,
    const std::vector<LocalFitResult>& local_fits_R)
{
    // 1. 이진 이미지를 컬러로 변환 (시각화용)
    cv::Mat result;
    cv::cvtColor(binary_image, result, cv::COLOR_GRAY2BGR);
    
    int height = result.rows;
    int width = result.cols;
    int windowHeight = height / cfg.nwindows;
    
    // === 좌측 차선 처리 ===
    if (!lanePoints_L.empty()) {
        int currentX = baseX_L;
        
        for (int window = 0; window < cfg.nwindows; ++window) {
            int win_y_low = height - (window + 1) * windowHeight;
            int win_y_high = height - window * windowHeight;
            if (win_y_low < 0) win_y_low = 0;
            
            int win_x_low = std::max(0, currentX - cfg.margin);
            int win_x_high = std::min(width - 1, currentX + cfg.margin);
            
            // 🔵 슬라이딩 윈도우 사각형 (파란색, 얇게)
            cv::rectangle(result, 
                         cv::Point(win_x_low, win_y_low),
                         cv::Point(win_x_high, win_y_high),
                         cv::Scalar(255, 0, 0), 1);  // 파란색, 1px
            
            // 현재 윈도우 내 포인트 수집
            std::vector<cv::Point> window_points;
            for (const auto& pt : lanePoints_L) {
                if (pt.y >= win_y_low && pt.y < win_y_high &&
                    pt.x >= win_x_low && pt.x <= win_x_high) {
                    window_points.push_back(pt);
                    
                    // 🟢 검출된 픽셀 표시 (초록색)
                    cv::circle(result, pt, 1, cv::Scalar(0, 255, 0), -1);
                }
            }
            
            // 🟡 국소 피팅 직선 그리기
            if (window < (int)local_fits_L.size() && local_fits_L[window].success) {
                auto fit = local_fits_L[window];
                
                // 직선 시작/끝점 계산
                int y1 = win_y_low;
                int y2 = win_y_high;
                int x1 = (int)(fit.slope * y1 + fit.intercept);
                int x2 = (int)(fit.slope * y2 + fit.intercept);
                
                // 노란색 직선 (굵게)
                cv::line(result, 
                        cv::Point(x1, y1), 
                        cv::Point(x2, y2),
                        cv::Scalar(0, 255, 255), 2);  // 노란색, 2px
                
                // R² 값 표시 (작은 텍스트)
                std::string r2_text = cv::format("R2:%.2f", fit.R2);
                cv::putText(result, r2_text,
                           cv::Point(win_x_low + 2, win_y_low + 12),
                           cv::FONT_HERSHEY_SIMPLEX, 0.3,
                           cv::Scalar(0, 255, 255), 1);
            }
            
            // 다음 윈도우 중심 계산
            if (!window_points.empty()) {
                long sumX = 0;
                for (const auto& pt : window_points) sumX += pt.x;
                currentX = sumX / window_points.size();
            }
        }
    }
    
    // === 우측 차선 처리 (동일 로직) ===
    if (!lanePoints_R.empty()) {
        int currentX = baseX_R;
        
        for (int window = 0; window < cfg.nwindows; ++window) {
            int win_y_low = height - (window + 1) * windowHeight;
            int win_y_high = height - window * windowHeight;
            if (win_y_low < 0) win_y_low = 0;
            
            int win_x_low = std::max(0, currentX - cfg.margin);
            int win_x_high = std::min(width - 1, currentX + cfg.margin);
            
            // 🔴 슬라이딩 윈도우 (빨간색)
            cv::rectangle(result,
                         cv::Point(win_x_low, win_y_low),
                         cv::Point(win_x_high, win_y_high),
                         cv::Scalar(0, 0, 255), 1);  // 빨간색
            
            std::vector<cv::Point> window_points;
            for (const auto& pt : lanePoints_R) {
                if (pt.y >= win_y_low && pt.y < win_y_high &&
                    pt.x >= win_x_low && pt.x <= win_x_high) {
                    window_points.push_back(pt);
                    
                    // 🟣 검출된 픽셀 (마젠타)
                    cv::circle(result, pt, 1, cv::Scalar(255, 0, 255), -1);
                }
            }
            
            // 🟡 국소 피팅 직선
            if (window < (int)local_fits_R.size() && local_fits_R[window].success) {
                auto fit = local_fits_R[window];
                int y1 = win_y_low;
                int y2 = win_y_high;
                int x1 = (int)(fit.slope * y1 + fit.intercept);
                int x2 = (int)(fit.slope * y2 + fit.intercept);
                
                cv::line(result, 
                        cv::Point(x1, y1), 
                        cv::Point(x2, y2),
                        cv::Scalar(0, 255, 255), 2);
                
                std::string r2_text = cv::format("R2:%.2f", fit.R2);
                cv::putText(result, r2_text,
                           cv::Point(win_x_high - 40, win_y_low + 12),
                           cv::FONT_HERSHEY_SIMPLEX, 0.3,
                           cv::Scalar(0, 255, 255), 1);
            }
            
            if (!window_points.empty()) {
                long sumX = 0;
                for (const auto& pt : window_points) sumX += pt.x;
                currentX = sumX / window_points.size();
            }
        }
    }
    
    return result;
}

cv::Mat LaneDetector::visualize_final_lanes_color(const cv::Mat& color_bev,
                                                const std::vector<cv::Point>& lanePoints_L,
                                                const std::vector<cv::Point>& lanePoints_R,
                                                const cv::Vec4d& coeffs_L,
                                                const cv::Vec4d& coeffs_R,
                                                bool success_L,
                                                bool success_R)
{
    cv::Mat result = color_bev.clone();
    
    // 1️⃣ 검출된 픽셀 표시 (밝은 색상)
    for (const auto& pt : lanePoints_L) {
        // 🟡 좌측 차선 픽셀 (노란색)
        cv::circle(result, pt, 2, cv::Scalar(0, 255, 255), -1);
    }
    
    for (const auto& pt : lanePoints_R) {
        // 🟠 우측 차선 픽셀 (주황색)
        cv::circle(result, pt, 2, cv::Scalar(0, 165, 255), -1);
    }
    
    // 2️⃣ 최종 피팅 곡선 그리기
    if (success_L && success_R) {
        cv::Mat overlay = result.clone();
        
        std::vector<cv::Point> left_curve, right_curve;
        
        double la = coeffs_L[0], lb = coeffs_L[1], lc = coeffs_L[2], ld = coeffs_L[3];
        double ra = coeffs_R[0], rb = coeffs_R[1], rc = coeffs_R[2], rd = coeffs_R[3];
        
        int height = result.rows;
        int width = result.cols;
        
        for (int y = 0; y < height; ++y) {
            double X = gnd_x1_m - y * METERS_PER_PIXEL_X;
            
            // 좌측 곡선
            double Yl = la*X*X*X + lb*X*X + lc*X + ld;
            double uL = (gnd_y1_m - Yl) / METERS_PER_PIXEL_Y;
            if (uL >= 0 && uL < width) {
                left_curve.emplace_back((int)std::lround(uL), y);
            }
            
            // 우측 곡선
            double Yr = ra*X*X*X + rb*X*X + rc*X + rd;
            double uR = (gnd_y1_m - Yr) / METERS_PER_PIXEL_Y;
            if (uR >= 0 && uR < width) {
                right_curve.emplace_back((int)std::lround(uR), y);
            }
        }
        
        // 🔵 최종 곡선 그리기 (굵은 파란색)
        if (!left_curve.empty()) {
            cv::polylines(overlay, left_curve, false, 
                         cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
        }
        if (!right_curve.empty()) {
            cv::polylines(overlay, right_curve, false, 
                         cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
        }
        
        // 🟢 차선 영역 채우기 (반투명 초록)
        if (!left_curve.empty() && !right_curve.empty()) {
            std::vector<cv::Point> lane_area;
            lane_area.insert(lane_area.end(), left_curve.begin(), left_curve.end());
            lane_area.insert(lane_area.end(), right_curve.rbegin(), right_curve.rend());
            
            cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{lane_area}, 
                        cv::Scalar(0, 255, 0), cv::LINE_AA);
        }
        
        // 블렌딩
        cv::addWeighted(overlay, 0.3, result, 0.7, 0, result);
    }
    
    return result;
}

/* 거리 계산 로직 */
const float Ext[3][3] = {
    { -0.000679f, 0.001056f, -0.382802f },
    { 0.004514f, 0.004514f, -2.299081f },
    { -0.000062f, -0.008053f, 1.000000f }
};
const float K[3][3] = {
    { 820.906100f, 0.f, 1049.183623f },
    { 0.f, 817.774538f, 588.665911f },
    { 0.f, 0.f, 1.f }
};
const float K_new[3][3] = {
    { 123.135915f, 0.f, 269.183623f },
    { 0.f, 122.666181f, 138.665911f },
    { 0.f, 0.f, 1.f }
};

const double D[4] = { -0.040784, -0.003428, -0.001554, 0.000751 };

int count = 1;          // 변화 가능
double f[2] = {};       // 필요하면 초기화
double c[2] = {};

// int tangencialUndistort(
//     float x_in,    ///< 입력 x 좌표, 0.0 ~ 1.0으로 normalize
//     float y_in,    ///< 입력 y 좌표, 0.0 ~ 1.0으로 normalize
//     float* x_out,  ///< 출력 x 좌표, 0.0 ~ 1.0으로 normalize
//     float* y_out   ///< 출력 y 좌표, 0.0 ~ 1.0으로 normalize
// )
// {
//     float p1 = 0.0f;
//     float p2 = 0.0f;

//     x_in -= 0.5f;
//     y_in -= 0.5f;
//     float r2;
//     float x_corrected, y_corrected;

//     r2 = (x_in * x_in) + (y_in * y_in);
//     x_corrected = x_in + ((2 * (-p1) * x_in * y_in) + ((-p2) * (r2 + 2 * x_in * x_in)));
//     y_corrected = y_in + (((-p1) * (r2 + 2 * y_in * y_in)) + (2 * (-p2) * x_in * y_in));

//     x_in = x_corrected;
//     y_in = y_corrected;

//     r2 = (x_in * x_in) + (y_in * y_in);
//     x_corrected = x_in + ((2 * (-p1 / 10.0f) * x_in * y_in) + ((-p2 / 10.0f) * (r2 + 2 * x_in * x_in)));
//     y_corrected = y_in + (((-p1 / 10.0f) * (r2 + 2 * y_in * y_in)) + (2 * (-p2 / 10.0f) * x_in * y_in));

//     *x_out = x_corrected + 0.5f;
//     *y_out = y_corrected + 0.5f;

//     return 0;
// }

// cv::Point2f convert_cam2gnd(cv::Point2f in)
// {   
//     float w_org = 1280.0f;
//     float h_org = 720.0f;

//     f[0] = K[0][0];
//     f[1] = K[1][1];
//     c[0] = K[0][2];
//     c[1] = K[1][2];

//     /* normalize */   
//     cv::Point2f in_norm;
//     in_norm.x = in.x / (float)w_org;
//     in_norm.y = in.y / (float)h_org;

//     /* undistort */
    
//     vec2 pi = {in_norm.x, in_norm.y};  // image point

// /// tangencial 오차 보정
//     tangencialUndistort(pi.x, pi.y, &pi.x, &pi.y);

//     pi.x *= (float)(w_org - 1);
//     pi.y *= (float)(h_org - 1); //?

//     vec2 pw = make_vec2((pi.x - c[0]) / f[0], (pi.y - c[1]) / f[1]);      //정규화된 카메라좌표, Z=1 평면에 맺힌 점으로
//     double scale = 1.0;
//     double theta_d = sqrt(pw.x * pw.x + pw.y * pw.y);

//     theta_d = Min(Max(-M_PI / 2., theta_d), M_PI / 2.);
//     if (theta_d > 1e-8)
//     {

//         double theta = theta_d;
//         double theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 = theta6 * theta2;
//         theta = theta_d / (1 + D[0] * theta2 + D[1] * theta4 + D[2] * theta6 + D[3] * theta8);
//         scale = tan(theta) / theta_d;
//     }

//     vec2 pu = make_vec2(pw.x * scale, pw.y * scale); //undistorted point

//     double _x = K_new[0][0] * pu.x + K_new[0][1] * pu.y + K_new[0][2];
//     double _y = K_new[1][0] * pu.x + K_new[1][1] * pu.y + K_new[1][2];
//     double _w = K_new[2][0] * pu.x + K_new[2][1] * pu.y + K_new[2][2];
//     // double _x = K[0][0] * pu.x + K[0][1] * pu.y + K[0][2];
//     // double _y = K[1][0] * pu.x + K[1][1] * pu.y + K[1][2];
//     // double _w = K[2][0] * pu.x + K[2][1] * pu.y + K[2][2];
    
//     /* reproject */
//     vec2 fi;
//     fi = make_vec2((float)(_x / _w), (float)(_y / _w));
//     fi.x /= (float)(SC_SES_W - 1); //ses? 원본이랑 무슨 차이?
//     fi.y /= (float)(SC_SES_H - 1);
//     //dst[i] = fi;
//     ///
//     // return fi;
    
//     cv::Point2f out3;
//     out3.x = fi.x * (float)SC_SES_W;
//     out3.y = fi.y * (float)SC_SES_H;

//     float x_ = Ext[0][0] * out3.x + Ext[0][1] * out3.y + Ext[0][2];
//     float y_ = Ext[1][0] * out3.x + Ext[1][1] * out3.y + Ext[1][2];
//     float w_ = Ext[2][0] * out3.x + Ext[2][1] * out3.y + Ext[2][2];

//     // 정규화
//     cv::Point2f out;
//     out.x = (x_ / w_ );
//     out.y = (y_ / w_ );

//     printf("out.X : %f m\n", out.x);
//     printf("out.Y : %f m\n", out.y);

//     return out;
// }
