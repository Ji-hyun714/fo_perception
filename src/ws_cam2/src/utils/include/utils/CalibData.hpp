#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp> 
// #include <opencv2/fisheye.hpp> 
#include <stdexcept>           
#include <vector>
#include <cmath>
#include <iostream>


struct CalibData 
{
    enum class Model { PINHOLE, FISHEYE };

    // ==========================================
    // 1. Intrinsics (내부 파라미터)
    // ==========================================
    Model  model = Model::PINHOLE;
    cv::Size imageSize;      // 원본 해상도
    cv::Mat K;               // 3x3, RAW intrinsic
    cv::Mat D;               // 왜곡 계수 (PINHOLE: 1x5, FISHEYE: 1x4)

    // --- Rectified (왜곡 보정 후) ---
    cv::Mat Krect;           // 3x3, 보정된 가상의 Camera Matrix
    cv::Mat map1, map2;      // remap용 LUT

    // ==========================================
    // 2. Extrinsics (외부 파라미터)
    // ==========================================
    // 월드(지면) 좌표계 -> 카메라 좌표계
    cv::Mat R_w2c; // 3x3 회전 행렬
    cv::Mat t_w2c; // 3x1 이동 벡터
    
    // 카메라 -> 월드 (역변환용)
    cv::Mat R_c2w; 
    cv::Mat Cw;    // 카메라의 월드 상 위치

    // ==========================================
    // 3. Ground Homography (BEV 변환용)
    // ==========================================
    cv::Mat H_w2i_rect; // 지면(m) -> 픽셀(uv)
    cv::Mat H_i2w_rect; // 픽셀(uv) -> 지면(m)

    // ==========================================
    // 4. Factory Methods (생성 함수)
    // ==========================================
    
    // 일반 핀홀 카메라 (RadTan 모델)
    static CalibData MakePinhole(const cv::Size& imgSize,
                             const cv::Mat& K_, const cv::Mat& D_,
                             const cv::Mat& R_w2c_, const cv::Mat& t_w2c_)
    {
        CalibData c;
        c.model = Model::PINHOLE;
        c.imageSize = imgSize;
        c.K = K_.clone();
        c.D = D_.clone();
        // c.Krect = c.K.clone(); // 핀홀은 Krect를 K와 동일하게 유지하거나, 필요 시 getOptimalNewCameraMatrix 사용
        c.Krect = cv::getOptimalNewCameraMatrix(c.K, c.D, c.imageSize, 0.0, c.imageSize, 0);

        c.setExtrinsic(R_w2c_, t_w2c_);
        c.buildHomography();  
        return c;
    }

    // 어안 렌즈 (Fisheye 모델)
    // balance: 0.0(많이 잘림) ~ 1.0(검은 테두리 포함 전체 보임)
    static CalibData MakeFisheye(const cv::Size& imgSize,
                             const cv::Mat& K_, const cv::Mat& D_,
                             const cv::Mat& R_w2c_, const cv::Mat& t_w2c_,
                             double balance = 0.0)
    {
        CalibData c;
        c.model = Model::FISHEYE;
        c.imageSize = imgSize;
        c.K = K_.clone();
        c.D = D_.clone();

        // Fisheye -> Rectilinear 변환용 Krect 및 LUT 생성
        cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
            c.K, c.D, c.imageSize, R, c.Krect, balance, c.imageSize, 1.0);
        
        cv::fisheye::initUndistortRectifyMap(
            c.K, c.D, R, c.Krect, c.imageSize, CV_16SC2, c.map1, c.map2);

        c.setExtrinsic(R_w2c_, t_w2c_);
        c.buildHomography(); 
        return c;
    }

    // ==========================================
    // 5. Helper Methods
    // ==========================================

    // Extrinsic 설정 및 파생 변수 업데이트
    void setExtrinsic(const cv::Mat& Rw2c, const cv::Mat& tw2c)
    {
        R_w2c = Rw2c.clone();
        t_w2c = tw2c.clone();
        R_c2w = R_w2c.t();     // 회전 행렬의 역행렬은 전치 행렬
        Cw    = -R_c2w * t_w2c; // 카메라의 월드 좌표
    }

    // 지면(Z=0) <-> 이미지 간 호모그래피 행렬 계산
    void buildHomography()
    {
        // Projection Matrix P = K * [R|t]
        // 지면(Z=0) 가정 시, 3번째 열(r3)이 사라짐 -> 3x3 호모그래피가 됨
        // H = Krect * [r1 r2 t]
        
        cv::Mat M(3, 3, CV_64F);
        R_w2c.col(0).copyTo(M.col(0)); // r1
        R_w2c.col(1).copyTo(M.col(1)); // r2
        t_w2c.copyTo(M.col(2));        // t

        H_w2i_rect = Krect * M;
        
        // 정규화 (마지막 요소를 1로 맞춤)
        if (std::abs(H_w2i_rect.at<double>(2, 2)) > 1e-9) {
            H_w2i_rect /= H_w2i_rect.at<double>(2, 2);
        }
        
        H_i2w_rect = H_w2i_rect.inv();
    }

    // [핵심] 지면 좌표(m) -> 픽셀 좌표(uv) 변환
    inline cv::Point2d gndToPixRect(const cv::Point2d& XY) const
    {
        // [u, v, 1]^T = H * [X, Y, 1]^T
        cv::Mat Xw = (cv::Mat_<double>(3, 1) << XY.x, XY.y, 1.0);
        cv::Mat ph = H_w2i_rect * Xw;

        double w = ph.at<double>(2);
        if (std::abs(w) < 1e-12) return cv::Point2d(-1, -1); // 에러 처리 혹은 예외

        return { ph.at<double>(0) / w, ph.at<double>(1) / w };
    }

    // [핵심] 픽셀 좌표(uv) -> 지면 좌표(m) 변환
    inline cv::Point2d pixRectToGnd(const cv::Point2d& uv) const
    {
        // [X, Y, 1]^T = H_inv * [u, v, 1]^T
        cv::Mat p = (cv::Mat_<double>(3, 1) << uv.x, uv.y, 1.0);
        cv::Mat Xw = H_i2w_rect * p;

        double w = Xw.at<double>(2);
        if (std::abs(w) < 1e-12) return cv::Point2d(0, 0);

        return { Xw.at<double>(0) / w, Xw.at<double>(1) / w };
    }
};