#pragma once
#include <opencv2/opencv.hpp>
#include <stdexcept>

struct Calib
{
    enum class Model { PINHOLE, FISHEYE };

    // --- Intrinsics (RAW domain) ---
    Model  model = Model::PINHOLE;
    cv::Size imageSize;      // RAW 이미지 해상도
    cv::Mat K;               // 3x3, RAW intrinsic
    cv::Mat D;               // PINHOLE: 1x5 (radtan), FISHEYE: 1x4

    // --- Rectified intrinsics (RECT domain = 언디스토션 후 가상의 pinhole) ---
    cv::Mat Krect;           // 3x3, RECT intrinsic (PINHOLE이면 Krect=K)
    cv::Mat map1, map2;      // RAW->RECT remap(double-buffer). PINHOLE이면 비움.

    // --- Extrinsics (월드=지면 좌표계 → 카메라) ---
    cv::Mat R_w2c; // 3x3
    cv::Mat t_w2c; // 3x1
    cv::Mat R_c2w; // 3x3
    cv::Mat Cw;    // 3x1 (camera center in world)

    // --- Ground-plane homography (RECT domain) ---
    cv::Mat H_w2i_rect; // [X,Y,1]^T -> RECT 픽셀
    cv::Mat H_i2w_rect; // RECT 픽셀 -> [X,Y,1]^T

    // ====== 팩토리 ======

    // 일반 pinhole(rad-tan) 모델 사용 시
    static Calib MakePinhole(const cv::Size& imgSize,
                                const cv::Mat& K_, const cv::Mat& D_,
                                const cv::Mat& R_w2c_, const cv::Mat& t_w2c_)
    {
        Calib c;
        c.model = Model::PINHOLE;
        c.imageSize = imgSize;
        c.K = K_.clone();  c.D = D_.clone();
        c.Krect = c.K.clone();

        c.setExtrinsic(R_w2c_, t_w2c_);
        c.buildHomography();  // Krect 기반

        return c;
    }

    // 광각/어안(≈150°)은 fisheye 모델 권장
    // balance∈[0,1]: 0은 더 크롭, 1은 최대 시야. 보통 0~0.3이면 PnP/BEV가 안정적
    static Calib MakeFisheye(const cv::Size& imgSize,
                                const cv::Mat& K_, const cv::Mat& D_,
                                const cv::Mat& R_w2c_, const cv::Mat& t_w2c_,
                                double balance = 0.0)
    {
        Calib c;
        c.model = Model::FISHEYE;
        c.imageSize = imgSize;
        c.K = K_.clone();  c.D = D_.clone();

        // fisheye → RECT용 Krect 및 리맵 맵 생성
        cv::Mat R = cv::Mat::eye(3,3,CV_64F);
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
            c.K, c.D, c.imageSize, R, c.Krect, balance, c.imageSize, 1.0);
        cv::fisheye::initUndistortRectifyMap(
            c.K, c.D, R, c.Krect, c.imageSize, CV_16SC2, c.map1, c.map2);

        c.setExtrinsic(R_w2c_, t_w2c_);
        c.buildHomography();  // Krect 기반

        return c;
    }

    // Extrinsic 교체/미세조정 후 다시 호출
    void setExtrinsic(const cv::Mat& Rw2c, const cv::Mat& tw2c)
    {
        R_w2c = Rw2c.clone();
        t_w2c = tw2c.clone();
        R_c2w = R_w2c.t();
        Cw    = -R_c2w * t_w2c;
    }

    // Ground-plane(Z=0) 호모그래피(RECT용) 구성
    void buildHomography()
    {
        // M = [r1 r2 t] (3x3)
        cv::Mat M(3,3,CV_64F);
        R_w2c.col(0).copyTo(M.col(0));
        R_w2c.col(1).copyTo(M.col(1));
        t_w2c.copyTo(M.col(2));

        H_w2i_rect = Krect * M;              // scale 자유
        H_w2i_rect /= H_w2i_rect.at<double>(2,2);
        H_i2w_rect = H_w2i_rect.inv();
    }

    // ====== 투영/역투영 유틸 (RECT 도메인 권장) ======

    // 지면(X,Y,0) → RECT 픽셀(u,v)
    inline cv::Point2d gndToPixRect(const cv::Point2d& XY) const
    {
        cv::Mat Xw = (cv::Mat_<double>(3,1) << XY.x, XY.y, 1.0);
        cv::Mat ph = H_w2i_rect * Xw;
        const double w = ph.at<double>(2);
        if (std::abs(w) < 1e-12) throw std::runtime_error("gndToPixRect: w=0");
        return { ph.at<double>(0)/w, ph.at<double>(1)/w };
    }

    // RECT 픽셀(u,v) → 지면(X,Y,0)
    inline cv::Point2d pixRectToGnd(const cv::Point2d& uv) const
    {
        cv::Mat p = (cv::Mat_<double>(3,1) << uv.x, uv.y, 1.0);
        cv::Mat Xw = H_i2w_rect * p;
        const double w = Xw.at<double>(2);
        if (std::abs(w) < 1e-12) throw std::runtime_error("pixRectToGnd: w≈0");
        return { Xw.at<double>(0)/w, Xw.at<double>(1)/w };
    }

    // 일반 3D 포인트 RAW 투영(검증용): projectPoints(K,D,R,t)
    inline bool worldToImageRAW(const cv::Point3d& Pw, cv::Point2d& uv_raw) const
    {
        std::vector<cv::Point3d> obj{Pw};
        std::vector<cv::Point2d> img;
        cv::Mat rvec; cv::Rodrigues(R_w2c, rvec);
        if (model == Model::PINHOLE)
            cv::projectPoints(obj, rvec, t_w2c, K, D, img);
        else {
            // fisheye RAW 투영이 꼭 필요할 때는 전용 함수를 써야 하나,
            // 보통은 RECT로 처리하는 것을 권장.
            std::vector<cv::Point2f> proj;
            cv::fisheye::projectPoints(std::vector<cv::Point3f>{
                cv::Point3f((float)Pw.x,(float)Pw.y,(float)Pw.z)}, proj, rvec, t_w2c, K, D);
            img.assign(proj.begin(), proj.end());
        }
        if (img.empty()) return false;
        uv_raw = img[0]; return true;
    }

    // Prect = Krect * [R|t] (3x4) 제공(디버그 또는 OpenGL 등)
    inline cv::Mat projectionMatrixRect() const
    {
        cv::Mat Rt; cv::hconcat(std::vector<cv::Mat>{R_w2c, t_w2c}, Rt); // 3x4
        return Krect * Rt; // 3x4
    }
};
