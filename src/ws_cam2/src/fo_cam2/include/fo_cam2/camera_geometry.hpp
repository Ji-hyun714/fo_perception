#include "wafl3.h"
#include "autocal/SensorMath.h"

#include "opencv2/opencv.hpp"


/* variables */
#define SC_SES_W 512.0f
#define SC_SES_H 256.0f


/*  */
cv::Point2f convert_cam2gnd(cv::Point2f in);
cv::Point2f convert_gnd2cam(cv::Point2f in);

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <iomanip> 

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <iomanip> 

#define W_ORG 1280
#define H_ORG 720

struct CamCalib
{
    cv::Mat K; //3x3, CV_64F
    // cv::Mat K_rect; //3x3, CV_64F
    cv::Mat D; //1x5, CV_64F
    cv::Mat R_w2c; //3x3, CV_64F w2c from solvePnP
    cv::Mat t_w2c; //3x1, CV_64F w2c

    cv::Mat R_c2w; //3x3
    // cv::Mat t_c2w; //3x1

    cv::Mat Cw;    //camera center in world
    cv::Mat H_w2i; //homography world(ground) to image 
    cv::Mat H_i2w; //homography image to world(ground)

    /*  */

    static CamCalib Make(const cv::Mat& K_,    const cv::Mat& D_,
                         const cv::Mat& R_w2c, const cv::Mat& t_w2c )
    {
        CamCalib c;

        cv::Size image_size_ = cv::Size(W_ORG,H_ORG);

        c.K = K_.clone(); 
        c.D = D_.clone();
        c.R_w2c = R_w2c.clone(); 
        c.t_w2c = t_w2c.clone();
        // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        //                 c.K, c.D, image_size_, cv::Mat::eye(3,3,CV_64F), c.K_rect, 0.0 );
        c.R_c2w = c.R_w2c.t();
        c.Cw    = -c.R_w2c.t() * c.t_w2c;

        cv::Mat M(3,3,CV_64F);           // z=0 평면 상의 점에만 적용하는 호모그래피 
        c.R_w2c.col(0).copyTo(M.col(0)); // r1
        c.R_w2c.col(1).copyTo(M.col(1)); // r2
        c.t_w2c.copyTo(M.col(2));        // t

        c.H_w2i = c.K * M;
        c.H_i2w = c.H_w2i.inv();

        return c;
    }
};

inline cv::Point2d gndToPix(const cv::Point2d& XY, const CamCalib& c)
{
    cv::Mat Xw = (cv::Mat_<double>(3,1) << XY.x, XY.y, 1.0);

    cv::Mat ph = c.H_w2i * Xw;

    double w = ph.at<double>(2);

    if (std::abs(w) < 1e-12) throw std::runtime_error("gndToPix: w=0");

    // 여기서 다시 distort를 해야 원래 픽셀이랑 일치함
    // 애초에 카메라 드라이버 쪽에서 undistortion을 해서 /image_raw를 쏘면?
    return {ph.at<double>(0)/w, ph.at<double>(1)/w}; // 왜곡보정 안되어있음
}


inline cv::Point2d pixToGndUndist(const cv::Point2d& uv, const CamCalib& c) // 왜곡 보정 이미지 픽셀 -> 지면 좌표
{
    // 왜곡보정된 픽셀 좌표 그대로 homography 적용
    cv::Mat p_img = (cv::Mat_<double>(3,1) << uv.x, uv.y, 1.0);
    cv::Mat p_gnd = c.H_i2w * p_img; 

    double w = p_gnd.at<double>(2);
    if (std::abs(w) < 1e-9) throw std::runtime_error("pixToGndUndist: w is close to zero.");

    return {p_gnd.at<double>(0)/w, p_gnd.at<double>(1)/w};
}


inline CamCalib setManualCalib()
{
    // cv::Mat K = (cv::Mat_<double>(3, 3) << 
    //                 557.286481458488, 0, 645.434763202952,
    //                 0, 560.2662472618434, 422.5041216026116,
    //                 0, 0, 1
    //             );
    // cv::Mat D = (cv::Mat_<double>(1, 4) << 
    //             -0.04717653045611134, 0.08466535058959987, -0.1122427643930151, 0.02860302134167773
    //             ); 
    // cv::Mat R_w2c = (cv::Mat_<double>(3, 3) << 
    //             0.003380210197028666, -0.9996296606114867, -0.02700214444794445,
    //             0.0978710174402659, 0.02720336796941658, -0.9948272416436572,
    //             0.9951933672026914, 0.0007199978362965238, 0.09792672504931757
    //             );
    // cv::Mat t_w2c = (cv::Mat_<double>(3, 1) << 
    //             0.2034374452515109, 1.723653058144044, 2.369491982068624
    //             ); 
    
    /* 250912 */
    cv::Mat K = (cv::Mat_<double>(3, 3) << 
                    596.242804413754, 0, 640,
                    0, 606.212049838454, 360,
                    0.0, 0.0, 1.0
                );
    cv::Mat D = (cv::Mat_<double>(1, 4) << 
                    0.145233418368253, -0.448981613840834, 0.348017752004174, -0.0460532929321028
                ); 
    cv::Mat K_rect = (cv::Mat_<double>(3, 3) << 
                    532.9377761804633, 0.0, 1387.398374688271,
                    0.0, 535.7873514233846, 679.053415832703,
                    0.0, 0.0, 1.0
                );
    cv::Mat R_w2c = (cv::Mat_<double>(3, 3) << 
                    0.003380210197028666, -0.9996296606114867, -0.02700214444794445,
                    0.0978710174402659, 0.02720336796941658, -0.9948272416436572,
                    0.9951933672026914, 0.0007199978362965238, 0.09792672504931757
                );
    cv::Mat t_w2c = (cv::Mat_<double>(3, 1) << 
                0.2034374452515109, 1.723653058144044, 2.369491982068624
                ); 
                /**
                 * Cw    (Camera center in world ):
    [-2.941499929852141;
    0.3892818214645057;
    1.264777915212199]

    R_w2c (Rotation Matrix, World to Camera):
    [0.02006879253265631, -0.9997020415116865, -0.01389502657959448;
    0.1828656803129355, 0.01733366206390713, -0.9829850899800778;
    0.9829330529237322, 0.01718640034506079, 0.1831590595990568]

    t_w2c (Translation Vector, World to Camera):
    [0.4657723062176374;
    1.774409539061654;
    2.652951619441843]
                 */
    
    
    CamCalib calib = CamCalib::Make(K, D, R_w2c, t_w2c);

    return calib;
}

// CamCalib parseYAML2calib(const std::string yaml_path)
// {
//     CamCalib calib;
    

//     return calib;
// }