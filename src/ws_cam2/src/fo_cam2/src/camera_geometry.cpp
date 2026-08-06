#include "fo_cam2/camera_geometry.hpp"

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
const float Proj[3][3] = {
    { 593.108760f, -163.877548f, -149.724316f }, 
    { 353.417169f, 56.886443f, 266.075489f },
    { 2.883074f, 0.447859f, 1.000000f }
};

cv::Mat Proj_mat(3, 3, CV_32F, (void*)Proj);


const double D[4] = { -0.040784, -0.003428, -0.001554, 0.000751 };

int count = 1;          // 변화 가능
double f[2] = {};       // 필요하면 초기화
double c[2] = {};

int tangencialUndistort(
    float x_in,    ///< 입력 x 좌표, 0.0 ~ 1.0으로 normalize
    float y_in,    ///< 입력 y 좌표, 0.0 ~ 1.0으로 normalize
    float* x_out,  ///< 출력 x 좌표, 0.0 ~ 1.0으로 normalize
    float* y_out   ///< 출력 y 좌표, 0.0 ~ 1.0으로 normalize
)
{
    float p1 = 0.0f;
    float p2 = 0.0f;

    x_in -= 0.5f;
    y_in -= 0.5f;
    float r2;
    float x_corrected, y_corrected;

    r2 = (x_in * x_in) + (y_in * y_in);
    x_corrected = x_in + ((2 * (-p1) * x_in * y_in) + ((-p2) * (r2 + 2 * x_in * x_in)));
    y_corrected = y_in + (((-p1) * (r2 + 2 * y_in * y_in)) + (2 * (-p2) * x_in * y_in));

    x_in = x_corrected;
    y_in = y_corrected;

    r2 = (x_in * x_in) + (y_in * y_in);
    x_corrected = x_in + ((2 * (-p1 / 10.0f) * x_in * y_in) + ((-p2 / 10.0f) * (r2 + 2 * x_in * x_in)));
    y_corrected = y_in + (((-p1 / 10.0f) * (r2 + 2 * y_in * y_in)) + (2 * (-p2 / 10.0f) * x_in * y_in));

    *x_out = x_corrected + 0.5f;
    *y_out = y_corrected + 0.5f;

    return 0;
}

cv::Point2f convert_cam2gnd(cv::Point2f in) // 역투영
{   
    float w_org = 1280.0f;
    float h_org = 720.0f;

    f[0] = K[0][0];
    f[1] = K[1][1];
    c[0] = K[0][2];
    c[1] = K[1][2];

    /* normalize */   
    cv::Point2f in_norm;
    in_norm.x = in.x / (float)w_org;
    in_norm.y = in.y / (float)h_org;

    /* undistort */
    
    vec2 pi = {in_norm.x, in_norm.y};  // image point

/// tangencial 오차 보정
    tangencialUndistort(pi.x, pi.y, &pi.x, &pi.y);

    pi.x *= (float)(w_org - 1);
    pi.y *= (float)(h_org - 1); //?

    vec2 pw = make_vec2((pi.x - c[0]) / f[0], (pi.y - c[1]) / f[1]);      //정규화된 카메라좌표, Z=1 평면에 맺힌 점으로
    double scale = 1.0;
    double theta_d = sqrt(pw.x * pw.x + pw.y * pw.y);

    theta_d = Min(Max(-M_PI / 2., theta_d), M_PI / 2.);
    if (theta_d > 1e-8)
    {

        double theta = theta_d;
        double theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 = theta6 * theta2;
        theta = theta_d / (1 + D[0] * theta2 + D[1] * theta4 + D[2] * theta6 + D[3] * theta8);
        scale = tan(theta) / theta_d;
    }

    vec2 pu = make_vec2(pw.x * scale, pw.y * scale); //undistorted point

    double _x = K_new[0][0] * pu.x + K_new[0][1] * pu.y + K_new[0][2];
    double _y = K_new[1][0] * pu.x + K_new[1][1] * pu.y + K_new[1][2];
    double _w = K_new[2][0] * pu.x + K_new[2][1] * pu.y + K_new[2][2];
    // double _x = K[0][0] * pu.x + K[0][1] * pu.y + K[0][2];
    // double _y = K[1][0] * pu.x + K[1][1] * pu.y + K[1][2];
    // double _w = K[2][0] * pu.x + K[2][1] * pu.y + K[2][2];
    
    /* reproject */
    vec2 fi;
    fi = make_vec2((float)(_x / _w), (float)(_y / _w));
    fi.x /= (float)(SC_SES_W - 1); //ses? 원본이랑 무슨 차이?
    fi.y /= (float)(SC_SES_H - 1);
    //dst[i] = fi;
    ///
    // return fi;
    
    cv::Point2f out3;
    out3.x = fi.x * (float)SC_SES_W;
    out3.y = fi.y * (float)SC_SES_H;

    float x_ = Ext[0][0] * out3.x + Ext[0][1] * out3.y + Ext[0][2];
    float y_ = Ext[1][0] * out3.x + Ext[1][1] * out3.y + Ext[1][2];
    float w_ = Ext[2][0] * out3.x + Ext[2][1] * out3.y + Ext[2][2];

    // 정규화
    cv::Point2f out;
    out.x = (x_ / w_ );
    out.y = (y_ / w_ );

    printf("out.X : %f m\n", out.x);
    printf("out.Y : %f m\n", out.y);

    return out;
}

cv::Point2f convert_gnd2cam(cv::Point2f in)
{

}

// CamCalib setCamParameters()
// {
//     CamCalib calib;

//     cv::Mat K = (cv::Mat_<double>(3, 3) << 
//                     557.286481458488, 0, 645.434763202952,
//                     0, 560.2662472618434, 422.5041216026116,
//                     0, 0, 1
//                 );
//     cv::Mat D = (cv::Mat_<double>(1, 4) << 
//                 -0.04717653045611134, 0.08466535058959987, -0.1122427643930151, 0.02860302134167773
//                 ); 

// }