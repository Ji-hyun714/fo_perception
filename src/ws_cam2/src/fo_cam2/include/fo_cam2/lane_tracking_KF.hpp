#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <utility>

// 상태벡터: [ y0, theta_5, theta_15, theta_25, W ]
// 좌표계: x(전방+), y(좌측+), 모델: y(x)
struct state_vec {
    double y0;        // x=0 에서 중심선 y (좌우 위치)
    double th5;       // x=5m  지점의 방향각 (rad)
    double th15;      // x=15m 지점의 방향각 (rad)
    double th25;      // x=25m 지점의 방향각 (rad)
    double width;     // 차선 폭 W (m)
};

// ---- Cubic y(x) = a x^3 + b x^2 + c x + d ----
struct Cubic {
    double a=0, b=0, c=0, d=0;

    inline double eval(double x) const {
        return ((a*x + b)*x + c)*x + d;
    }
    // dy/dx
    inline double slope(double x) const {
        return (3.0*a*x + 2.0*b)*x + c;
    }
};

// ----------------- 중심선 3차식 복원 -----------------
// 제약:
//   y(0)=d = y0
//   y'(5)  = tan(th5)
//   y'(15) = tan(th15)
//   y'(25) = tan(th25)
// 미지수는 a,b,c (d는 y0)
inline Cubic centerCubicFromState(const state_vec& sv) {
    const double x1=5.0, x2=15.0, x3=25.0; 
    const double s1=std::tan(sv.th5);
    const double s2=std::tan(sv.th15);
    const double s3=std::tan(sv.th25);

    Eigen::Matrix3d A;
    A << 3*x1*x1, 2*x1, 1,
         3*x2*x2, 2*x2, 1,
         3*x3*x3, 2*x3, 1;
    Eigen::Vector3d b; b << s1, s2, s3;

    Eigen::Vector3d abc = A.colPivHouseholderQr().solve(b);

    Cubic f;
    f.a = abc(0);
    f.b = abc(1);
    f.c = abc(2);
    f.d = sv.y0; // y(0)=d
    return f;
}

// ----------------- 3차 최소제곱 피팅 -----------------
// 입력 샘플 (x_i, y_i)로 y ≈ a x^3 + b x^2 + c x + d 피팅
inline Cubic fitCubicLeastSquares(const std::vector<double>& xs,
                                  const std::vector<double>& ys) {
    const int N = static_cast<int>(xs.size());
    Eigen::MatrixXd M(N,4);
    Eigen::VectorXd v(N);
    for (int i=0;i<N;++i) {
        const double x = xs[i];
        M(i,0) = x*x*x;
        M(i,1) = x*x;
        M(i,2) = x;
        M(i,3) = 1.0;
        v(i)   = ys[i];
    }
    Eigen::VectorXd sol = M.colPivHouseholderQr().solve(v);
    Cubic f; f.a=sol(0); f.b=sol(1); f.c=sol(2); f.d=sol(3);
    return f;
}

struct LanePolys {
    Cubic center;
    Cubic left;
    Cubic right;
};

// ----------------- 좌/우 차선 산출 -----------------
// 중심선 y_c(x)의 각 x에서 법선으로 ±W/2 이동한 (x',y') 샘플을 만들고,
// 그 샘플들로 좌/우 차선을 다시 3차로 피팅한다.
//
// 주의: 법선 이동은 x, y 모두 바뀐다.
//  - 접선 t = (1, y'), 법선 n = (-y', 1) / sqrt(1+y'^2)
//  - (x', y') = (x, y) ± (W/2) * n
// 이후 (x',y')를 독립변수 x' 기준으로 다시 y(x') 3차 피팅.
inline LanePolys makeLanePolysFromState(const State5& st,
                                        double x_min=0.0,
                                        double x_max=35.0,
                                        double x_step=0.25) {
    LanePolys out;
    out.center = centerCubicFromState(st);

    std::vector<double> xsL, ysL, xsR, ysR;

    for (double x = x_min; x <= x_max; x += x_step) {
        const double y  = out.center.eval(x);
        const double yp = out.center.slope(x);            // y' = dy/dx
        const double denom = std::sqrt(1.0 + yp*yp);
        const double nx = -yp / denom;                    // 법선 x성분
        const double ny =  1.0 / denom;                   // 법선 y성분
        const double off = 0.5 * st.width;

        // 좌(+) / 우(-) : y축 좌측이 +이므로, +방향은 좌차선
        const double xL = x + off * nx;
        const double yL = y + off * ny;
        const double xR = x - off * nx;
        const double yR = y - off * ny;

        xsL.push_back(xL); ysL.push_back(yL);
        xsR.push_back(xR); ysR.push_back(yR);
    }

    out.left  = fitCubicLeastSquares(xsL, ysL);
    out.right = fitCubicLeastSquares(xsR, ysR);
    return out;
}
