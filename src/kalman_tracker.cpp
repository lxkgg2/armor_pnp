#include "../include/kalman_tracker.hpp"
#include <cmath>
#include <algorithm>

using namespace std;

// ============================================================
//  初始化：用第一帧测量值当起点
//
//  两个细节很容易漏：
//   ① 速度设 0（还没有任何信息）
//   ② 协方差 P 要设"大"——表示"我对初值没信心"
//      如果沿用默认的单位阵，滤波器会以为自己已经知道得很准，
//      前几帧会几乎不理会测量，跟得特别迟钝。
// ============================================================
void ArmorTracker::init(const cv::Vec3d& z, const CameraParam& cam, const ArmorSize& size)
{
    for (int i = 0; i < 6; ++i) x_[i] = 0.0;
    x_[0] = z[0];
    x_[1] = z[1];
    x_[2] = z[2];
    // 速度保持 0

    P_ = cv::Matx<double, 6, 6>::zeros();
    // 位置：初始不确定度给 50mm 量级
    P_(0, 0) = P_(1, 1) = P_(2, 2) = 50.0 * 50.0;
    // 速度：完全不知道，给很大
    P_(3, 3) = P_(4, 4) = P_(5, 5) = 1000.0 * 1000.0;

    // H：只观测位置
    H_ = cv::Matx<double, 3, 6>::zeros();
    for (int i = 0; i < 3; ++i) H_(i, i) = 1.0;

    rawMeas_ = z;
    initialized_ = true;
    lostFrames_ = 0;

    // 先按 dt=1/30 建一次矩阵，避免第一帧 update 时用到未初始化的 Q/R
    buildMatrices(1.0 / 30.0, z, cam, size);
}

// ============================================================
//  构建 F / Q / R
//
//  F = [I  dt·I]
//      [0   I  ]
//
//  Q 用"连续白噪声加速度"模型，每轴独立：
//      [dt⁴/4  dt³/2]
//      [dt³/2  dt²  ] × σa²
//
//  R 是关键：纵向（z 方向）噪声 ∝ z²/(f·W)，
//  横向 ∝ z/f。所以三个轴必须给不同的值。
//  写成各向同性 R = σ²I 的话，要么横向被过度平滑，
//  要么纵向完全压不住。
// ============================================================
void ArmorTracker::buildMatrices(double dt, const cv::Vec3d& z,
                                 const CameraParam& cam, const ArmorSize& size)
{
    // ---- F ----
    F_ = cv::Matx<double, 6, 6>::eye();
    for (int i = 0; i < 3; ++i) F_(i, i + 3) = dt;

    // ---- Q ----
    const double sa2 = params_.sigmaAccel * params_.sigmaAccel;
    const double q11 = dt * dt * dt * dt / 4.0;
    const double q12 = dt * dt * dt / 2.0;
    const double q22 = dt * dt;

    Q_ = cv::Matx<double, 6, 6>::zeros();
    for (int i = 0; i < 3; ++i) {
        Q_(i, i)     = q11 * sa2;
        Q_(i, i + 3) = q12 * sa2;
        Q_(i + 3, i) = q12 * sa2;
        Q_(i + 3, i + 3) = q22 * sa2;
    }

    // ---- R：按距离自适应 ----
    const double fx = cam.K.at<double>(0, 0);
    const double fy = cam.K.at<double>(1, 1);
    const double W  = size.width;
    const double dist = std::max(1.0, std::fabs(z[2]));
    const double sp = params_.sigmaPixel;

    // 横向：一个像素对应 dist/f 毫米
    const double sx = dist / fx * sp;
    const double sy = dist / fy * sp;
    // 纵向：一个像素对应 dist²/(f·W) 毫米
    const double sz = dist * dist / (fx * W) * sp;

    R_ = cv::Matx<double, 3, 3>::zeros();
    R_(0, 0) = sx * sx;
    R_(1, 1) = sy * sy;
    R_(2, 2) = sz * sz;
}

// ============================================================
//  每帧更新
//
//  流程严格按卡尔曼五步：
//    预测  x⁻ = Fx ,  P⁻ = FPFᵀ + Q
//    新息  y  = z - Hx⁻,  S = HP⁻Hᵀ + R
//    增益  K  = P⁻HS⁻¹
//    更新  x  = x⁻ + Ky ,  P = (I - KH)P⁻
// ============================================================
void ArmorTracker::update(const cv::Vec3d& z, bool hasMeas,
                          const CameraParam& cam, const ArmorSize& size, double dt)
{
    // dt 兜底：程序卡顿或第一帧时可能异常
    if (!(dt > 1e-4 && dt < 1.0)) {
        dt = 1.0 / 30.0;
    }

    if (!initialized_) {
        if (hasMeas) init(z, cam, size);
        return;
    }

    buildMatrices(dt, hasMeas ? z : rawMeas_, cam, size);

    // ---- ① 预测 ----
    x_ = F_ * x_;
    // Use cv::Mat for matrix operations (Matx doesn't support * chain)
    cv::Mat Fm(F_), Pm(P_), Qm(Q_);
    cv::Mat Ft;
    cv::transpose(Fm, Ft);
    Pm = Fm * Pm * Ft + Qm;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            P_(r, c) = Pm.at<double>(r, c);

    // ---- ② 没有测量：只预测，直接返回 ----
    //  这就是"目标被遮挡时轨迹不断"的机制。
    //  注意：此时 P 会持续增大，表示不确定性在增加。
    if (!hasMeas) {
        ++lostFrames_;
        return;
    }

    // ---- ③ 新息 ----
    cv::Vec<double, 3> zz(z[0], z[1], z[2]);
    cv::Mat Hm(H_), Rm(R_);
    cv::Mat xm(x_);
    cv::Mat ym = cv::Mat(zz) - Hm * xm;

    // Compute S = H * P * H^T + R
    cv::Mat Ht;
    cv::transpose(Hm, Ht);
    cv::Mat Sm(Pm);
    cv::Mat S = Hm * Sm * Ht + Rm;

    // ---- ④ 增益 ----
    cv::Mat SinvM;
    if (cv::invert(S, SinvM, cv::DECOMP_SVD) <= 0) {
        ++lostFrames_;
        return;
    }

    cv::Mat Km = Pm * Ht * SinvM;

    // ---- ⑤ 更新 ----
    xm = xm + Km * ym;
    for (int i = 0; i < 6; ++i) x_[i] = xm.at<double>(i, 0);
    cv::Mat I6(cv::Matx<double,6,6>::eye());
    Pm = (I6 - Km * Hm) * Pm;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            P_(r, c) = Pm.at<double>(r, c);

    rawMeas_ = z;
    lostFrames_ = 0;
}

// ============================================================
//  解析式预测
//
//  匀速模型下：p(t+T) = p(t) + v(t)·T
//  这里故意不去调 predict()，因为那会真的推进滤波器状态，
//  把 statePost 污染掉（连续 predict 会让轨迹越跑越偏）。
// ============================================================
cv::Vec3d ArmorTracker::predictAfter(double T) const
{
    return cv::Vec3d(x_[0] + x_[3] * T,
                     x_[1] + x_[4] * T,
                     x_[2] + x_[5] * T);
}

TrackState ArmorTracker::state() const
{
    TrackState s;
    s.initialized = initialized_;
    s.pos  = cv::Vec3d(x_[0], x_[1], x_[2]);
    s.vel  = cv::Vec3d(x_[3], x_[4], x_[5]);
    s.pred = cv::Vec3d(x_[0] + x_[3] * params_.predictTime,
                       x_[1] + x_[4] * params_.predictTime,
                       x_[2] + x_[5] * params_.predictTime);
    s.rawPos[0] = rawMeas_[0];
    s.rawPos[1] = rawMeas_[1];
    s.rawPos[2] = rawMeas_[2];
    s.hasMeasurement = (lostFrames_ == 0);
    return s;
}

void ArmorTracker::reset()
{
    initialized_ = false;
    lostFrames_ = 0;
    for (int i = 0; i < 6; ++i) x_[i] = 0.0;
    P_ = cv::Matx<double, 6, 6>::eye();
}
