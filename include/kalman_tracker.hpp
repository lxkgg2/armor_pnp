#pragma once

#include "types.hpp"

// ============================================================
//  卡尔曼滤波（匀速 CV 模型）
//
//  状态  x = [x, y, z, vx, vy, vz]^T     (mm, mm/s)
//  测量  z = [x, y, z]^T                 (PnP 给的平移)
//
//  两个关键点：
//   ① dt 必须用真实时间差，不能假设 30fps
//   ② R 必须各向异性且随距离变化（纵向噪声 ∝ z²）
// ============================================================

class ArmorTracker {
public:
    struct Params {
        // 过程噪声：目标"可能有多不按匀速走"，单位 mm/s²
        // 机动性强（小陀螺、急停）就调大
        double sigmaAccel = 400.0;

        // 测量噪声基准：角点检测的像素噪声
        double sigmaPixel = 0.5;

        // 预测时长，秒
        double predictTime = 0.1;

        // 重投影误差超过这个值就不喂给滤波器
        double maxReprojErr = 3.0;

        // 连续多少帧没测量就认为目标丢失
        int maxLostFrames = 15;
    };

    ArmorTracker() = default;
    explicit ArmorTracker(const Params& p) : params_(p) {}

    // 第一帧：用测量值初始化
    void init(const cv::Vec3d& z, const CameraParam& cam, const ArmorSize& size);

    // 每帧调用。hasMeas=false 时只预测不修正
    void update(const cv::Vec3d& z, bool hasMeas,
                const CameraParam& cam, const ArmorSize& size, double dt);

    // 取当前状态
    TrackState state() const;

    // 解析式预测 T 秒后的位置（不污染滤波器状态）
    cv::Vec3d predictAfter(double T) const;

    // 目标丢失判断
    bool lost() const { return lostFrames_ > params_.maxLostFrames; }

    void reset();

    double lastReprojErr() const { return lastReproj_; }

private:
    void buildMatrices(double dt, const cv::Vec3d& z, const CameraParam& cam,
                       const ArmorSize& size);

    Params params_;

    cv::Matx<double, 6, 6> F_ = cv::Matx<double, 6, 6>::eye();
    cv::Matx<double, 6, 6> Q_ = cv::Matx<double, 6, 6>::zeros();
    cv::Matx<double, 3, 3> R_ = cv::Matx<double, 3, 3>::eye();
    cv::Matx<double, 3, 6> H_ = cv::Matx<double, 3, 6>::zeros();

    cv::Vec<double, 6> x_ = cv::Vec<double, 6>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    cv::Matx<double, 6, 6> P_ = cv::Matx<double, 6, 6>::eye(); // 协方差

    bool initialized_ = false;
    int lostFrames_ = 0;
    double lastReproj_ = 0.0;

    // 原始测量，用于对比显示
    cv::Vec3d rawMeas_{0, 0, 0};
};
