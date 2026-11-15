#pragma once

#include "types.hpp"
#include <vector>

// ============================================================
//  PnP 位姿解算
//
//  核心难点不是"调 solvePnP"，而是处理平面二义性：
//  同一张图有两个都讲得通的解，必须用物理约束 + 时间连续性挑。
// ============================================================

class PnPSolver {
public:
    explicit PnPSolver(const ArmorSize& size, const CameraParam& cam);

    // 设定板子的物理尺寸（换装甲板型号时调用）
    void setArmorSize(const ArmorSize& s) { size_ = s; rebuildObjectPoints(); }

    // 主入口：给四个角点（已按 左上/右上/右下/左下 排好），返回最优位姿
    PoseResult solve(const std::array<cv::Point2f, 4>& corners);

    // 只看不记状态：给一组角点，返回全部候选解（用于调试/分析）
    std::vector<PoseResult> solveAll(const std::array<cv::Point2f, 4>& corners) const;

    // 清空历史（比如目标丢失后重新捕获）
    void reset() { hasLast_ = false; }

private:
    void rebuildObjectPoints();

    // 从两个候选里挑一个更合理的
    bool chooseBest(const std::vector<PoseResult>& cands, PoseResult& out) const;

    // 物理约束：物体必须在相机前方，且板面朝向相机
    static bool physicallyValid(const PoseResult& p);

    // 重投影误差
    static double reprojectionError(const std::vector<cv::Point3f>& obj,
                                    const std::array<cv::Point2f, 4>& img,
                                    const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                                    const CameraParam& cam);

    ArmorSize size_;
    CameraParam cam_;
    std::vector<cv::Point3f> objPts_;   // 模型四点，原点在板中心

    bool hasLast_ = false;
    cv::Vec3d lastRvec_{0, 0, 0};
    cv::Vec3d lastTvec_{0, 0, 0};
};
