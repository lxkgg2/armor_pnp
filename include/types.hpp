#pragma once

// ============================================================
//  全项目共用的数据结构
//  单位约定：长度一律 mm，时间一律 s，角度一律 rad
//  改这个文件会影响所有模块，改之前先想清楚
// ============================================================

#include <opencv2/core.hpp>
#include <array>
#include <vector>
#include <string>

// ---------------- 相机参数 ----------------
struct CameraParam {
    cv::Mat K;            // 3x3 内参矩阵 (CV_64F)
    cv::Mat D;            // 1x5 畸变系数 [k1,k2,p1,p2,k3] (CV_64F)
    cv::Size imageSize;   // 标定时的分辨率，用来检查是否匹配

    bool valid() const { return !K.empty() && !D.empty(); }
};

// ---------------- 装甲板物理尺寸 ----------------
// 注意：这里的 width/height 必须和模型标注的 4 个点严格对应！
//
// 你的模型关键点含义（已实测验证）：
//   kp0 = 左灯条上端    kp1 = 左灯条下端
//   kp2 = 右灯条下端    kp3 = 右灯条上端
//
// 所以：
//   width  = kp0→kp3 的距离 = 两灯条中心间距 = 137 mm   ✓ 实测
//   height = kp0→kp1 的距离 = 单侧灯条可见长度 ≈ 52 mm  ✗ 不是外壳的 125mm！
//
// 用 125mm 会让 PnP 重投影误差飙到 15~69 像素（已实测），
// 因为模型的点根本不在外壳上下边缘上。
struct ArmorSize {
    float width  = 138.0f;   // mm，两灯条中心距（实测 137，拟合最优 140，取中间）
    float height = 52.0f;    // mm，单侧灯条可见长度（拟合得出，不是外壳 125！）
};

// ---------------- 装甲板类别 ----------------
// 索引与模型输出一致：0..5
enum class ArmorClass : int {
    B1 = 0, B2, B3, R1, R2, R3
};

inline const char* armorClassName(int id)
{
    static const char* names[] = {"B1", "B2", "B3", "R1", "R2", "R3"};
    if (id < 0 || id > 5) return "?";
    return names[id];
}

// 按类别返回板子尺寸
// 大装甲板（B1/R1）两灯条间距更宽，其余参数同理可推
inline ArmorSize armorSizeOf(int classId)
{
    ArmorSize s;
    if (classId == 0 || classId == 3) {   // B1 / R1 大装甲板
        s.width  = 225.0f;                // 两灯条中心距（大板，需实测确认）
        s.height = 52.0f;                 // 灯条可见长度，与大板一致
    } else {                              // B2 B3 R2 R3 小装甲板
        s.width  = 138.0f;                // 实测 137 / 拟合最优 140
        s.height = 52.0f;                 // 拟合最优
    }
    return s;
}

// ---------------- 一个装甲板的观测结果 ----------------
struct Armor {
    // 四个角点，顺序固定：左上 → 右上 → 右下 → 左下
    std::array<cv::Point2f, 4> corners{};

    // 灯条（传统检测器才有）
    bool hasBars = false;
    cv::RotatedRect leftBar{};
    cv::RotatedRect rightBar{};

    float confidence = 1.0f;   // 置信度
    int classId = -1;          // 0..5 对应 B1,B2,B3,R1,R2,R3
    std::string className;     // 类别名，方便显示
    cv::Rect2f box{};          // 检测框（调试用）
};

// ---------------- PnP 解算结果 ----------------
struct PoseResult {
    bool ok = false;

    cv::Vec3d rvec{0, 0, 0};   // 旋转向量（轴角），做运算前要转成矩阵
    cv::Vec3d tvec{0, 0, 0};   // 平移向量，单位 mm
    cv::Matx33d R = cv::Matx33d::eye();

    double distance = 0.0;     // |tvec|，mm
    double reprojErr = 0.0;    // 重投影误差，像素
    bool flipped = false;      // 是否被判定为"翻面"（兄弟解）
};

// ---------------- 卡尔曼输出 ----------------
struct TrackState {
    bool initialized = false;

    cv::Vec3d pos{0, 0, 0};     // 滤波后的位置 mm
    cv::Vec3d vel{0, 0, 0};     // 估计速度 mm/s
    cv::Vec3d pred{0, 0, 0};    // T 秒后的预测位置 mm

    double rawPos[3] = {0, 0, 0};   // 原始测量，用来对比
    bool hasMeasurement = false;    // 本帧是否有有效测量
};
