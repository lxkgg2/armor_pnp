#pragma once

#include "types.hpp"
#include <string>
#include <vector>

// ============================================================
//  相机标定
//  用法：
//    1) 采集一批棋盘格照片，放 images/ 下
//    2) 调 runCalibration() 生成 calib.yml
//    3) 主程序用 loadCalibration() 读回来
// ============================================================

// 从一堆图片标定，结果写入 outputYml
// 返回 true 表示标定成功
bool runCalibration(const std::vector<std::string>& imagePaths,
                    cv::Size patternSize,      // 内角点数，例如 9x6
                    float squareSizeMm,        // 每个格子的边长 mm
                    const std::string& outputYml);

// 读取标定文件；顺带检查分辨率是否与 expectedSize 一致
bool loadCalibration(const std::string& ymlPath,
                     CameraParam& cam,
                     cv::Size expectedSize = cv::Size());

// 把角点从"带畸变的原图"转换到"理想针孔模型"下
// 之后 solvePnP 就可以传零畸变
std::vector<cv::Point2f> undistortPoints(const std::vector<cv::Point2f>& pts,
                                         const CameraParam& cam);

// 生成去畸变的查找表（每帧 remap 用，比每帧全场标定快）
void buildUndistortMap(const CameraParam& cam,
                       cv::Mat& map1, cv::Mat& map2);
