#include "../include/camera_calib.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <iomanip>

using namespace std;

// ============================================================
//  构建棋盘格的三维点
//  棋盘坐标系：Z=0 的平面，格点间距 squareSizeMm
//  点的顺序必须和 findChessboardCorners 的输出一致：
//  从左到右、从上到下逐行
// ============================================================
static vector<cv::Point3f> makeObjectPoints(cv::Size pattern, float squareMm)
{
    vector<cv::Point3f> obj;
    obj.reserve(pattern.width * pattern.height);
    for (int r = 0; r < pattern.height; ++r) {
        for (int c = 0; c < pattern.width; ++c) {
            obj.emplace_back(c * squareMm, r * squareMm, 0.0f);
        }
    }
    return obj;
}

bool runCalibration(const vector<string>& imagePaths,
                    cv::Size patternSize,
                    float squareSizeMm,
                    const string& outputYml)
{
    if (imagePaths.empty()) {
        cerr << "[标定] 没有输入图片\n";
        return false;
    }

    const vector<cv::Point3f> objTemplate = makeObjectPoints(patternSize, squareSizeMm);

    vector<vector<cv::Point3f>> objectPoints;   // 每张图的 3D 点
    vector<vector<cv::Point2f>> imagePoints;    // 每张图的 2D 点
    cv::Size imageSize;

    int used = 0;
    for (const auto& path : imagePaths) {
        cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            cerr << "[标定] 读不到图片: " << path << "\n";
            continue;
        }
        if (imageSize.empty()) {
            imageSize = img.size();
        } else if (img.size() != imageSize) {
            cerr << "[标定] 分辨率不一致，跳过: " << path << "\n";
            continue;
        }

        vector<cv::Point2f> corners;
        // 先用 SB 版本（扇形采样，对光照和模糊更鲁棒，且自带亚像素）
        bool found = cv::findChessboardCornersSB(
            img, patternSize, corners,
            cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);

        if (!found) {
            // 退回经典算法 + 手动亚像素
            found = cv::findChessboardCorners(
                img, patternSize, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
            if (found) {
                cv::cornerSubPix(
                    img, corners, cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
            }
        }

        if (!found) {
            cerr << "[标定] 没找到棋盘格: " << path << "\n";
            continue;
        }

        objectPoints.push_back(objTemplate);
        imagePoints.push_back(corners);
        ++used;
    }

    cout << "[标定] 成功使用 " << used << " 张图，分辨率 "
         << imageSize.width << "x" << imageSize.height << "\n";

    if (used < 3) {
        cerr << "[标定] 至少需要 3 张，建议 15~25 张不同姿态\n";
        return false;
    }

    // ---------------- 核心：标定 ----------------
    cv::Mat K, D;
    vector<cv::Mat> rvecs, tvecs;
    // 图少时高阶畸变容易乱飞，这里固定 k3=0、切向为 0，更稳
    int flags = cv::CALIB_FIX_K3 | cv::CALIB_ZERO_TANGENT_DIST;

    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                     K, D, rvecs, tvecs, flags,
                                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                                      100, 1e-6));

    cout << "[标定] RMS 重投影误差 = " << fixed << setprecision(4) << rms << " px\n";
    cout << "[标定] 内参 K =\n" << K << "\n";
    cout << "[标定] 畸变 D = " << D.reshape(1, 1) << "\n";

    // ---------------- 逐张检查，找出拍得差的图 ----------------
    // 这一步很重要：RMS 是平均值，个别烂图会被平均掉
    double worst = 0;
    int worstIdx = -1;
    for (size_t i = 0; i < objectPoints.size(); ++i) {
        vector<cv::Point2f> proj;
        cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], K, D, proj);
        double e = cv::norm(imagePoints[i], proj, cv::NORM_L2) / (double)proj.size();
        if (e > worst) { worst = e; worstIdx = (int)i; }
        if (e > 1.0) {
            cout << "  [警告] 第 " << i << " 张图误差偏大: "
                 << setprecision(3) << e << " px\n";
        }
    }
    cout << "[标定] 最差一张误差 " << setprecision(3) << worst
         << " px (索引 " << worstIdx << ")\n";

    // 自检：拍摄距离应该是合理的正数
    for (size_t i = 0; i < tvecs.size(); ++i) {
        double z = tvecs[i].at<double>(2, 0);
        if (z <= 0) {
            cerr << "  [警告] 第 " << i << " 张图解出的 Z<0，可能角点顺序错了\n";
        }
    }

    // ---------------- 保存 ----------------
    cv::FileStorage fs(outputYml, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        cerr << "[标定] 无法写入 " << outputYml << "\n";
        return false;
    }
    fs << "image_width"  << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "camera_matrix" << K;
    fs << "distortion_coefficients" << D;
    fs << "rms" << rms;
    fs.release();

    cout << "[标定] 已保存到 " << outputYml << "\n";
    return true;
}

bool loadCalibration(const string& ymlPath, CameraParam& cam, cv::Size expectedSize)
{
    cv::FileStorage fs(ymlPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "[标定] 打不开 " << ymlPath << "\n"
             << "        先运行 ./calibrate 生成标定文件\n";
        return false;
    }

    int w = 0, h = 0;
    fs["image_width"] >> w;
    fs["image_height"] >> h;
    fs["camera_matrix"] >> cam.K;
    fs["distortion_coefficients"] >> cam.D;
    cam.imageSize = cv::Size(w, h);

    if (cam.K.rows != 3 || cam.K.cols != 3) {
        cerr << "[标定] camera_matrix 格式不对\n";
        return false;
    }
    cam.K.convertTo(cam.K, CV_64F);
    cam.D.convertTo(cam.D, CV_64F);

    // 分辨率检查：这是最容易被忽略、后果最严重的一条
    if (expectedSize.width > 0 && cam.imageSize != expectedSize) {
        cerr << "[标定] 分辨率不匹配！\n"
             << "        标定时: " << cam.imageSize << "\n"
             << "        现在:   " << expectedSize << "\n"
             << "        内参是像素单位的，必须重新标定或把分辨率改回去\n";
        return false;
    }

    cout << "[标定] 载入成功，分辨率 " << cam.imageSize
         << "，fx=" << cam.K.at<double>(0, 0)
         << " fy=" << cam.K.at<double>(1, 1) << "\n";
    return true;
}

vector<cv::Point2f> undistortPoints(const vector<cv::Point2f>& pts, const CameraParam& cam)
{
    if (pts.empty()) return {};

    vector<cv::Point2f> out;
    // 注意：这里传了 K 作为新相机矩阵，所以输出仍是像素坐标
    cv::undistortPoints(pts, out, cam.K, cam.D, cv::noArray(), cam.K);
    return out;
}

void buildUndistortMap(const CameraParam& cam, cv::Mat& map1, cv::Mat& map2)
{
    // alpha=1 表示保留全部像素（会有黑边）；想裁掉黑边改成 0
    cv::Mat newK = cv::getOptimalNewCameraMatrix(
        cam.K, cam.D, cam.imageSize, 1.0, cam.imageSize, nullptr, false);

    cv::initUndistortRectifyMap(cam.K, cam.D, cv::noArray(),
                                newK, cam.imageSize, CV_32FC1, map1, map2);
}
