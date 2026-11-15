// ============================================================
//  calibrate.cpp — 棋盘格标定工具
//
//  用法：
//    ./calibrate <图片目录> [列数 行数 格子mm]
//  例：
//    ./calibrate calibration_images 9 6 25
//
//  9 6 是【内角点】数，不是格子数。
//  生成 calib.yml，主程序直接读。
// ============================================================

#include "camera_calib.hpp"

#include <opencv2/core/utils/filesystem.hpp>
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <unistd.h>

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "用法: " << argv[0] << " <图片目录> [列数 行数 格子mm]\n\n"
             << "  列数/行数是【内角点】数量，不是格子数。\n"
             << "  例如 10x7 的格子对应 9 6。\n\n"
             << "示例:\n"
             << "  ./calibrate calibration_images 9 6 25\n\n"
             << "CLion 里运行:\n"
             << "  Program arguments 填: calibration_images 9 6 25\n"
             << "  Working directory 填: $ProjectFileDir$\n";
        return 1;
    }

    const string dir = argv[1];
    int cols    = (argc > 2) ? stoi(argv[2]) : 9;
    int rows    = (argc > 3) ? stoi(argv[3]) : 6;
    float square = (argc > 4) ? stof(argv[4]) : 25.0f;

    // 打印当前目录，路径出错时能立刻定位
    {
        char cwd[1024] = {0};
        if (getcwd(cwd, sizeof(cwd))) {
            cout << "[信息] 当前工作目录: " << cwd << "\n"
                 << "[信息] 图片目录参数: " << dir << "\n\n";
        }
    }

    // ---- 收集图片 ----
    vector<string> images;
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec)) {
        cerr << "目录不存在: " << dir << "\n"
             << "  检查 Working directory 是否为 $ProjectFileDir$\n"
             << "  或改用绝对路径\n";
        return 1;
    }

    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (!entry.is_regular_file()) continue;
        string s = entry.path().string();
        string low = s;
        transform(low.begin(), low.end(), low.begin(), ::tolower);
        if (low.size() > 4 &&
            (low.substr(low.size() - 4) == ".png" ||
             low.substr(low.size() - 4) == ".jpg" ||
             low.substr(low.size() - 4) == ".bmp" ||
             low.substr(low.size() - 4) == ".pgm" ||
             (low.size() > 5 && low.substr(low.size() - 5) == ".jpeg"))) {
            images.push_back(s);
        }
    }
    sort(images.begin(), images.end());

    cout << "找到 " << images.size() << " 张图片\n"
         << "棋盘格内角点: " << cols << " x " << rows
         << "，格子边长 " << square << " mm\n\n";

    if (images.size() < 3) {
        cerr << "图片太少。至少 3 张，建议 15~25 张不同姿态。\n";
        return 1;
    }

    bool ok = runCalibration(images, cv::Size(cols, rows), square, "calib.yml");
    return ok ? 0 : 1;
}
