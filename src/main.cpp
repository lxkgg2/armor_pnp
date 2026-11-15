// ============================================================
//  main.cpp — 海康 MVS 工业相机实时 YOLO + PnP + 卡尔曼滤波
//
//  用法：
//    ./armor_pnp --model best.onnx
//    ./armor_pnp --model best.onnx --serial 你的相机序列号
//    ./armor_pnp --model best.onnx --calib calib.yml --width 1600
//
//  按 q 或 ESC 退出
// ============================================================

#include <MvCameraControl.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "types.hpp"
#include "pnp_solver.hpp"
#include "kalman_tracker.hpp"
#include "camera_calib.hpp"

using namespace std;

// ============================================================
//  一、MVS SDK 错误处理
//
//  MVS 的每个接口都返回 int 错误码，不抛异常。
//  这里包一层，把错误码翻译成中文，失败时直接抛异常。
// ============================================================

// 把常见错误码翻译成人话
static std::string MvName(unsigned int c)
{
    switch (c) {
    case MV_OK:          return "成功";
    case MV_E_HANDLE:    return "句柄无效";
    case MV_E_NODATA:    return "无数据(超时)";
    case MV_E_CALLORDER: return "调用顺序错";
    case MV_E_PARAMETER: return "参数错误";
    case MV_E_UNKNOW:    return "未知错误(设备被占用?)";
    default:             return "错误码";
    }
}

// 检查返回值，非 MV_OK 就抛异常（带错误码和中文说明）
static void MvOk(int r, const char* what)
{
    if (r == MV_OK) return;
    std::ostringstream o;
    o << "MVS失败[" << what << "]: 0x" << std::hex << (unsigned)r
      << " " << MvName((unsigned)r);
    throw std::runtime_error(o.str());
}

// 宏：自动把表达式文本作为 "what" 传进去
#define MVOK(x) MvOk((x), #x)

// ============================================================
//  二、字符串工具
//
//  MVS 的字符串字段是 unsigned char[N] 且不保证 0 结尾，
//  直接当 C 字符串用会读到垃圾。这几个重载负责安全截断。
// ============================================================

static std::string S(const char* p, size_t n)
{
    if (!p || !n) return "";
    size_t i = 0;
    while (i < n && p[i]) ++i;
    return std::string(p, i);
}
static std::string S(const unsigned char* p, size_t n)
{
    return S(reinterpret_cast<const char*>(p), n);
}
template <size_t N> static std::string S(const char (&a)[N]) { return S(a, N); }
template <size_t N> static std::string S(const unsigned char (&a)[N]) { return S(a, N); }

// ============================================================
//  三、Cam — 海康相机封装（RAII）
//
//  生命周期必须成对、反序：
//    Initialize   ↔ Finalize
//    CreateHandle ↔ DestroyHandle
//    OpenDevice   ↔ CloseDevice
//    StartGrabbing↔ StopGrabbing
//    GetImageBuffer ↔ FreeImageBuffer（每帧！）
// ============================================================
class Cam {
public:
    // 析构时反序释放，即使抛异常也保证清理
    ~Cam()
    {
        try {
            if (grabbing_) { MV_CC_StopGrabbing(h_); grabbing_ = false; }
            if (opened_)   { MV_CC_CloseDevice(h_);   opened_ = false; }
            if (h_)        { MV_CC_DestroyHandle(h_); h_ = nullptr; }
            if (inited_)   { MV_CC_Finalize();        inited_ = false; }
        } catch (...) {}
    }

    // 打开相机。sn 为空表示用第一台
    void Open(const std::string& sn)
    {
        MVOK(MV_CC_Initialize());
        inited_ = true;

        MV_CC_DEVICE_INFO_LIST ls{};
        MVOK(MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &ls));

        MV_CC_DEVICE_INFO* sel = nullptr;
        for (unsigned i = 0; i < ls.nDeviceNum; ++i) {
            auto* d = ls.pDeviceInfo[i];
            if (!d) continue;
            std::string s;
            if (d->nTLayerType == MV_USB_DEVICE)
                s = S(d->SpecialInfo.stUsb3VInfo.chSerialNumber,
                      sizeof(d->SpecialInfo.stUsb3VInfo.chSerialNumber));
            else if (d->nTLayerType == MV_GIGE_DEVICE)
                s = S(d->SpecialInfo.stGigEInfo.chSerialNumber,
                      sizeof(d->SpecialInfo.stGigEInfo.chSerialNumber));
            if (sn.empty() || s == sn) { sel = d; break; }
        }
        if (!sel) throw std::runtime_error("找不到相机。检查连接或关闭 MVS 客户端");

        MVOK(MV_CC_CreateHandle(&h_, sel));
        MVOK(MV_CC_OpenDevice(h_));
        opened_ = true;
    }

    // 读曝光/增益的取值范围（用于初始化滑块）
    void Range(float& emin, float& emax, float& gmin, float& gmax)
    {
        MVCC_FLOATVALUE v{};
        emin = emax = gmin = gmax = 0;
        if (MV_CC_GetFloatValue(h_, "ExposureTime", &v) == MV_OK) { emin = v.fMin; emax = v.fMax; }
        if (MV_CC_GetFloatValue(h_, "Gain", &v) == MV_OK)         { gmin = v.fMin; gmax = v.fMax; }
    }

    // 配置采集：连续模式、关触发、关自动曝光
    void Config(float exp, float gain)
    {
        MVOK(MV_CC_SetEnumValueByString(h_, "AcquisitionMode", "Continuous"));
        MVOK(MV_CC_SetEnumValueByString(h_, "TriggerMode", "Off"));
        MVOK(MV_CC_SetEnumValueByString(h_, "ExposureAuto", "Off"));
        MVOK(MV_CC_SetEnumValueByString(h_, "GainAuto", "Off"));
        MV_CC_SetEnumValueByString(h_, "ExposureMode", "Timed");

        // 优先用 BGR8（省掉转换），不支持则退回 BayerRG8
        int ret = MV_CC_SetEnumValue(h_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
        if (ret != MV_OK) {
            MVOK(MV_CC_SetEnumValue(h_, "PixelFormat", PixelType_Gvsp_BayerRG8));
        }

        // 先读范围再设，越界会静默失败
        MVCC_FLOATVALUE v{};
        MVOK(MV_CC_GetFloatValue(h_, "ExposureTime", &v));
        if (exp < v.fMin || exp > v.fMax) throw std::runtime_error("曝光超范围");
        MVOK(MV_CC_SetFloatValue(h_, "ExposureTime", exp));

        MVOK(MV_CC_GetFloatValue(h_, "Gain", &v));
        if (gain >= v.fMin && gain <= v.fMax) MV_CC_SetFloatValue(h_, "Gain", gain);
    }

    void SetExp(float us)  { if (h_) MV_CC_SetFloatValue(h_, "ExposureTime", us); }
    void SetGain(float db) { if (h_) MV_CC_SetFloatValue(h_, "Gain", db); }

    void Start() { if (!grabbing_) { MVOK(MV_CC_StartGrabbing(h_)); grabbing_ = true; } }
    void Stop()  { if (grabbing_ && h_) { MV_CC_StopGrabbing(h_); grabbing_ = false; } }

    // ========================================================
    //  取一帧并转成 cv::Mat(BGR)
    //
    //  三个坑：
    //   ① 缓冲区是"借"的，必须 FreeImageBuffer 归还
    //   ② 一行可能有对齐填充，step 要用 nFrameLen/height 算
    //   ③ Bayer 有四种排列，要用对应的转换码
    // ========================================================
    bool Grab(cv::Mat& bgr, int timeoutMs)
    {
        MV_FRAME_OUT f{};
        int r = MV_CC_GetImageBuffer(h_, &f, timeoutMs);
        if ((unsigned)r == MV_E_NODATA) return false;   // 超时，正常现象
        MVOK(r);

        // RAII 保证归还缓冲区，即使下面抛异常
        struct G {
            void* h; MV_FRAME_OUT* f;
            ~G() { MV_CC_FreeImageBuffer(h, f); }
        } g{h_, &f};

        const auto& in = f.stFrameInfo;
        const int w = (int)in.nWidth, hgt = (int)in.nHeight;
        if (!f.pBufAddr || w <= 0 || hgt <= 0) return false;

        const unsigned t = in.enPixelType;
        const bool mono = (t == PixelType_Gvsp_Mono8);
        const bool bgr8 = (t == PixelType_Gvsp_BGR8_Packed);
        const bool rgb8 = (t == PixelType_Gvsp_RGB8_Packed);
        const bool bayer = (t == PixelType_Gvsp_BayerRG8 || t == PixelType_Gvsp_BayerGB8 ||
                            t == PixelType_Gvsp_BayerGR8 || t == PixelType_Gvsp_BayerBG8);
        if (!mono && !bgr8 && !rgb8 && !bayer) return false;

        // 真实步长：一行可能比 w*bpp 更长（有对齐填充）
        const size_t bpp = (bgr8 || rgb8) ? 3 : 1;
        size_t step = (size_t)w * bpp;
        if (in.nFrameLen >= step * (size_t)hgt) {
            size_t c = in.nFrameLen / (size_t)hgt;
            if (c >= step) step = c;
        }

        if (bgr8) {
            // 视图 + clone：原缓冲区马上要归还，必须深拷贝
            bgr = cv::Mat(hgt, w, CV_8UC3, f.pBufAddr, step).clone();
            return true;
        }
        if (rgb8) {
            cv::Mat v(hgt, w, CV_8UC3, f.pBufAddr, step);
            cv::cvtColor(v, bgr, cv::COLOR_RGB2BGR);
            return true;
        }
        if (mono) {
            cv::Mat v(hgt, w, CV_8UC1, f.pBufAddr, step);
            cv::cvtColor(v, bgr, cv::COLOR_GRAY2BGR);
            return true;
        }

        // Bayer：四种排列对应不同转换码，选错会导致红蓝颠倒
        cv::Mat v(hgt, w, CV_8UC1, f.pBufAddr, step);
        int code = cv::COLOR_BayerRG2BGR;
        if (t == PixelType_Gvsp_BayerGB8)      code = cv::COLOR_BayerGB2BGR;
        else if (t == PixelType_Gvsp_BayerGR8) code = cv::COLOR_BayerGR2BGR;
        else if (t == PixelType_Gvsp_BayerBG8) code = cv::COLOR_BayerBG2BGR;

        cv::cvtColor(v, bgr, code);
        return true;
    }

private:
    void* h_ = nullptr;
    bool inited_ = false, opened_ = false, grabbing_ = false;
};

// ============================================================
//  四、Det / Opt — 检测结果与推理参数
// ============================================================

struct Det {
    cv::Rect2f box;
    int id = -1;
    float conf = 0;
    std::string name;
    std::vector<cv::Point2f> kp;   // 4 个关键点，逆时针顺序
};

struct Opt {
    float conf = 0.25f;
    float iou = 0.45f;
    int size = 640;
    bool letterbox = true;
    int numKpt = 4;
    int kptDims = 2;
    std::vector<std::string> names;
};

// ============================================================
//  五、Detector — YOLO 推理
//
//  兼容两种输出布局：
//    (1, 18, 8400)  原始候选，需 NMS
//    (1, 8400, 18)  转置版
// ============================================================
class Detector {
public:
    void Load(const std::string& p)
    {
        net_ = cv::dnn::readNetFromONNX(p);
        if (net_.empty()) throw std::runtime_error("ONNX 加载失败");
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }

    std::vector<Det> Run(const cv::Mat& bgr, const Opt& o)
    {
        // ---- letterbox：等比缩放 + 灰边填充 ----
        float sx, sy, px, py;
        cv::Mat in;
        if (o.letterbox) {
            float s = std::min((float)o.size / bgr.cols, (float)o.size / bgr.rows);
            int nw = (int)std::lround(bgr.cols * s), nh = (int)std::lround(bgr.rows * s);
            cv::Mat t; cv::resize(bgr, t, cv::Size(nw, nh));
            int dw = o.size - nw, dh = o.size - nh;
            px = dw / 2.f; py = dh / 2.f; sx = sy = s;
            cv::copyMakeBorder(t, in, dh / 2, dh - dh / 2, dw / 2, dw - dw / 2,
                               cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
        } else {
            cv::resize(bgr, in, cv::Size(o.size, o.size));
            sx = (float)o.size / bgr.cols; sy = (float)o.size / bgr.rows; px = py = 0;
        }

        // swapRB=true：相机给 BGR，模型要 RGB。反了颜色特征全错
        cv::Mat blob;
        cv::dnn::blobFromImage(in, blob, 1 / 255.0, cv::Size(o.size, o.size),
                               cv::Scalar(), true, false, CV_32F);
        net_.setInput(blob);
        cv::Mat out = net_.forward();

        // ---- 判定布局：是 (C,N) 还是 (N,C) ----
        int C = 18, N = 8400;
        bool isNC = false;
        if (out.dims == 3) {
            if (out.size[2] == 18) isNC = true;      // (1, 8400, 18)
        } else if (out.dims == 4) {
            if (out.size[3] == 18) isNC = true;
        }

        const float* d = (const float*)out.data;
        const int kv = o.numKpt * o.kptDims;
        int nc = o.names.empty() ? (C - 4 - kv) : (int)o.names.size();
        if (nc <= 0) nc = C - 4;
        if (nc <= 0) throw std::runtime_error("无法推导类别数");

        std::vector<cv::Rect2f> bs;
        std::vector<float> sc;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> kps;

        // ---- 逐 anchor 解码 ----
        for (int i = 0; i < N; ++i) {
            // 类别分数取最大
            int bi = -1; float bv = 0;
            for (int c = 0; c < nc; ++c) {
                float v = isNC ? d[i * C + (4 + c)] : d[(4 + c) * N + i];
                if (v > bv) { bv = v; bi = c; }
            }
            if (bv < o.conf || bi < 0) continue;

            float cx = isNC ? d[i * C + 0] : d[0 * N + i];
            float cy = isNC ? d[i * C + 1] : d[1 * N + i];
            float bw = isNC ? d[i * C + 2] : d[2 * N + i];
            float bh = isNC ? d[i * C + 3] : d[3 * N + i];

            // 还原坐标：先减填充，再除缩放
            float x1 = (cx - bw / 2 - px) / sx, y1 = (cy - bh / 2 - py) / sy;
            float x2 = (cx + bw / 2 - px) / sx, y2 = (cy + bh / 2 - py) / sy;
            x1 = std::max(0.f, std::min(x1, (float)bgr.cols - 1));
            y1 = std::max(0.f, std::min(y1, (float)bgr.rows - 1));
            x2 = std::max(0.f, std::min(x2, (float)bgr.cols - 1));
            y2 = std::max(0.f, std::min(y2, (float)bgr.rows - 1));
            if (x2 <= x1 || y2 <= y1) continue;

            bs.emplace_back(x1, y1, x2 - x1, y2 - y1);
            sc.push_back(bv); ids.push_back(bi);

            // 关键点
            std::vector<cv::Point2f> kp;
            int ks = 4 + nc;
            if (kv > 0 && C - ks >= kv) {
                for (int k = 0; k < o.numKpt; ++k) {
                    float kx = isNC ? d[i * C + (ks + k * o.kptDims + 0)]
                                    : d[(ks + k * o.kptDims + 0) * N + i];
                    float ky = isNC ? d[i * C + (ks + k * o.kptDims + 1)]
                                    : d[(ks + k * o.kptDims + 1) * N + i];
                    kp.emplace_back((kx - px) / sx, (ky - py) / sy);
                }
            }
            kps.push_back(kp);
        }

        // ---- NMS：按置信度降序，抑制重叠框 ----
        std::vector<int> idx(bs.size());
        for (size_t i = 0; i < idx.size(); ++i) idx[i] = (int)i;
        std::sort(idx.begin(), idx.end(),
                  [&](int a, int b) { return sc[a] > sc[b]; });

        std::vector<char> dead(bs.size(), 0);
        std::vector<int> keep;
        for (size_t i = 0; i < idx.size(); ++i) {
            if (dead[idx[i]]) continue;
            keep.push_back(idx[i]);
            for (size_t j = i + 1; j < idx.size(); ++j) {
                if (dead[idx[j]]) continue;
                const cv::Rect2f& A = bs[idx[i]];
                const cv::Rect2f& B = bs[idx[j]];
                float ix = std::max(A.x, B.x), iy = std::max(A.y, B.y);
                float ax = std::min(A.x + A.width, B.x + B.width);
                float ay = std::min(A.y + A.height, B.y + B.height);
                float iw = ax - ix, ih = ay - iy;
                if (iw <= 0 || ih <= 0) continue;
                float inter = iw * ih;
                float uni = A.width * A.height + B.width * B.height - inter;
                if (uni > 0 && inter / uni > o.iou) dead[idx[j]] = 1;
            }
        }

        std::vector<Det> r;
        for (int k : keep) {
            Det t;
            t.box = bs[k]; t.conf = sc[k]; t.id = ids[k]; t.kp = kps[k];
            t.name = (t.id >= 0 && t.id < (int)o.names.size())
                     ? o.names[t.id] : ("class" + std::to_string(t.id));
            r.push_back(t);
        }
        return r;
    }

    static void Draw(cv::Mat& img, const std::vector<Det>& ds, const Opt&)
    {
        static const cv::Scalar pal[] = {
            {255,0,0},{0,255,0},{0,0,255},{255,255,0},{255,0,255},{0,255,255}};
        for (const auto& d : ds) {
            cv::Scalar c = pal[(d.id < 0 ? 0 : d.id) % 6];
            cv::Rect r((int)d.box.x, (int)d.box.y, (int)d.box.width, (int)d.box.height);
            cv::rectangle(img, r, c, 2);
            std::string lb = d.name + " " + cv::format("%.2f", d.conf);
            cv::putText(img, lb, cv::Point(r.x, r.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, c, 1, cv::LINE_AA);
            for (size_t k = 0; k < d.kp.size(); ++k) {
                cv::Point p((int)d.kp[k].x, (int)d.kp[k].y);
                cv::circle(img, p, 4, cv::Scalar(0,255,0), cv::FILLED);
            }
        }
    }

private:
    cv::dnn::Net net_;
};

// ============================================================
//  六、全局 UI 状态
//
//  OpenCV 的滑动条回调是 C 风格函数指针，不能捕获局部变量，
//  只能用全局变量传状态。
// ============================================================
static int   g_conf = 25, g_iou = 45, g_exp = 1000, g_gain = 0;
static bool  g_pause = false;
static float g_emin = 100, g_emax = 100000, g_gmin = 0, g_gmax = 24;
static Cam*  g_cam = nullptr;

static float EUs() { return g_emin + (g_emax - g_emin) * (g_exp / 1000.f); }
static float GDb() { return g_gmin + (g_gmax - g_gmin) * (g_gain / 100.f); }
static void OnExp(int, void*)  { if (g_cam) g_cam->SetExp(EUs()); }
static void OnGain(int, void*) { if (g_cam) g_cam->SetGain(GDb()); }

// ============================================================
//  七、main
// ============================================================
int main(int argc, char** argv)
{
    // ---- 参数解析 ----
    std::string model, serial, calibPath = "calib.yml",
                cls = "B1,B2,B3,R1,R2,R3";
    int winW = 1280;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto nx = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(a + " 缺参数");
            return argv[++i];
        };
        if (a == "--model")      model = nx();
        else if (a == "--serial") serial = nx();
        else if (a == "--classes") cls = nx();
        else if (a == "--calib")  calibPath = nx();
        else if (a == "--width")  winW = std::stoi(nx());
        else if (a == "--help") {
            std::cout << "用法: " << argv[0] << " --model best.onnx [选项]\n"
                      << "  --serial  <sn>   相机序列号（空=第一台）\n"
                      << "  --calib   <path> 标定文件（默认 calib.yml）\n"
                      << "  --classes <list> 类别名，逗号分隔\n"
                      << "  --width   <px>   窗口宽度\n";
            return 0;
        }
    }

    try {
        if (model.empty()) {
            std::cerr << "缺少 --model 参数。用法: ./armor_pnp --model best.onnx\n";
            return 1;
        }

        Opt o;
        {
            std::stringstream ss(cls);
            std::string t;
            while (std::getline(ss, t, ',')) if (!t.empty()) o.names.push_back(t);
        }

        // ---- 标定 ----
        CameraParam cam;
        if (!loadCalibration(calibPath, cam)) {
            std::cerr << "标定文件加载失败: " << calibPath << "\n"
                      << "  用 --calib 指定路径，或先运行 ./calibrate 生成\n";
            return -1;
        }

        // ---- PnP 与卡尔曼 ----
        ArmorSize asize;
        asize.width = 138.0f;    // 两灯条中心距
        asize.height = 52.0f;    // 单侧灯条可见长度（不是外壳 125！）
        PnPSolver pnp(asize, cam);

        ArmorTracker::Params tparams;
        ArmorTracker tracker(tparams);

        // ---- 模型 ----
        Detector det;
        det.Load(model);

        // ---- 相机 ----
        Cam cam_dev;
        g_cam = &cam_dev;
        cam_dev.Open(serial);
        cam_dev.Range(g_emin, g_emax, g_gmin, g_gmax);
        if (g_emax <= g_emin) { g_emin = 100; g_emax = 100000; }

        float initExp = std::max(g_emin, std::min(g_emax, (g_emin + g_emax) * 0.02f));
        cam_dev.Config(initExp, g_gmin);
        cam_dev.Start();
        g_exp = (int)((initExp - g_emin) / (g_emax - g_emin) * 1000.f);

        // ---- 窗口与滑动条 ----
        std::string W = "MVS + YOLO + PnP + Kalman";
        cv::namedWindow(W, cv::WINDOW_NORMAL);
        cv::resizeWindow(W, winW, (int)(winW * 0.62));
        cv::createTrackbar("Conf", W, &g_conf, 100, nullptr);
        cv::createTrackbar("IoU",  W, &g_iou,  100, nullptr);
        cv::createTrackbar("Exposure", W, &g_exp, 1000, OnExp);
        cv::createTrackbar("Gain",     W, &g_gain, 100, OnGain);

        // ---- 主循环 ----
        int lastClass = -1;
        auto tPrev = std::chrono::steady_clock::now();

        while (true) {
            o.conf = g_conf / 100.f;
            o.iou  = g_iou / 100.f;

            cv::Mat bgr;
            if (!cam_dev.Grab(bgr, 1000) || bgr.empty()) continue;

            // 真实时间差，不能假设固定帧率
            auto tNow = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(tNow - tPrev).count();
            tPrev = tNow;

            // ---- 推理 ----
            std::vector<Det> ds = det.Run(bgr, o);
            cv::Mat show = bgr.clone();
            Detector::Draw(show, ds, o);

            // ---- 选置信度最高且关键点齐全的 ----
            int pick = -1;
            float bestConf = -1;
            for (size_t i = 0; i < ds.size(); ++i) {
                if (ds[i].conf > bestConf && ds[i].kp.size() == 4) {
                    bestConf = ds[i].conf;
                    pick = (int)i;
                }
            }

            PoseResult bestPose;
            bool gotPose = false;

            if (pick != -1) {
                const auto& d = ds[pick];

                // 模型输出逆时针（左上、左下、右下、右上）
                // PnP 需顺时针（左上、右上、右下、左下）→ {0,3,2,1}
                std::array<cv::Point2f, 4> corners;
                corners[0] = d.kp[0];
                corners[1] = d.kp[3];
                corners[2] = d.kp[2];
                corners[3] = d.kp[1];

                // 类别变化 → 切换板子尺寸（大/小板差 87mm）
                if (d.id != lastClass) {
                    lastClass = d.id;
                    ArmorSize s = armorSizeOf(d.id);
                    pnp.setArmorSize(s);
                    std::cout << "[信息] 切换尺寸: " << d.name
                              << " width=" << s.width << "mm\n";
                }

                bestPose = pnp.solve(corners);
                if (bestPose.ok && bestPose.reprojErr <= tparams.maxReprojErr) {
                    gotPose = true;
                }
            }

            // ---- 卡尔曼 ----
            ArmorSize curSize = armorSizeOf(lastClass == -1 ? 1 : lastClass);
            if (gotPose) {
                // 首次必须 init，否则 update 内部会自动 init
                tracker.update(bestPose.tvec, true, cam, curSize, dt);
            } else {
                tracker.update(cv::Vec3d(0, 0, 0), false, cam, curSize, dt);
            }

            // 目标长时间丢失 → 重置
            if (tracker.lost()) {
                tracker.reset();
                pnp.reset();
                lastClass = -1;
            }

            auto st = tracker.state();

            // ---- 可视化 ----
            if (gotPose) {
                cv::Mat rv = (cv::Mat_<double>(3,1)
                              << bestPose.rvec[0], bestPose.rvec[1], bestPose.rvec[2]);
                cv::Mat tv = (cv::Mat_<double>(3,1)
                              << bestPose.tvec[0], bestPose.tvec[1], bestPose.tvec[2]);
                cv::drawFrameAxes(show, cam.K, cam.D, rv, tv, 80.0f);

                cv::putText(show,
                    cv::format("RAW: X=%.1f Y=%.1f Z=%.1f | Err=%.1fpx",
                               bestPose.tvec[0], bestPose.tvec[1],
                               bestPose.tvec[2], bestPose.reprojErr),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0,255,0), 2);
            }

            if (st.initialized) {
                cv::putText(show,
                    cv::format("KF  : X=%.1f Y=%.1f Z=%.1f | Dist=%.2fm",
                               st.pos[0], st.pos[1], st.pos[2],
                               cv::norm(st.pos)/1000.0),
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0,255,255), 2);

                cv::putText(show,
                    cv::format("PRED(0.1s): X=%.1f Y=%.1f Z=%.1f",
                               st.pred[0], st.pred[1], st.pred[2]),
                    cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255,255,0), 2);

                if (gotPose) {
                    double jitter = cv::norm(st.pos - bestPose.tvec);
                    cv::putText(show, cv::format("Jitter: %.1f mm", jitter),
                        cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0,128,255), 2);
                }
            } else {
                cv::putText(show, "No Track", cv::Point(10, 60),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);
            }

            cv::imshow(W, show);
            int k = cv::waitKey(1);
            if (k == 27 || k == 'q') break;
        }

        cam_dev.Stop();
        cv::destroyAllWindows();
    }
    catch (const std::exception& e) {
        std::cerr << "\n[错误] " << e.what() << "\n";
        return -1;
    }

    return 0;
}
