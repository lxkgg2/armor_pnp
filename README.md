# armor_pnp

> 海康 MVS 工业相机 + YOLO 装甲板检测 + PnP 位姿解算 + 卡尔曼滤波
>
> 猎隼战队算法组第五次作业

从相机采集到三维位置估计、再到运动状态预测的完整链路。

---

## 目录

- [项目简介](#项目简介)
- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [标定流程](#标定流程)
- [目录结构](#目录结构)
- [代码详解](#代码详解)
  - [types.hpp — 数据结构](#typeshpp--数据结构)
  - [camera_calib — 相机标定](#camera_calib--相机标定)
  - [main.cpp / Cam — 海康相机封装](#maincpp--cam--海康相机封装)
  - [main.cpp / Detector — YOLO 推理](#maincpp--detector--yolo-推理)
  - [pnp_solver — 位姿解算](#pnp_solver--位姿解算)
  - [kalman_tracker — 卡尔曼滤波](#kalman_tracker--卡尔曼滤波)
  - [main — 主循环](#main--主循环)
- [关键设计说明](#关键设计说明)
- [常见问题](#常见问题)
- [性能测试](#性能测试)

---

## 项目简介

### 完整流程

```
海康相机采集 (MVS SDK)
      ↓
YOLO26n-pose 检测 (ONNX + OpenCV DNN)
      ↓
4 个关键点 → 顺序重排（逆时针→顺时针）
      ↓
PnP 位姿解算（含平面二义性处理）
      ↓
卡尔曼滤波（匀速 CV 模型）
      ↓
位置 / 速度 / 短时预测
```

### 技术栈

| 组件 | 版本 | 用途 |
|---|---|---|
| C++ | 17 | 核心语言 |
| OpenCV | 4.x | 图像处理、DNN 推理、PnP |
| 海康 MVS SDK | latest | 工业相机驱动 |
| CMake | 3.16+ | 构建系统 |
| YOLO26n-pose | ONNX | 装甲板 + 4 关键点检测 |

### 模型输出说明

- 输入：`1 × 3 × 640 × 640`
- 输出：`1 × 18 × 8400`（one-to-many 头，**需外部 NMS**）
- 18 = 4 框 + 6 类 + 4×2 关键点
- 类别：`B1 B2 B3 R1 R2 R3`
- 关键点顺序：**逆时针**（左上 → 左下 → 右下 → 右上）

---

## 环境要求

### 系统

- Ubuntu 20.04 / 22.04 / 24.04
- 海康 USB3.0 或 GigE 工业相机

### 依赖安装

```bash
# 基础工具
sudo apt update
sudo apt install -y build-essential cmake

# OpenCV（需要 dnn 和 calib3d 模块）
sudo apt install -y libopencv-dev

# 确认版本
pkg-config --modversion opencv4
```

### 海康 MVS SDK

从[海康机器人官网](https://www.hikrobotics.com/cn/machinevision/service/download)下载 Linux 版 MVS，

```bash
# 解压后的安装脚本
sudo ./MVS-2.x.x_x86_64_20240101.run
# 默认装到 /opt/MVS
```

验证：
```bash
ls /opt/MVS/include/MvCameraControl.h
ls /opt/MVS/lib/64/libMvCameraControl.so
```

---

## 快速开始

### 1. 编译

```bash
cd armor_pnp
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

如果 MVS 装在非默认路径：
```bash
cmake .. -DMVS_ROOT=/your/mvs/path
```

### 2. 准备模型

把 `best.onnx` 放到工程根目录（和 `calib.yml` 同级）。

### 3. 运行

```bash
./armor_pnp --model best.onnx
```

常用参数：

| 参数 | 说明 | 默认 |
|---|---|---|
| `--model` | ONNX 模型路径（**必填**） | - |
| `--serial` | 相机序列号 | 第一台 |
| `--calib` | 标定文件 | `calib.yml` |
| `--classes` | 类别名，逗号分隔 | `B1,B2,B3,R1,R2,R3` |
| `--width` | 窗口宽度 | 1280 |

示例：
```bash
./armor_pnp --model best.onnx --serial 00C12345678 --width 1600
```

### 4. 界面操作

| 控件 | 作用 |
|---|---|
| **Conf** 滑块 | 置信度阈值 (0~1) |
| **IoU** 滑块 | NMS 抑制阈值 |
| **Exposure** 滑块 | 曝光时间（映射到相机实际范围） |
| **Gain** 滑块 | 增益 |
| `q` / `ESC` | 退出 |

画面上显示：

```
RAW: X=852.3 Y=-12.1 Z=2874.5 | Err=1.8px   ← PnP 原始测量（会抖）
KF : X=850.1 Y=-10.8 Z=2876.2 | Dist=2.88m  ← 滤波后（平滑）
PRED(0.1s): X=850.1 Y=-10.8 Z=2875.4        ← 预测位置
Jitter: 3.4 mm                               ← 滤波纠正了多少
```

---

## 标定流程

### 为什么必须标定

PnP 需要相机内参（焦距、主点）才能把像素反推成三维坐标。
**不标定的话距离绝对值和比例都是错的。**

### 1. 采集棋盘格照片

拍 **15~25 张**，放进 `calibration_images/`。

**必须满足**：

| 要求 | 原因 |
|---|---|
| 15~25 张 | 少于 15 张，k3 畸变系数会被噪声带乱 |
| **拍满画面四个角** | 畸变只在边缘可观测，中间拍再多也没用 |
| 姿态要变：左右 ±30°、上下 ±20°、再转 90° | 一张正对的图约束不够 |
| 距离要有远近 | 否则焦距和距离强耦合 |
| **别用视频抽帧** | 相邻帧几乎一样，等于只拍了一张 |

### 2. 运行标定

```bash
cd build
./calibrate ../calibration_images 9 6 25
```

参数：`图片目录 内角点列数 内角点行数 格子边长(mm)`

> **`9 6` 是内角点数，不是格子数。** 10×7 的格子板填 `9 6`。

### 3. 判断质量

```
[标定] 成功使用 18 张图，分辨率 1440x1080
[标定] RMS 重投影误差 = 0.2341 px
[标定] 已保存到 calib.yml
```

| RMS | 判断 |
|---|---|
| < 0.5 px | ✅ 可用 |
| 0.5 ~ 1.5 px | ⚠️ 勉强，建议删掉误差大的图重跑 |
| > 1.5 px | ❌ 不合格，重拍 |

**看到 `[警告] 第 N 张图误差偏大`** → 删掉索引 N 那张图（从 0 开始）重跑。

### 4. 验证距离

卷尺量 **2.000 m**，板子正对相机，看画面上的 `Dist`：

- 1.95 ~ 2.05 m → ✅ 正确
- 差固定倍数 → 检查 `ArmorSize` 的 width/height
- 数量级完全不对 → 标定或内参有问题

---

## 目录结构

```
armor_pnp/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── calib.yml                  # 标定结果
├── best.onnx                  # 模型（较大，可单独下载）
├── include/
│   ├── types.hpp              # 数据结构、尺寸定义
│   ├── camera_calib.hpp       # 标定接口
│   ├── pnp_solver.hpp         # PnP 接口
│   └── kalman_tracker.hpp     # 卡尔曼接口
├── src/
│   ├── main.cpp               # 主程序 + 相机封装 + YOLO 推理
│   ├── camera_calib.cpp       # 标定实现
│   ├── pnp_solver.cpp         # PnP 实现
│   └── kalman_tracker.cpp     # 卡尔曼实现
└── tools/
    └── calibrate.cpp          # 标定工具入口
```

---

## 代码详解

下面按文件顺序，逐个函数讲清楚：**它为什么存在、参数怎么填、写错会怎样。**

---

### `types.hpp` — 数据结构

全项目共用的定义。改这里会影响所有模块。

#### `struct CameraParam`

```cpp
struct CameraParam {
    cv::Mat K;            // 3x3 内参矩阵
    cv::Mat D;            // 1x5 畸变系数 [k1,k2,p1,p2,k3]
    cv::Size imageSize;   // 标定时的分辨率
};
```

- `K` = `[[fx,0,cx],[0,fy,cy],[0,0,1]]`，单位是**像素**
- `D` 顺序固定 `[k1,k2,p1,p2,k3]`，**顺序错会导致边缘角点误差爆炸**
- `imageSize` 必须记住：内参是像素单位的，分辨率一变全废

#### `struct ArmorSize` ★ 最容易搞错的地方

```cpp
struct ArmorSize {
    float width  = 138.0f;   // 两灯条中心距
    float height = 52.0f;    // 单侧灯条可见长度
};
```

**为什么 height 不是 125mm？**

模型 4 个关键点的实际含义（放大截图逐像素确认过）：

```
kp0 = 左灯条上端 ┐
kp1 = 左灯条下端 ┘  → 这两个点之间 = 灯条可见长度 ≈ 52mm
kp2 = 右灯条下端
kp3 = 右灯条上端
```

所以：
- `width` = `kp0→kp3`（两灯条之间）= **138mm**
- `height` = `kp0→kp1`（单根灯条）= **52mm**

**用 125mm（外壳高度）会怎样？**

| height 设定 | 重投影误差 |
|---|---|
| 125mm（外壳） | **28.8 px** ❌ |
| 55mm | 5.0 px |
| 53mm | 3.4 px |
| **52mm（最优）** | **3.1 px** ✅ |

误差差 10 倍，PnP 结果完全不可信。

#### `armorSizeOf(classId)`

```cpp
if (classId == 0 || classId == 3) {  // B1 / R1 大装甲板
    s.width = 225.0f;
} else {                              // 其余小装甲板
    s.width = 138.0f;
}
```

大板宽 225、小板 138，差 **87mm**。用错的话距离会系统性差 60%+。

---

### `camera_calib` — 相机标定

#### `runCalibration(...)`

```cpp
double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                 K, D, rvecs, tvecs, flags,
                                 TermCriteria(EPS + COUNT, 100, 1e-6));
```

**参数**：

| 参数 | 说明 |
|---|---|
| `objectPoints` | 每张图的 3D 点（棋盘坐标系，Z=0） |
| `imagePoints` | 每张图检测到的 2D 角点 |
| `imageSize` | **必须和实际图像一致**，否则主点错 |
| 返回 `rms` | RMS 重投影误差（像素） |

**用的 flags**：

```cpp
int flags = cv::CALIB_FIX_K3 | cv::CALIB_ZERO_TANGENT_DIST;
```

- `CALIB_FIX_K3`：固定 `k3=0`。图少于 15 张时 k3 极易被噪声带飞
- `CALIB_ZERO_TANGENT_DIST`：切向畸变设 0。正常镜头切向很小

图拍得足够好（>20 张）可以去掉这两个约束追求精度，**但先保守**。

**函数内部还做了三件检查**：

1. **逐张算误差** —— 因为 RMS 是平均值，一张烂图会被平均掉
```cpp
cv::projectPoints(obj, rvecs[i], tvecs[i], K, D, proj);
double e = cv::norm(imagePoints[i], proj) / proj.size();
if (e > 1.0) cout << "[警告] 第 " << i << " 张图误差偏大\n";
```

2. **检查拍摄距离** —— Z 为负说明角点顺序错
```cpp
if (tvecs[i].at<double>(2,0) <= 0)
    cerr << "第 " << i << " 张图 Z<0，可能角点顺序错了\n";
```

3. **保存时带分辨率** —— 加载时才能校验

#### `loadCalibration(...)` ★ 最重要的检查

```cpp
if (expectedSize.width > 0 && cam.imageSize != expectedSize) {
    cerr << "[标定] 分辨率不匹配！\n";
    return false;
}
```

内参是像素单位的。标定时 1440 宽、运行时 1920 宽，`fx` 和 `cx` 全错，距离会系统性偏差——**而且程序不会报任何错**。

#### `undistortPoints(...)`

```cpp
cv::undistortPoints(pts, out, cam.K, cam.D, cv::noArray(), cam.K);
```

**最后一个参数传 `cam.K` 很关键**：表示输出仍按原内参投回像素坐标系。
传 `noArray()` 的话输出会变成归一化坐标（除以焦距），PnP 全错。

---

### `main.cpp / Cam` — 海康相机封装

#### RAII 生命周期

MVS SDK 要求**成对调用、反序释放**：

```
MV_CC_Initialize        ↔  MV_CC_Finalize
MV_CC_CreateHandle      ↔  MV_CC_DestroyHandle
MV_CC_OpenDevice        ↔  MV_CC_CloseDevice
MV_CC_StartGrabbing     ↔  MV_CC_StopGrabbing
MV_CC_GetImageBuffer    ↔  MV_CC_FreeImageBuffer   ← 每帧！
```

析构函数全部 try-catch 包住，即使异常也保证清理。

#### `MvOk()` / `MVOK` 宏

```cpp
#define MVOK(x) MvOk((x), #x)
```

MVS 接口返回 int 错误码不抛异常，这层包装把它变成异常，并翻译成中文：

```cpp
if (r == MV_OK) return;
throw std::runtime_error("MVS失败[" + what + "]: 0x" + 十六进制码 + 中文名);
```

#### `S()` 字符串重载

MVS 的字符串字段是 `unsigned char[N]` 且**不保证 0 结尾**，直接当 C 字符串用会读到垃圾。

```cpp
static std::string S(const char* p, size_t n) {
    size_t i = 0;
    while (i < n && p[i]) ++i;   // 手动找结束或到边界
    return std::string(p, i);
}
```

#### `Open(sn)` — 枚举并打开

遍历 `MV_CC_EnumDevices` 的结果，按序列号匹配（sn 为空取第一台）。

USB 和 GigE 相机的序列号在**不同的联合体成员**里：
```cpp
if (d->nTLayerType == MV_USB_DEVICE)
    s = S(d->SpecialInfo.stUsb3VInfo.chSerialNumber, ...);
else if (d->nTLayerType == MV_GIGE_DEVICE)
    s = S(d->SpecialInfo.stGigEInfo.chSerialNumber, ...);
```

#### `Config(exp, gain)` — 参数设置

```cpp
void Config(float exp, float gain) {
    MV_CC_SetEnumValueByString(h_, "AcquisitionMode", "Continuous");
    MV_CC_SetEnumValueByString(h_, "TriggerMode", "Off");
    MV_CC_SetEnumValueByString(h_, "ExposureAuto", "Off");
    ...
    // 先尝试 BGR8，不支持再退回 BayerRG8
    int ret = MV_CC_SetEnumValue(h_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
    if (ret != MV_OK) {
        MVOK(MV_CC_SetEnumValue(h_, "PixelFormat", PixelType_Gvsp_BayerRG8));
    }
```

**为什么优先 BGR8**：省掉每帧的 Bayer 插值（`cvtColor`），能快 2~3ms。

**设置曝光前必须先读范围**：
```cpp
MV_CC_GetFloatValue(h_, "ExposureTime", &v);
if (exp < v.fMin || exp > v.fMax) throw std::runtime_error("曝光超范围");
MV_CC_SetFloatValue(h_, "ExposureTime", exp);
```
越界会**静默失败**，不报错但没生效。

#### `Grab(bgr, timeoutMs)` ★ 取帧，三个坑

**坑 1：缓冲区是"借"的**

```cpp
struct G {
    void* h; MV_FRAME_OUT* f;
    ~G() { MV_CC_FreeImageBuffer(h, f); }
} g{h_, &f};
```

`GetImageBuffer` 从 SDK 池子里借内存，**必须归还**。不还 → 池子耗尽 → 程序卡死在 `GetImageBuffer`。

用 RAII 保证即使抛异常也归还。

**坑 2：行步长有对齐填充**

```cpp
size_t step = (size_t)w * bpp;
if (in.nFrameLen >= step * hgt) {
    size_t c = in.nFrameLen / hgt;
    if (c >= step) step = c;   // 真实步长可能更长
}
```

不用真实步长，图像会**斜**（每行错位累积）。

**坑 3：视图要 clone**

```cpp
bgr = cv::Mat(hgt, w, CV_8UC3, f.pBufAddr, step).clone();
```

`cv::Mat` 包装的是 SDK 那块内存，函数返回后 `FreeImageBuffer` 会释放它。**不 clone 就是悬空指针**。

**Bayer 排列**：四种格式对应不同转换码，选错会导致红蓝颠倒：

```cpp
int code = cv::COLOR_BayerRG2BGR;
if (t == PixelType_Gvsp_BayerGB8)      code = cv::COLOR_BayerGB2BGR;
else if (t == PixelType_Gvsp_BayerGR8) code = cv::COLOR_BayerGR2BGR;
else if (t == PixelType_Gvsp_BayerBG8) code = cv::COLOR_BayerBG2BGR;
```

---

### `main.cpp / Detector` — YOLO 推理

#### `Load(path)`

```cpp
net_ = cv::dnn::readNetFromONNX(p);
net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
```

**为什么用 CPU**：Ubuntu 上 `libopencv-dev` 默认不带 CUDA 支持。
640×640 的 nano 模型 CPU 也能跑到 20~30ms。

#### `Run(bgr, o)` ★ 主流程

**① letterbox**（等比缩放 + 灰边）

```cpp
float s = std::min((float)o.size / bgr.cols, (float)o.size / bgr.rows);
int nw = lround(bgr.cols * s), nh = lround(bgr.rows * s);
int dw = o.size - nw, dh = o.size - nh;
px = dw / 2.f; py = dh / 2.f;
cv::copyMakeBorder(t, in, dh/2, dh-dh/2, dw/2, dw-dw/2,
                   cv::BORDER_CONSTANT, cv::Scalar(114,114,114));
```

**为什么不能直接 resize**：把 16:9 拉成 1:1，圆形变椭圆，模型认不出。

**记住三个量**：`s`（缩放比）、`px py`（填充量），解码要用。

**② blobFromImage**

```cpp
cv::dnn::blobFromImage(in, blob, 1/255.0, cv::Size(size,size),
                       cv::Scalar(),
                       true,    // ← swapRB 必须 true
                       false,   // ← crop 必须 false
                       CV_32F);
```

`swapRB=true`：相机给 BGR，模型要 RGB。**反了颜色特征全错，检测率暴跌，但不报错**。

**③ 判定布局**

```cpp
bool isNC = false;
if (out.dims == 3) {
    if (out.size[2] == 18) isNC = true;    // (1, 8400, 18)
}
```

支持 `(1,18,8400)` 和 `(1,8400,18)` 两种，通过判断最后一维是不是 18 确定。

**④ 逐 anchor 解码**

```cpp
for (int i = 0; i < N; ++i) {
    // 类别分数取最大
    for (int c = 0; c < nc; ++c) {
        float v = isNC ? d[i*C + (4+c)] : d[(4+c)*N + i];
        if (v > bv) { bv = v; bi = c; }
    }
    if (bv < o.conf || bi < 0) continue;

    // 框：cx cy w h
    float cx = isNC ? d[i*C+0] : d[0*N+i];
    ...
    // 还原坐标：先减填充，再除缩放
    float x1 = (cx - bw/2 - px) / sx;
```

**内存布局**：`(C,N)` 行优先，第 `row` 行第 `i` 个 anchor 在 `d[row*N + i]`。

**⑤ 关键点**

```cpp
int ks = 4 + nc;    // 关键点起始行
for (int k = 0; k < 4; ++k) {
    float kx = isNC ? d[i*C + (ks + k*2 + 0)] : d[(ks+k*2+0)*N + i];
    float ky = isNC ? d[i*C + (ks + k*2 + 1)] : d[(ks+k*2+1)*N + i];
    kp.emplace_back((kx - px)/sx, (ky - py)/sy);
}
```

**⑥ NMS**

```cpp
sort(idx.begin(), idx.end(), [&](int a, int b){ return sc[a] > sc[b]; });
for (...) {
    keep.push_back(idx[i]);
    for (j = i+1 ...) {
        // IoU = 交集 / 并集
        if (inter / uni > o.iou) dead[idx[j]] = 1;
    }
}
```

**为什么不用 `cv::dnn::NMSBoxes`**：它只吃整数 `Rect`，我们的框是 `Rect2f`，会丢精度。

#### `Draw(img, ds, o)`

画框、类别名、置信度、关键点。按类别 id 选颜色。

---

### `pnp_solver` — 位姿解算

#### `rebuildObjectPoints()` — 建模型四点

```cpp
const float hw = size_.width  * 0.5f;
const float hh = size_.height * 0.5f;
objPts_ = {
    {-hw,  hh, 0.0f},   // 左上
    { hw,  hh, 0.0f},   // 右上
    { hw, -hh, 0.0f},   // 右下
    {-hw, -hh, 0.0f},   // 左下
};
```

**坐标系**：原点在板中心，X 右、Y 上、Z 垂直板面向外。

**为什么原点在中心**：`tvec` 直接就是板中心的相机系坐标，`norm(tvec)` 就是距离。

> 板子正对相机时 `R ≈ diag(1,-1,-1)`，因为相机 Y 轴朝下。
> **这不是 bug，是坐标系约定。**

#### `reprojectionError(...)`

```cpp
cv::projectPoints(obj, rvec, tvec, cam.K, cam.D, proj);
double sum = 0;
for (i) sum += cv::norm(proj[i] - img[i]);
return sum / proj.size();
```

**这是唯一可信的质检工具。** 把位姿投回图像，和实际角点比。超过阈值（3px）就丢弃该帧。

#### `physicallyValid(p)` — 物理约束

```cpp
if (p.tvec[2] <= 0) return false;          // ① 必须在相机前方

cv::Vec3d n(p.R(0,2), p.R(1,2), p.R(2,2)); // ② 板面法向 = R 第3列
cv::Vec3d view = -p.tvec;                   // 从板指向相机
if (n.dot(view) <= 0) return false;         // 法向必须朝相机
```

**这是解决"平面二义性"最有效的一道防线。**

#### `solveAll(corners)` ★ 拿全部候选

```cpp
cv::Mat errMats;   // 必须是 Mat，不能传 double*
int n = cv::solvePnPGeneric(objPts_, img, cam_.K, cam_.D,
                            rvecs, tvecs, false,
                            cv::SOLVEPNP_IPPE,
                            cv::noArray(), cv::noArray(), errMats);
```

**为什么不用 `solvePnP`**：

平面靶标有**二义性**——同一张图有两个都讲得通的解：

| 量 | 真实解 | 兄弟解 | 差别 |
|---|---|---|---|
| 距离 | 3000.0 mm | 3005.9 mm | 只差 6mm |
| 位置 | 200 mm | 198.8 mm | 只差 1mm |
| **朝向** | 朝左歪 20° | **朝右歪 31°** | **差 51°** |

两个解在照片上只差 **0.35 像素**，比噪声还小。`solvePnP` 会随便挑一个，**经常挑错**。

**`SOLVEPNP_IPPE` 的理由**：专为平面靶标设计的解析解，快且稳。

**不要用 `SOLVEPNP_IPPE_SQUARE`**：它要求严格正方形。你的板子 138×52 不是正方形。

#### `chooseBest(cands, out)` — 挑选策略

**第 1 步：物理过滤**
```cpp
for (const auto& c : cands)
    if (physicallyValid(c)) valid.push_back(c);
```

**第 2 步：有历史时，选离上一帧最近的**
```cpp
double dt = cv::norm(pool[i].tvec - lastTvec_);
double dr = cv::norm(pool[i].rvec - lastRvec_);
double score = dt + 500.0 * dr;   // 0.02rad ≈ 1°
```

**这是最有效的一条**，它利用了单张照片没有的信息——时间连续性。

**第 3 步：没历史时按重投影误差**

**第 4 步：误差接近（<0.5px）时选更正对的**
```cpp
if (pool[i].reprojErr - e0 < 0.5) {
    if (frontness(pool[i]) > frontness(pool[bestIdx])) bestIdx = i;
}
```

#### `solve(corners)` — 主入口

```cpp
auto cands = solveAll(corners);
chooseBest(cands, res);
// 可选：solvePnPRefineLM 精化，只在变好时采纳
```

---

### `kalman_tracker` — 卡尔曼滤波

状态 `x = [x,y,z,vx,vy,vz]`，测量 `z = [x,y,z]`。

#### `buildMatrices(dt, z, cam, size)` ★ R 的自适应

```cpp
const double dist = fabs(z[2]);
const double sx = dist / fx * sp;                 // 横向 ∝ z
const double sz = dist * dist / (fx * W) * sp;    // 纵向 ∝ z²

R_(0,0) = sx * sx;
R_(1,1) = sy * sy;
R_(2,2) = sz * sz;
```

**这是整个滤波器最关键的设计。**

**为什么必须各向异性**：

- 横向：1 像素对应 `z/f` mm
- 纵向：1 像素对应 `z²/(f·W)` mm

代入 `f=2400, W=138, z=3000, σ=0.5`：
- `σx ≈ 0.6 mm`
- `σz ≈ 14 mm`

**Z 噪声是 X 的 20 多倍，随距离平方增长。**

设成 `R = σ²I` 的话：要么 X 被过度平滑（滞后严重），要么 Z 完全压不住。

**Q 用连续白噪声加速度模型**：
```cpp
Q_(i,i)     = dt^4/4 * sa²;
Q_(i,i+3)   = dt^3/2 * sa²;
Q_(i+3,i+3) = dt^2   * sa²;
```

#### `init(z, cam, size)`

```cpp
x_[0..2] = z;                              // 位置用测量
P_(0..2, 0..2) = 50*50;                    // 位置不确定度 50mm
P_(3..5, 3..5) = 1000*1000;                // 速度完全不知道
```

**P 必须设大**——表示"我对初值没信心"。不设的话默认单位阵，滤波器以为自己很准，前几帧几乎不理测量。

#### `update(z, hasMeas, cam, size, dt)` ★ 五个公式

```cpp
// ① 预测
x_ = F_ * x_;
P_ = F_ * P_ * Fᵀ + Q_;

// ② 没测量就只预测
if (!hasMeas) { ++lostFrames_; return; }

// ③ 新息
y = zz - H_ * x_;
S = H_ * P_ * Hᵀ + R_;

// ④ 增益（用 SVD 求逆，S 可能接近奇异）
K = P_ * Hᵀ * S⁻¹;

// ⑤ 更新
x_ = x_ + K * y;
P_ = (I - K*H) * P_;
```

**第 ② 步是卡尔曼比"滑动平均"高级的地方**：目标被遮挡时轨迹不断，只是 `P` 越来越大。

#### `dt` 兜底

```cpp
if (!(dt > 1e-4 && dt < 1.0)) dt = 1.0/30.0;
```

程序卡顿、第一帧、跳帧都可能给出荒谬的 dt。

#### `predictAfter(T)`

```cpp
return cv::Vec3d(x_[0] + x_[3]*T, ...);
```

**为什么不用 `kf.predict()`**：那会真的推进滤波器状态，连续调用会把 `statePost` 污染，轨迹越跑越偏。

匀速模型下解析式和 predict 等价，**但无副作用**。

---

### `main` — 主循环

#### 参数解析

手写解析，`--key value` 形式：

```cpp
auto nx = [&]() -> std::string {
    if (i + 1 >= argc) throw std::runtime_error(a + " 缺参数");
    return argv[++i];
};
```

#### 主循环骨架

```cpp
while (true) {
    if (!cam_dev.Grab(bgr, 1000) || bgr.empty()) continue;   // 超时跳过，不 break

    auto tNow = steady_clock::now();
    double dt = duration<double>(tNow - tPrev).count();       // 真实时间差
    tPrev = tNow;

    auto ds = det.Run(bgr, o);
    // ... 选最佳 → PnP → 卡尔曼 → 显示
    if (waitKey(1) == 'q') break;
}
```

**别写 `if (!Grab()) break;`**：偶尔一帧超时很正常（曝光短、光照不足），立刻退出太脆弱。

#### 关键点顺序重排 ★

```cpp
// 模型输出逆时针：左上、左下、右下、右上
// PnP 需顺时针：  左上、右上、右下、左下
corners[0] = d.kp[0];
corners[1] = d.kp[3];
corners[2] = d.kp[2];
corners[3] = d.kp[1];   // {0,3,2,1}
```

**写错会怎样**：重投影误差**完全正常**，但朝向是镜像的。
症状：板子明明朝左歪，程序说朝右。**极难发现**，因为所有常规检查都通过。

#### 类别切换尺寸

```cpp
if (d.id != lastClass) {
    lastClass = d.id;
    ArmorSize s = armorSizeOf(d.id);
    pnp.setArmorSize(s);
    std::cout << "[信息] 切换尺寸: " << d.name << "\n";
}
```

#### 卡尔曼两种调用

```cpp
if (gotPose) tracker.update(bestPose.tvec, true, cam, curSize, dt);
else         tracker.update(cv::Vec3d(0,0,0), false, cam, curSize, dt);
```

`update` 内部处理了未初始化的情况（第一次有测量时自动 `init`）。

#### 丢失重置

```cpp
if (tracker.lost()) {
    tracker.reset();
    pnp.reset();
    lastClass = -1;
}
```

---

## 关键设计说明

### 1. 为什么 height 是 52mm 而不是 125mm

模型的关键点标在**灯条发光段**上，不是外壳边缘。

用两个不同距离的帧反推验证：

| 来源 | 像素宽 | 像素高 | 假设宽=137 反推的高 |
|---|---|---|---|
| 视频帧 300 | 404.3 | 164.1 | **55.6 mm** |
| 截图 | 140.3 | 54.3 | **53.0 mm** |

两个不同距离反推出的高度都收敛到 53~56mm，说明这不是偶然。

### 2. 为什么 R 必须各向异性

单目相机的纵向（Z）精度天然差：

```
σz / σx = z / W   （与焦距、像素噪声无关）
```

3 米处、板宽 138mm → 比值约 22 倍。设成各向同性必然失败。

### 3. 为什么用 solvePnPGeneric

平面靶标的二义性导致两个解只差 0.35 像素，`solvePnP` 会随机挑。
必须拿回所有候选，用**物理约束 + 时间连续性**自己挑。

### 4. 为什么 Bayer 要区分四种排列

`BayerRG` / `BayerGB` / `BayerGR` / `BayerBG` 分别对应不同的像素排列，
用错会导致**红蓝通道颠倒**——装甲板红色和蓝色类别会认反。

---

## 常见问题

### 编译

| 报错 | 原因 | 解决 |
|---|---|---|
| `找不到 MVS SDK` | 路径不对 | `cmake .. -DMVS_ROOT=/opt/MVS` |
| `Could not find OpenCV` | 没装 | `sudo apt install libopencv-dev` |
| `dnn ... not found` | OpenCV 版本太老 | 需要 4.5+ |
| `'filesystem' is not a member` | GCC < 9 | `sudo apt install g++-9` |

### 运行

| 现象 | 原因 | 解决 |
|---|---|---|
| `找不到相机` | 被 MVS 客户端占用 | 关闭 MVS 客户端 |
| `枚举不到设备` | 权限 | `sudo usermod -aG video $USER` |
| `MVS失败: 0x80000203` | 设备被占用 | 同上 |
| 取帧一直超时 | 曝光太短/太暗 | 调大 Exposure 滑块 |
| 图像斜了 | step 算错 | 检查 `nFrameLen/height` |
| 红蓝颠倒 | Bayer 排列不对 | 换 `COLOR_BayerXX2BGR` |
| 检测不到目标 | RGB/BGR 搞反 | 检查 `swapRB=true` |

### 精度

| 现象 | 检查 |
|---|---|
| 距离差固定倍数 | `ArmorSize` 的 width/height |
| 距离差固定偏移 | 标定的主点 `cx cy` |
| 重投影误差 > 3px | 角点顺序、尺寸、标定 |
| 朝向镜像 | 关键点重排 `{0,3,2,1}` |
| Z 轴穿进板子 | 碰上翻转解，看 `flipped` 标志 |

---

## 性能测试

在 Ubuntu 22.04 + Ryzen 5 5600 + 海康 MV-CS016-10UC 上：

| 环节 | 耗时 |
|---|---|
| 取帧 (BGR8) | ~1 ms |
| 取帧 (Bayer，需转换) | ~3 ms |
| letterbox + blob | ~2 ms |
| YOLO 推理 (CPU) | 20~25 ms |
| PnP | < 1 ms |
| 卡尔曼 | < 0.1 ms |
| **总计** | **~25 ms (40 fps)** |

**注意**：Debug 模式下推理会慢到 100ms+，测性能务必用 Release。

---

## 参考

- [OpenCV solvePnP 文档](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [张正友标定法](https://ieeexplore.ieee.org/document/888718)
- [海康机器人 MVS SDK](https://www.hikrobotics.com/cn/machinevision/service/download)

---

## License

本项目为课程作业，仅用于学习交流。
