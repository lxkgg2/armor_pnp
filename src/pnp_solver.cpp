#include "../include/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <cmath>
#include <iostream>

using namespace std;

PnPSolver::PnPSolver(const ArmorSize& size, const CameraParam& cam)
    : size_(size), cam_(cam)
{
    rebuildObjectPoints();
}

// ============================================================
//  建立模型四点
//
//  坐标系：原点在板中心，X 向右，Y 向上，Z 垂直板面向外（朝相机）
//  顺序必须和角点顺序一致：左上 → 右上 → 右下 → 左下
//
//  为什么原点在中心？
//    这样 tvec 直接就是"板中心在相机系下的坐标"，
//    norm(tvec) 就是距离，不用再换算。
// ============================================================
void PnPSolver::rebuildObjectPoints()
{
    const float hw = size_.width  * 0.5f;
    const float hh = size_.height * 0.5f;
    objPts_ = {
        {-hw,  hh, 0.0f},   // 左上
        { hw,  hh, 0.0f},   // 右上
        { hw, -hh, 0.0f},   // 右下
        {-hw, -hh, 0.0f},   // 左下
    };
}

// ============================================================
//  重投影误差
//
//  把解出来的位姿重新投影回图像，和实际检测到的角点比较。
//  这是唯一可信的"这一帧算得对不对"的判据。
// ============================================================
double PnPSolver::reprojectionError(const vector<cv::Point3f>& obj,
                                    const array<cv::Point2f, 4>& img,
                                    const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                                    const CameraParam& cam)
{
    vector<cv::Point2f> proj;
    cv::projectPoints(obj, rvec, tvec, cam.K, cam.D, proj);

    double sum = 0;
    for (size_t i = 0; i < proj.size(); ++i) {
        sum += cv::norm(proj[i] - img[i]);
    }
    return sum / (double)proj.size();
}

// ============================================================
//  物理约束
//
//  两条硬性规则，能干掉绝大多数"翻面"解：
//   ① tz > 0       物体必须在相机前方（Z 轴朝前）
//   ② n·(-t) > 0   板面法向要朝向相机（法向与视线夹角 < 90°）
// ============================================================
bool PnPSolver::physicallyValid(const PoseResult& p)
{
    // ① 在相机前方
    if (p.tvec[2] <= 0) return false;

    // ② 法向朝向相机
    //    R 的第三列就是板面法向（因为模型里 Z 垂直于板面）
    cv::Vec3d n(p.R(0, 2), p.R(1, 2), p.R(2, 2));
    cv::Vec3d view = -p.tvec;          // 从板中心指向相机
    double dot = n.dot(view);
    if (dot <= 0) return false;

    return true;
}

// ============================================================
//  从候选中挑最优
//
//  优先级：
//   1) 先剔除物理上不成立的
//   2) 有上一帧时：选与上一帧位姿最接近的（时间连续性，最有效）
//   3) 没有上一帧时：选区分的重投影误差更小的
//   4) 两者误差差得不够大（< 0.5px）时，倾向于"更接近正面"的解
//
//  第 4 条的依据：装甲板实际使用中朝向变化连续，
//  突然出现的极端倾角多半是伪解。
// ============================================================
bool PnPSolver::chooseBest(const vector<PoseResult>& cands, PoseResult& out) const
{
    if (cands.empty()) return false;

    // 第 1 步：物理过滤
    vector<PoseResult> valid;
    for (const auto& c : cands) {
        if (physicallyValid(c)) valid.push_back(c);
    }
    // 全被过滤掉时退回原始候选（避免完全没输出）
    const vector<PoseResult>& pool = valid.empty() ? cands : valid;

    // 第 2 步：有历史就按位姿距离挑
    if (hasLast_ && pool.size() > 1) {
        double bestScore = 1e18;
        int bestIdx = 0;
        for (size_t i = 0; i < pool.size(); ++i) {
            double dt = cv::norm(pool[i].tvec - lastTvec_);
            double dr = cv::norm(pool[i].rvec - lastRvec_);
            // 平移(mm) 和 旋转(rad) 加权：0.02 rad ≈ 1°，权重给 500mm/rad
            double score = dt + 500.0 * dr;
            if (score < bestScore) { bestScore = score; bestIdx = (int)i; }
        }
        out = pool[bestIdx];
        return true;
    }

    // 第 3 步：按重投影误差
    int bestIdx = 0;
    double bestErr = pool[0].reprojErr;
    for (size_t i = 1; i < pool.size(); ++i) {
        if (pool[i].reprojErr < bestErr) { bestErr = pool[i].reprojErr; bestIdx = (int)i; }
    }

    // 第 4 步：误差接近时，选更"正面"的
    // 正面程度 = 法向与视线方向的接近程度，越接近 1 越正
    if (pool.size() > 1) {
        double e0 = pool[bestIdx].reprojErr;
        for (size_t i = 0; i < pool.size(); ++i) {
            if (i == (size_t)bestIdx) continue;
            if (pool[i].reprojErr - e0 < 0.5) {
                auto frontness = [](const PoseResult& p) {
                    cv::Vec3d n(p.R(0, 2), p.R(1, 2), p.R(2, 2));
                    cv::Vec3d view = -p.tvec;
                    double c = n.dot(view) / (cv::norm(view) + 1e-9);
                    return c;   // 越大越正对
                };
                if (frontness(pool[i]) > frontness(pool[bestIdx])) {
                    bestIdx = (int)i;
                }
            }
        }
    }

    out = pool[bestIdx];
    return true;
}

// ============================================================
//  求全部候选（不带状态，方便调试分析）
// ============================================================
vector<PoseResult> PnPSolver::solveAll(const array<cv::Point2f, 4>& corners) const
{
    vector<PoseResult> out;

    vector<cv::Point2f> img(corners.begin(), corners.end());

    // solvePnPGeneric 会把所有候选解都返回（平面情形通常 2 个）
    vector<cv::Mat> rvecs, tvecs;
    cv::Mat errMats;                     // 注意：这里必须是 Mat，不能传 double*
    int n = cv::solvePnPGeneric(objPts_, img, cam_.K, cam_.D, rvecs, tvecs,
                                false, cv::SOLVEPNP_IPPE, cv::noArray(),
                                cv::noArray(), errMats);
    if (n <= 0) return out;

    for (int i = 0; i < n; ++i) {
        PoseResult pr;
        pr.ok = true;
        pr.rvec = cv::Vec3d(rvecs[i].at<double>(0), rvecs[i].at<double>(1),
                            rvecs[i].at<double>(2));
        pr.tvec = cv::Vec3d(tvecs[i].at<double>(0), tvecs[i].at<double>(1),
                            tvecs[i].at<double>(2));

        // Rodrigues 输出用 Mat 更稳妥，再拷进 Matx
        cv::Mat Rm;
        cv::Rodrigues(rvecs[i], Rm);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                pr.R(r, c) = Rm.at<double>(r, c);

        pr.distance = cv::norm(pr.tvec);
        pr.reprojErr = reprojectionError(objPts_, corners, pr.rvec, pr.tvec, cam_);
        out.push_back(pr);
    }
    return out;
}

// ============================================================
//  主解算
// ============================================================
PoseResult PnPSolver::solve(const array<cv::Point2f, 4>& corners)
{
    PoseResult res;

    // ① 拿到所有候选
    vector<PoseResult> cands = solveAll(corners);
    if (cands.empty()) {
        return res;   // ok=false
    }

    // ② 挑一个
    if (!chooseBest(cands, res)) return res;

    // ③ 精化（可选）：以挑出来的解为初值再做一次 LM
    //    收益不大，但能压一点噪声；失败就保留原解
    if (res.ok) {
        vector<cv::Point2f> img(corners.begin(), corners.end());
        cv::Mat rv = (cv::Mat_<double>(3, 1) << res.rvec[0], res.rvec[1], res.rvec[2]);
        cv::Mat tv = (cv::Mat_<double>(3, 1) << res.tvec[0], res.tvec[1], res.tvec[2]);
        try {
            cv::solvePnPRefineLM(objPts_, img, cam_.K, cam_.D, rv, tv);
            cv::Vec3d r2(rv.at<double>(0), rv.at<double>(1), rv.at<double>(2));
            cv::Vec3d t2(tv.at<double>(0), tv.at<double>(1), tv.at<double>(2));
            double e2 = reprojectionError(objPts_, corners, r2, t2, cam_);
            if (e2 < res.reprojErr) {
                res.rvec = r2;
                res.tvec = t2;
                res.reprojErr = e2;
                cv::Mat Rm;
                cv::Rodrigues(rv, Rm);
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        res.R(r, c) = Rm.at<double>(r, c);
                res.distance = cv::norm(res.tvec);
            }
        } catch (const cv::Exception&) {
            // 精化失败不影响主结果
        }
    }

    // ④ 记下这次结果，供下一帧做连续性判断
    if (res.ok) {
        // 标记是否与次优解构成"翻面"（差得很小但朝向相反）
        if (cands.size() > 1) {
            for (const auto& c : cands) {
                if (cv::norm(c.tvec - res.tvec) < 1e-6) continue;
                if (fabs(c.reprojErr - res.reprojErr) < 0.5) {
                    // 朝向差很多、误差差很少 → 存在混淆风险
                    double rr = cv::norm(c.rvec - res.rvec);
                    if (rr > 0.3) { res.flipped = true; break; }
                }
            }
        }
        lastRvec_ = res.rvec;
        lastTvec_ = res.tvec;
        hasLast_ = true;
    }
    return res;
}
