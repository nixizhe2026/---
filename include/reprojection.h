// include/reprojection.h
// 重投影计算：世界点 -> 相机坐标 -> 像素坐标，并计算与观测点的像素误差
//
// 约定：
//   外参：Pc = R * Pw + t   （世界到相机）
//   内参：fx, fy, cx, cy
//   针孔模型，无畸变
//   三维坐标与平移量单位一致

#ifndef REPROJECTION_H
#define REPROJECTION_H

#include <array>
#include <optional>

namespace reprojection {

// ---------- 基础类型 ----------
using Vec3 = std::array<double, 3>;
using Vec2 = std::array<double, 2>;
using Mat3 = std::array<std::array<double, 3>, 3>;

// ---------- 相机内参 ----------
struct Intrinsics {
    double fx;  // 焦距（像素）
    double fy;  // 焦距（像素）
    double cx;  // 光心 u（像素）
    double cy;  // 光心 v（像素）
};

// ---------- 相机外参（世界 -> 相机）----------
struct Extrinsics {
    Mat3 R;  // 旋转矩阵
    Vec3 t;  // 平移向量
};

// ---------- 重投影结果 ----------
struct Result {
    Vec2 pixel;    // 重投影得到的像素坐标 (u, v)
    double error;  // 与观测点的像素欧氏距离
};

// ---------- 核心函数 ----------
// 世界点 Pw 投影到像素，并与观测点比较。
// 返回 std::nullopt 表示点在相机后方（Zc <= 0），投影无意义。
std::optional<Result> reproject(const Vec3& Pw,
                                const Intrinsics& K,
                                const Extrinsics& E,
                                const Vec2& observed);

}  // namespace reprojection

#endif  // REPROJECTION_H

