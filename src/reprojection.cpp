// src/reprojection.cpp
// 重投影计算的实现

#include "reprojection.h"
#include <cmath>

namespace reprojection {

namespace {

// 3x3 矩阵乘 3x1 向量
Vec3 mat3_mul_vec3(const Mat3& R, const Vec3& v) {
    Vec3 out{};
    for (int i = 0; i < 3; ++i) {
        out[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
    }
    return out;
}

// 3x1 向量加法
Vec3 vec3_add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

}  // namespace

std::optional<Result> reproject(const Vec3& Pw,
                                const Intrinsics& K,
                                const Extrinsics& E,
                                const Vec2& observed) {
    // 1. 世界 -> 相机：Pc = R * Pw + t
    Vec3 Pc = vec3_add(mat3_mul_vec3(E.R, Pw), E.t);

    // 2. 非正深度：点在相机后方或平面上，投影无意义
    if (Pc[2] <= 0.0) {
        return std::nullopt;
    }

    // 3. 归一化平面
    double x = Pc[0] / Pc[2];
    double y = Pc[1] / Pc[2];

    // 4. 投影到像素
    double u = K.fx * x + K.cx;
    double v = K.fy * y + K.cy;

    // 5. 与观测点的像素欧氏距离
    double dx = u - observed[0];
    double dy = v - observed[1];
    double err = std::hypot(dx, dy);

    return Result{{u, v}, err};
}

}  // namespace reprojection

