// src/main.cpp
// 构造多组数据验证重投影计算，包含非正深度的异常用例

#include "reprojection.h"
#include <iostream>
#include <iomanip>

using namespace reprojection;

namespace {

// 打印一组测试结果
void run_case(const char* name,
              const Vec3& Pw,
              const Intrinsics& K,
              const Extrinsics& E,
              const Vec2& observed) {
    std::cout << "=== " << name << " ===\n";
    std::cout << "世界点 Pw = (" << Pw[0] << ", " << Pw[1] << ", " << Pw[2] << ")\n";

    auto res = reproject(Pw, K, E, observed);
    if (!res) {
        std::cout << "结果: 无效（点在相机后方，Zc <= 0）\n\n";
        return;
    }

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "重投影像素: (" << res->pixel[0] << ", " << res->pixel[1] << ")\n";
    std::cout << "观测像素:   (" << observed[0] << ", " << observed[1] << ")\n";
    std::cout << "像素误差:   " << res->error << " px\n\n";
}

}  // namespace

int main() {
    // 公共内参
    Intrinsics K{800.0, 800.0, 640.0, 360.0};

    // 公共外参：单位旋转 + 沿 Z 平移 1
    Mat3 R = {{{1, 0, 0},
               {0, 1, 0},
               {0, 0, 1}}};
    Vec3 t = {0.0, 0.0, 1.0};
    Extrinsics E{R, t};

    // 用例 1：光轴上的点，投影落于光心，误差应为 0
    run_case("用例1: 光轴点，误差 0",
             {0.0, 0.0, 3.0}, K, E, {640.0, 360.0});

    // 用例 2：观测点右移 10 像素，误差应为 10
    run_case("用例2: 观测偏移 10px",
             {0.0, 0.0, 3.0}, K, E, {650.0, 360.0});

    // 用例 3：世界点偏离光轴，验证投影公式
    // 预期重投影 (740, 360)，误差 100
    run_case("用例3: 偏离光轴",
             {0.5, 0.0, 3.0}, K, E, {640.0, 360.0});

    // 用例 4：异常——点在相机后方（Zc <= 0）
    run_case("用例4: 非正深度（异常）",
             {0.0, 0.0, -5.0}, K, E, {640.0, 360.0});

    return 0;
}

