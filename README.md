# Reprojection (C++17)

使用理想针孔模型计算三维点的重投影像素坐标，并与观测点比较像素欧氏距离。

## 约定

- 外参：`Pc = R * Pw + t`（世界到相机）
- 内参：`fx, fy, cx, cy`
- 忽略畸变
- 三维坐标与平移量单位一致

## 构建

```bash
mkdir -p build && cd build
cmake ..
make

