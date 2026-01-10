# 数据后处理与转换工具指南

`scripts/data_convert/mcap_to_dataset.py` 是用于将原始 MCAP 采集数据转换为标准训练数据集格式的核心工具。

## 1. 功能概述

输入：
*   `.mcap` 文件：包含原始 RGBD 视频流与机械臂状态。
*   `calibration.yaml`：双相机外参标定结果。

核心逻辑：
*   **智能提取**: 自动扫描并解码 H.264 视频附件，仅取**第 1 帧**作为 RGB 数据。
*   **自动过滤**: 从连续的深度流中仅取**第 1 帧**深度图。
*   **状态锁定**: 从连续的机械臂状态流中仅取**最后 1 帧**关节角。

输出：
*   `_fused.ply`：双相机视角融合后的带颜色点云（来自第1帧）。
*   `.json`：机械臂的7维关节角状态（来自最后1帧，即抓取得分位姿）。

---

## 2. 命令行参数详解

### 基础参数
| 参数 | 说明 | 示例 |
| :--- | :--- | :--- |
| `--mcap` | 单个文件路径 (与dir互斥) | `data/demo_0.mcap` |
| `--mcap-dir` | 批量处理的目录路径 | `data/task_A/` |
| `--calibration` | **[必需]** 标定文件路径 | `calibration_result.yaml` |
| `--pointcloud-output` | 点云文件输出目录 | `dataset/pointclouds/` |
| `--joints-output` | 关节角数据输出目录 | `dataset/joints/` |

### 高级调优参数
| 参数 | 默认值 | 作用说明 |
| :--- | :--- | :--- |
| `--voxel-size` | `None` | 体素下采样尺寸(米)。推荐 `0.005` (5mm) 以显著减小文件体积。 |
| `--depth-min` | `0.1` | 最小有效深度(米)。小于此距离的点将被过滤。 |
| `--depth-max` | `5.0` | 最大有效深度(米)。大于此距离的点将被过滤。 |
| `--depth-scale` | `0.001` | 深度图单位换算因子 (RealSense默认毫米，通常无需修改)。 |

---

## 3. 使用示例

### 场景A: 抽样检查 (处理单个文件)
```bash
python scripts/data_convert/mcap_to_dataset.py \
  --mcap /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_03/0.mcap \
  --calibration /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/scripts/dual_camera_calibration.yaml \
  --pointcloud-output /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_03_pcd \
  --joints-output /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_03_joints
```

### 场景B: 制作训练集 (批量+下采样)
推荐在制作数据集时开启体素下采样 (`--voxel-size 0.005`)，可以将点云文件大小减少约90%，且保留足够几何细节。

```bash
python scripts/data_convert/mcap_to_dataset.py \
  --mcap-dir /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_03/ \
  --calibration /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/scripts/dual_camera_calibration.yaml \
  --pointcloud-output /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_03_pcd \
  --joints-output /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_03_joints \
  --voxel-size 0.005 \
  --depth-min 0.2 \
  --depth-max 1.5
```

---

## 4. 输出数据结构说明

### 关节角 JSON文件
```json
[
  0.523, -0.392, 1.047, -1.570, 1.570, 0.000,  // 关节 1-6 (弧度)
  0.085                                        // 夹爪开度 (米)
]
```

### 点云 PLY文件
标准的二进制 PLY 格式，包含：
*   `x, y, z`: 空间坐标 (单位: 米)
*   `red, green, blue`: 颜色信息 (0-255)
*   `nx, ny, nz`: 法向量 (如果计算了的话)
