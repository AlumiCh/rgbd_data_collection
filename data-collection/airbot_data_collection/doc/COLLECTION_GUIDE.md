# 单臂双摄数据采集与处理主指南

本指南涵盖了从硬件标定、数据采集到数据后处理的全流程操作。

## 1. 硬件外参标定 (准备阶段)

在正式采集前，若相机位置发生变动，需运行标定程序获取两个视角之间的相对转换矩阵。

*   **运行命令**:
    ```bash
    cd data-collection/airbot_data_collection
    # 使用 9x6 棋盘格，方格边长 0.02m
    python scripts/calibrate_dual_cameras.py --size "9x6" --square 0.02
    ```
*   **操作**: 手持棋盘格，在两个相机视野内移动。按 `s` 保存当前帧（建议>20组）。
*   **产出**: `calibration_result.yaml` (含 `T_2_1` 变换矩阵)。

## 2. 外参位姿精修 (优化阶段 - 推荐)

标定板得到的粗测矩阵可能因机械形变有微小偏差。建议在采集一个正式样本后，使用精修工具锁定最佳对齐位姿。

*   **运行命令**:
    ```bash
    python scripts/refine_calibration.py \
           --mcap data/task_name/sample_0.mcap \
           --yaml calibration_result.yaml
    ```
*   **交互操作**:
    1.  **自动**: 按 `I` 键执行 ICP 算法，系统会基于场景几何特征自动对齐点云。
    2.  **微调**: 使用键盘（Q/W/A/S/Z/X 等）手动移动点云，直到两个视角完美重合。
    3.  **保存**: 按 `Enter` 键，精修后的矩阵将直接写回 `calibration_result.yaml`。

---

## 3. 数据采集 (执行阶段)

### 2.1 启动机械臂服务
在采集前，**必须**启动机械臂主从跟随的实例

### 2.2 运行采集程序

系统默认配置为**首尾帧过滤模式**（只保存第1帧RGBD + 最后1帧关节角）。

*   **运行命令**:
    ```bash
    python main.py --path defaults/config_single_arm_dual_rgbd.yaml --dataset.directory data/task_name_here
    ```

*   **关键参数说明**:
    *   `dataset.directory`: 数据存放路径。
    *   `sample_limit.size=1000`: **重要**。设定采样缓冲区的最大帧数。

*   **操作员流程**:
    1.  **准备**: 将物体放置在场景中。
    2.  **激活**: 启动机械臂Follow模式（主臂控制从臂）。
    3.  **开始 (按Space)**: 系统记录第1帧（此时人手应远离，相机拍摄无遮挡物体）。
    4.  **动作**: 操作主臂移动到目标抓取位姿。
    5.  **结束 (再按Space)**: 系统记录最后1帧（此时机械臂处于抓取位姿）。
        *   *系统默认采集全过程数据（H.264视频流），后处理脚本会自动提取所需的第1帧和最后1帧。*
    6.  **保存 (按s)**: 写入MCAP文件。
    7.  **放弃 (按d)**: 如果操作失误，不保存并重置。

---

## 3. 数据后处理 (转换阶段)

采集得到的 `.mcap` 文件需要转换为标准的点云 (`.ply`) 和关节角数据 (`.json`) 才能用于可视化或训练。

*   **转换命令 (批量处理整个目录)**:
    ```bash
    python scripts/data_convert/mcap_to_dataset.py \
           --mcap-dir data/task_name_here/ \
           --calibration calibration_result.yaml \
           --pointcloud-output pointclouds/ \
           --joints-output joints/ \
           --depth-max-cam1 0.8 --depth-max-cam2 1.2 \
           --apply-icp
    ```

*   **主要功能**:
    1.  提取 RGBD 并根据标定文件融合双相机点云（自动处理外参求逆）。
    2.  **分相机过滤**: 支持为两台相机设置不同的深度阈值，适应不同安装距离。
    3.  **ICP 优化**: (可选) 使用点到面 ICP 算法精细化对齐，消除标定误差。
    4.  提取最后时刻的机械臂关节角。
    5.  (可选) 进行体素下采样 (`--voxel-size`) 减小文件体积。

*   *详细参数请参考: [MCAP_TO_DATASET_GUIDE.md](./MCAP_TO_DATASET_GUIDE.md)*

---

## 4. 数据可视化 (验证阶段)

检查转换后的数据质量。

*   **可视化命令**:
    ```bash
    # 自动加载对应的点云和关节角文件
    python scripts/visualize_sample.py --pointcloud pointclouds/demo_0_fused.ply
    ```

*   *详细操作请参考: [VISUALIZATION_GUIDE.md](./VISUALIZATION_GUIDE.md)*

---
