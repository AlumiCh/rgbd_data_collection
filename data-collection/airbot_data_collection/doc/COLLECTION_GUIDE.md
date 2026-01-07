# RGBD 数据采集与处理指南

本指南旨在说明如何使用单机械臂与双 D435i 相机进行同步 RGBD 数据采集及后续的点云转换流程。

## 1. 完整数采操作流程

### 第一步：外参标定 (准备阶段)
在正式采集前，若相机位置发生变动，需运行标定程序获取两个视角之间的相对转换矩阵。

*   **运行命令**:
    ```bash
    # 进入开发目录
    cd data-collection/airbot_data_collection
    # 运行标定程序（示例：使用 9x6 棋盘格，方格边长 0.02m）
    python scripts/calibrate_dual_cameras.py --size "9x6" --square 0.02
    ```
*   **操作员动作**: 手持棋盘格，确保两个相机都能清晰看到棋盘格。在程序提示时按下 `s` 键保存当前帧并移动棋盘格位置。建议至少保存 20 组不同角度的数据。
*   **产出**: 生成 `calibration_result.yaml`，包含 `T_2_1`（相机 2 到相机 1 的坐标变换矩阵）。

### 第二步：数据采集 (执行阶段)
*   **运行命令**:
    ```bash
    # 使用专用配置文件启动采集程序
    python main.py --config-name config_single_arm_dual_rgbd \
           dataset.directory="data/pick_and_place_task" \
           sample_limit=1
    ```
*   **主要参数**:
    *   `--config-name`: 指定使用 `config_single_arm_dual_rgbd.yaml`。
    *   `dataset.directory`: 设置数据存储目录。
    *   `sample_limit=1`: 按照需求，每运行一次程序采集 1 帧数据。
*   **操作员动作**: 
    1.  布置好抓取环境（物体放置）。
    2.  启动命令，机械臂会自动记录当前关节角。
    3.  程序会自动开启 RealSense 分发并捕捉一帧 RGBD 图像并存储为 MCAP 格式。
    4.  确认终端显示 "Collection finished" 即可。

### 第三步：点云转换 (后处理阶段)
*   **运行命令**:
    ```bash
    # 将采集的原始 RGBD 视频流转换为融合后的点云文件
    python scripts/data_convert/rgbd_to_pointcloud.py \
           --input data/pick_and_place_task/demo_0.mcap \
           --calibration calibration_result.yaml \
           --output_format pcd
    ```
*   **参数**:
    *   `--input`: 输入的 MCAP 文件路径。
    *   `--calibration`: 引用第一步生成的标定文件。
    *   `--output_format`: 选择 `pcd` 或 `ply`。

---

## 2. 关键可调参数说明

在实际环境下，您可能需要根据光照、物体材质和物理距离调整以下参数：

### 配置 YAML 文件中的参数 (`defaults/config_single_arm_dual_rgbd.yaml`):
1.  **`serial_number`**: 必须替换为实际 D435i 相机底部的序列号，否则无法识别硬件。
2.  **`image_size`**: `[640, 480]` 是推荐的平衡性能与分辨率的选项，若需更高精度可设为 `[1280, 720]`。
3.  **`fps`**: 当前设为 30。对于单帧采集，此参数影响较小，但对于动态轨迹采集需保证稳定。

### 转换脚本中的参数 (`scripts/data_convert/rgbd_to_pointcloud.py`):
1.  **`depth_threshold` (核心控制)**: 
    *   **作用**: 过滤掉背景点云（如远处墙壁）。单位为米。
    *   **建议**: 若抓取环境较小，建议设为 `1.0` (1米) 或更短，以减少杂点。
2.  **`voxel_size`**:
    *   **作用**: 点云下采样的体素大小（单位：米）。
    *   **建议**: 设置为 `0.005` (5mm) 可有效平衡点云细节与文件大小。
3.  **`depth_scale`**: 
    *   **作用**: 深度值映射。Intel RealSense 默认为 `1000.0` (即 1mm = 1单位)。若使用不同型号相机可能需要修改。

---

## 3. 程序依赖与架构逻辑

系统的运行流程遵循以下层级依赖关系：

1.  **硬件驱动层 (底层)**: 
    *   依赖 `librealsense` 和 `airbot_py`。
    *   `Main.py` 启动时会通过 `RealSenseObserver` 和 `AirbotArm` 实例初始化硬件模块。

2.  **状态机控制层 (中层)**: 
    *   `fsm.py` 驱动。它管理从 `START` -> `PREPARING` -> `SAMPLING` -> `SUCCESS` 的流转。
    *   **依赖关系**: FSM 依赖于 `McapSampler` 来执行具体的“数据写入”动作。

3.  **数据采样层 (核心)**: 
    *   `McapSampler` 订阅来自相机的 `OBSERVATION` 消息。
    *   当 FSM 触发采样信号时，Sampler 将 `uint16` 深度图（RAW）和 `JPEG/H264` 彩色图打包存入 MCAP 容器。

4.  **离线工具链 (顶层)**: 
    *   `rgbd_to_pointcloud.py` 独立运行，不依赖实时硬件。
    *   它读取 MCAP 容器，利用标定结果 (`T_2_1`) 对点云进行空间配准并执行 Open3D 投影算法。

**总结逻辑图**:
`用户命令` -> `main.py(FSM)` -> `传感器(获取RGBD)` + `机器人(获取关节角)` -> `MCAP(原始存储)` -> `转换工具(坐标变换+重构)` -> `点云PCD`
