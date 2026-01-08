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

#### 准备工作：启动两个机械臂服务
在运行采集程序前，**必须分别启动两个机械臂的通信服务**：

```bash
# 终端1：启动主臂服务(lead，端口50052)
airbot_fsm -i can_left -p 50052

# 终端2：启动从臂服务(follow，端口50053)
airbot_fsm -i can_right -p 50053
```

#### 运行数据采集程序

*   **运行命令**:
    ```bash
    # 使用专用配置文件启动采集程序
    python main.py --config-name config_single_arm_dual_rgbd \
           dataset.directory="data/pick_and_place_task" \
           sample_limit.size=1
    ```
*   **主要参数**:
    *   `--config-name`: 指定使用 `config_single_arm_dual_rgbd.yaml`。
    *   `dataset.directory`: 设置数据存储目录。
    *   `sample_limit.size=1`: 每次按Space采集的最大帧数（设为1表示单帧）。
    
*   **操作员动作**: 
    1.  布置好抓取环境（物体放置）。
    2.  启动上述命令，等待系统初始化完成。
    3.  **启动主从机械臂跟随的实例**：激活跟随模式（主臂→从臂同步）
    4.  **按 `Space` 键**：开始记录数据
    5.  此时保持机械臂静止约0.5秒（系统会记录第1帧作为物体点云，此时机械臂未遮挡物体）
    6.  移动主臂（lead臂）到目标抓取位置，从臂会实时跟随
    7.  到达理想抓取位姿后，**再按 `Space` 键**：停止记录数据（系统会记录最后1帧的关节角作为目标位姿）
    8.  **按 `s` 键**：保存当前采集的数据
    9.  重复步骤4-8采集多条数据，或按 `ctrl + c` 键结束采集

*   **数据说明**：
    - 每条数据包含：**第1帧的双相机RGBD图像**（用于生成物体点云） + **最后1帧的机械臂关节角**（目标抓取位姿）
    - 采样时长建议：1-3秒（30-100帧），给操作员足够时间完成动作
    - 系统会自动过滤中间帧，只保存关键帧

### 第三步：点云转换 (后处理阶段)

点云转换工具现已精简为**双相机融合模式**，直接输出PLY格式。

*   **运行命令（单个MCAP文件）**:
    ```bash
    # 将采集的原始 RGBD 视频流转换为融合后的点云文件
    python scripts/data_convert/rgbd_to_pointcloud.py \
           --mcap data/pick_and_place_task/demo_0.mcap \
           --calibration calibration_result.yaml \
           --output pointclouds/
    ```

*   **批量处理命令（整个目录）**:
    ```bash
    python scripts/data_convert/rgbd_to_pointcloud.py \
           --mcap-dir data/pick_and_place_task/ \
           --calibration calibration_result.yaml \
           --output pointclouds/ \
           --voxel-size 0.005
    ```

*   **参数说明**:
    *   `--mcap`: 单个MCAP文件路径（与 `--mcap-dir` 二选一）
    *   `--mcap-dir`: MCAP文件所在目录（与 `--mcap` 二选一）
    *   `--calibration`: 第一步标定结果的YAML文件路径（**必需**）
    *   `--output`: 输出点云文件的目录
    *   `--voxel-size`: 点云下采样体素大小（米，可选），默认不下采样
    
*   **输出格式**:
    *   所有点云文件保存为 `.ply` 格式（带颜色和法向量信息）
    *   文件名规则：`{原始mcap名}_fused.ply`（例如：`demo_0_fused.ply`）

---

## 2. 关键可调参数说明

在实际环境下，您可能需要根据光照、物体材质和物理距离调整以下参数：

### 配置YAML文件中的参数 (`defaults/config_single_arm_dual_rgbd.yaml`)：
1.  **`serial_number`**: 必须替换为实际 D435i 相机底部的序列号，否则无法识别硬件。
    *   查询方法：`rs-enumerate-devices` 或 `python tests/list_cameras.py`
2.  **`width` / `height`**: 分辨率。`640x480` 是推荐的平衡性能与分辨率的选项。
3.  **`enable_depth`**: 启用深度流（固定为 `true`）
4.  **`align_depth`**: 深度对齐到RGB坐标系（固定为 `true`，确保同步）

### 运行时参数 (`main.py` 命令行)：
1.  **`sample_limit.size`**: 
    *   **作用**: 每次按Space采集的最大帧数
    *   **说明**: 系统会自动只保存第1帧RGBD和最后1帧关节角，中间帧会被过滤
2.  **`dataset.directory`**: 
    *   **作用**: 数据保存目录
    *   **建议**: 使用具有描述性的目录名，如 `data/pick_task_20260108`

### 点云转换脚本中的参数 (`scripts/data_convert/rgbd_to_pointcloud.py`)：
1.  **`--depth-min` 和 `--depth-max`** (默认: 0.1m ~ 5.0m):
    *   **作用**: 有效深度范围过滤。移除过近或过远的点云。
    *   **建议**: 根据实际工作环境调整。例如，在1米范围内工作可设为 `--depth-min 0.3 --depth-max 1.5`
2.  **`--voxel-size`** (默认: 无下采样):
    *   **作用**: 点云下采样的体素大小（单位：米）。
    *   **建议**: 
        - `0.005` (5mm)：平衡点云细节与文件大小
        - `0.01` (10mm)：快速处理和低内存占用
3.  **`--depth-scale`** (默认: 0.001):
    *   **作用**: 深度值缩放因子。Intel RealSense D435i 深度值单位为mm，所以默认为0.001。
    *   **建议**: 一般无需修改

---

## 3. 程序依赖与架构逻辑

系统的运行流程遵循以下层级依赖关系：


### 数据采集阶段的依赖链：

```
硬件驱动层 (底层)
    ├── librealsense (D435i相机驱动)
    └── airbot_py (机械臂驱动)
           ↓
    RealSenseObserver (相机观察者)  +  AirbotArm (机械臂lead/follow)
           ↓
状态机控制层 (中层)
    ├── FSM (有限状态机)
    │   状态流转: START → PREPARING → SAMPLING → SUCCESS
    └── KeyboardCallbackManager (键盘控制)
           ↓
数据采样层 (核心)
    └── McapSampler
        │ 功能：
        ├── 订阅两个相机的OBSERVATION消息（RGB+Depth）
        ├── 订阅从臂的关节角数据（joint_state）
        └── 在FSM采样信号时，将数据同步打包存入MCAP容器
           ↓
    存储格式 (MCAP容器)
    ├── RGB图像：H.264编码（压缩）
    ├── 深度图：RAW uint16（无损，保留原始精度）
    └── 关节角：json格式（原始数据）
```

### 数据处理阶段的依赖链：

```
MCAP文件 (采集阶段产出)
    ↓
rgbd_to_pointcloud.py (离线处理工具)
    ├── 输入1：calibration_result.yaml
    │   (包含T_2_1外参矩阵：相机2→相机1的坐标变换)
    ├── 输入2：MCAP文件
    │   (包含RGBD数据和内参信息)
    ├── 处理流程：
    │   1. 解码RGB图像和深度图
    │   2. 深度图单位转换 (mm → m)
    │   3. 每个相机分别生成点云
    │   4. 使用T_2_1将相机2的点云变换到相机1坐标系
    │   5. 融合两个点云 (可选体素下采样)
    └── 输出：.ply文件 (带颜色和法向量)
```

### 关键依赖关系总结：

| 阶段 | 依赖关系 | 必需参数 |
|------|--------|--------|
| **标定** | OpenCV + 两个D435i相机 | 棋盘格尺寸 |
| **采集** | lead/follow机械臂 + 两个D435i相机 | 相机序列号、机械臂端口 |
| **转换** | Open3D + 标定文件 | calibration.yaml |

### 整体数据流：

```
┌─────────────────────────────────────────────────────────────┐
│                    完整数据采集流程                          │
└─────────────────────────────────────────────────────────────┘
        ↓
   外参标定 (一次)
   calibrate_dual_cameras.py
        ↓
   保存: calibration_result.yaml (T_2_1矩阵)
        ↓
   ┌──────────────────────────────────────┐
   │   数据采集循环 (重复多次)              │
   │                                      │
   │  1. 启动主从机械臂跟随实例激活跟随      │
   │  2. 按Space开始采集                   │
   │  3. 保持静止0.5秒（记录第1帧）         │
   │  4. 操作主臂移动到抓取位姿             │
   │  5. 按Space停止采集（记录最后1帧）     │
   │  6. 按s保存数据                       │
   │                                      │
   │  产出: demo_0.mcap, demo_1.mcap ...  │
   │  (每个文件包含第1帧RGBD+最后1帧关节角)│
   └──────────────────────────────────────┘
        ↓
   点云转换处理
   rgbd_to_pointcloud.py (离线)
        ↓
   输出: demo_0_fused.ply, demo_1_fused.ply ...
        ↓
   用于后续的机器学习/3D重建等任务
```
