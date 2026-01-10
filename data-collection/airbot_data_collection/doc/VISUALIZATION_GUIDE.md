# 数据可视化工具使用指南

`scripts/visualize_sample.py` 用于交互式地检查数据处理的结果。它能同时展示 3D 点云与对应的机械臂关节状态。

## 1. 快速使用

### 最简模式 (即使只有点云)
只需指定点云文件，程序会自动尝试在同级目录（或 `../joints/`）下寻找同名的 JSON 关节角文件。

```bash
python scripts/visualize_sample.py --pointcloud pointclouds/demo_0_fused.ply
```

### 完整模式 (指定所有文件)
如果文件路径不标准，可以手动指定：

```bash
python scripts/visualize_sample.py \
    --pointcloud /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_03_pcd/0_fused.ply \
    --joints /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_03_joints/0.json
```

---

## 2. 交互操作说明 (Open3D 窗口)

| 操作 | 效果 |
| :--- | :--- |
| **左键拖拽** | 旋转视角 |
| **右键拖拽** | 平移视角 |
| **滚轮滚动** | 缩放 |
| **按键 `Q`** | 退出当前窗口 |
| **按键 `H`** | 显示帮助信息 |

---

## 3. 终端输出解读

程序启动后，终端会打印该样本的详细物理数值：

```text
============================================================
机械臂关节角:
============================================================
  Joint 1    :     0.5236 rad ( 30.0°)
  Joint 2    :    -0.3927 rad (-22.5°)
  ...
  Gripper    :     0.0850 m
============================================================
```
