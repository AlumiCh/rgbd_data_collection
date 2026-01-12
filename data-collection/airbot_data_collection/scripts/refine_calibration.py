#!/usr/bin/env python3
"""
相机外参精修工具 (Refine Calibration Tool)

功能：
1. 加载 MCAP 采集样本和已有的标定 YAML 文件。
2. 实时显示双相机点云融合效果。
3. 支持一键 ICP 自动精细对齐。
4. 支持键盘交互式手动微调位姿（平移和旋转）。
5. 将精修后的变换矩阵保存回 YAML 文件。

交互快捷键：
  - 移动: Q-W (X轴), A-S (Y轴), Z-X (Z轴)
  - 旋转: E-R (Roll), D-F (Pitch), C-V (Yaw)
  - 步长: 1/2 (增加/减小移动步长), 3/4 (增加/减小旋转步长)
  - 功能: I (执行ICP), O (保存到YAML), H (打印帮助)
  - 退出: Esc / Q

日期：2026-01-12
"""

import argparse
import yaml
import numpy as np
import open3d as o3d
from pathlib import Path
import json
import sys
import copy

# 导入 mcap_to_dataset 中的逻辑以复用
from airbot_data_collection.scripts.data_convert.mcap_to_dataset import RGBDToPointCloud

class CalibrationRefiner:
    def __init__(self, mcap_path: str, calibration_path: str):
        self.mcap_path = Path(mcap_path)
        self.calibration_path = Path(calibration_path)
        
        # 加载标定数据
        with open(self.calibration_path, 'r') as f:
            self.calib_data = yaml.safe_load(f)
        
        # T_2_1: 从相机1到相机2的变换
        self.T_2_1 = np.array(self.calib_data['extrinsics']['transformation_matrix'])
        # 我们可视化时通常将相机2变换到相机1，即 T_1_2 = inv(T_2_1)
        self.T_1_2 = np.linalg.inv(self.T_2_1)
        
        # 转换器用于提取点云
        self.converter = RGBDToPointCloud(depth_threshold=(0.1, 1.5))
        
        # 数据存储
        self.pcd1 = None
        self.pcd2 = None
        self.pcd2_transformed = None
        
        # 调试/显示参数
        self.trans_step = 0.005  # 5mm
        self.rot_step = 0.5      # 0.5 deg
        
        print(f"\n[初始化] 标定文件: {self.calibration_path}")
        print(f"[初始化] 样本文件: {self.mcap_path}")
        print("-" * 40)
        self._print_help()

    def _print_help(self):
        print("\n=== 键盘交互说明 ===")
        print("  移动 (轴):  Q/W (X),  A/S (Y),  Z/X (Z)")
        print("  旋转 (角):  E/R (Roll), D/F (Pitch), C/V (Yaw)")
        print("  步长控制:   1/2 (移动步长 ±), 3/4 (旋转步长 ±)")
        print("  自动化:     I (执行ICP精修)")
        print("  保存:       O (覆盖保存到YAML)")
        print("  帮助:       H (显示此列表)")
        print("  退出:       Esc / Q")
        print("-" * 40)

    def load_samples(self):
        """从MCAP提取并生成点云"""
        print("\n[读取数据] 正在从 MCAP 提取第一帧点云...")
        data = self.converter.read_mcap_file(str(self.mcap_path))
        
        cameras = sorted(data['camera_names'])
        if len(cameras) < 2:
            raise ValueError("MCAP 中至少需要 2 个相机数据")
            
        cam1, cam2 = cameras[0], cameras[1]
        
        # 获取第一帧图像和深度
        rgb1 = [x for x in data['rgb'] if x['camera'] == cam1][0]['image']
        depth1 = [x for x in data['depth'] if x['camera'] == cam1][0]['image']
        rgb2 = [x for x in data['rgb'] if x['camera'] == cam2][0]['image']
        depth2 = [x for x in data['depth'] if x['camera'] == cam2][0]['image']
        
        # 获取内参 (优先使用标定文件的)
        intr1 = self.calib_data.get('camera1', {}).get('intrinsics') or data['intrinsics'].get(cam1)
        intr2 = self.calib_data.get('camera2', {}).get('intrinsics') or data['intrinsics'].get(cam2)
        
        # 生成点云 (相机2会被频繁变换)
        self.pcd1 = self.converter.rgbd_to_pointcloud(rgb1, depth1, intr1)
        self.pcd2 = self.converter.rgbd_to_pointcloud(rgb2, depth2, intr2)
        
        # 初始变换
        self.pcd2_transformed = copy.deepcopy(self.pcd2).transform(self.T_1_2)
        print(f"  ✓ 加载完成。点云1: {len(self.pcd1.points)}, 点云2: {len(self.pcd2.points)}")

    def save(self):
        """保存当前 T_1_2 的逆 (即 T_2_1) 到 YAML"""
        print("\n[保存] 正在更新标定文件...")
        
        # 更新 T_2_1
        self.T_2_1 = np.linalg.inv(self.T_1_2)
        self.calib_data['extrinsics']['transformation_matrix'] = self.T_2_1.tolist()
        
        # 更新旋转矩阵/向量/欧拉角
        R = self.T_2_1[:3, :3]
        t = self.T_2_1[:3, 3]
        self.calib_data['extrinsics']['rotation_matrix'] = R.tolist()
        self.calib_data['extrinsics']['translation_vector'] = t.tolist()
        
        # 计算欧拉角 (roll, pitch, yaw)
        sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
        singular = sy < 1e-6
        if not singular:
            roll = np.arctan2(R[2,1], R[2,2])
            pitch = np.arctan2(-R[2,0], sy)
            yaw = np.arctan2(R[1,0], R[0,0])
        else:
            roll = np.arctan2(-R[1,2], R[1,1])
            pitch = np.arctan2(-R[2,0], sy)
            yaw = 0
        self.calib_data['extrinsics']['rotation_euler_deg'] = np.degrees([roll, pitch, yaw]).tolist()
        
        # 写入文件
        with open(self.calibration_path, 'w', encoding='utf-8') as f:
            yaml.dump(self.calib_data, f, default_flow_style=False)
        print(f"  ✓ 标定已覆盖保存至: {self.calibration_path}")

    def run_icp(self):
        """执行自动 ICP"""
        print("\n[ICP] 启动自动优化...")
        new_T = self.converter.run_icp_registration(self.pcd2, self.pcd1, self.T_1_2)
        self.T_1_2 = new_T
        self.update_view()

    def update_pose(self, dt=0, dp=0, dr=0, dx=0, dy=0, dz=0):
        """更新当前 T_1_2 并刷新点云"""
        # 构建增量变换
        T_delta = np.eye(4)
        
        # 旋转 (Euler)
        if dr or dp or dt:
            r = np.radians(dr)
            p = np.radians(dp)
            y = np.radians(dt)
            # 使用 Open3D 的旋转习惯
            R_delta = o3d.geometry.get_rotation_matrix_from_xyz((r, p, y))
            T_delta[:3, :3] = R_delta
            
        # 平移
        T_delta[0, 3] = dx
        T_delta[1, 3] = dy
        T_delta[2, 3] = dz
        
        # 应用变换 (相对于当前 T_1_2)
        # 注意：这里我们是在参考坐标系(Camera1)下微调，所以左乘 T_delta
        self.T_1_2 = T_delta @ self.T_1_2
        self.update_view()

    def update_view(self):
        """重新变换点云并更新可视化"""
        self.pcd2_transformed.points = self.pcd2.points
        self.pcd2_transformed.transform(self.T_1_2)
        # 通知可视化器更新
        self.vis.update_geometry(self.pcd2_transformed)
        print(f"\r  当前位姿 [x={self.T_1_2[0,3]:.4f}, y={self.T_1_2[1,3]:.4f}, z={self.T_1_2[2,3]:.4f}] | 步长(T): {self.trans_step*1000:.1f}mm, (R): {self.rot_step:.1f}deg", end="")

    def run(self):
        self.load_samples()
        
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="相机外参微调工具", width=1280, height=720)
        
        # 添加几何体
        self.vis.add_geometry(self.pcd1)
        self.vis.add_geometry(self.pcd2_transformed)
        
        # 坐标系参考轴
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        self.vis.add_geometry(coord)
        
        # 注册回调
        # 翻译
        self.vis.register_key_callback(ord('Q'), lambda v: self.update_pose(dx=self.trans_step))
        self.vis.register_key_callback(ord('W'), lambda v: self.update_pose(dx=-self.trans_step))
        self.vis.register_key_callback(ord('A'), lambda v: self.update_pose(dy=self.trans_step))
        self.vis.register_key_callback(ord('S'), lambda v: self.update_pose(dy=-self.trans_step))
        self.vis.register_key_callback(ord('Z'), lambda v: self.update_pose(dz=self.trans_step))
        self.vis.register_key_callback(ord('X'), lambda v: self.update_pose(dz=-self.trans_step))
        
        # 旋转
        self.vis.register_key_callback(ord('E'), lambda v: self.update_pose(dr=self.rot_step))
        self.vis.register_key_callback(ord('R'), lambda v: self.update_pose(dr=-self.rot_step))
        self.vis.register_key_callback(ord('D'), lambda v: self.update_pose(dp=self.rot_step))
        self.vis.register_key_callback(ord('F'), lambda v: self.update_pose(dp=-self.rot_step))
        self.vis.register_key_callback(ord('C'), lambda v: self.update_pose(dt=self.rot_step))
        self.vis.register_key_callback(ord('V'), lambda v: self.update_pose(dt=-self.rot_step))
        
        # 步长切换
        def change_step(t=0, r=0):
            if t > 0: self.trans_step *= 2
            if t < 0: self.trans_step /= 2
            if r > 0: self.rot_step *= 2
            if r < 0: self.rot_step /= 2
            self.update_view()
            
        self.vis.register_key_callback(ord('1'), lambda v: change_step(t=1))
        self.vis.register_key_callback(ord('2'), lambda v: change_step(t=-1))
        self.vis.register_key_callback(ord('3'), lambda v: change_step(r=1))
        self.vis.register_key_callback(ord('4'), lambda v: change_step(r=-1))
        
        # 功能
        self.vis.register_key_callback(ord('I'), lambda v: self.run_icp())
        self.vis.register_key_callback(ord('O'), lambda v: self.save())
        self.vis.register_key_callback(ord('H'), lambda v: self._print_help())
        
        print("\n[可视化] 窗口已打开，请使用键盘进行调整...")
        self.vis.run()
        self.vis.destroy_window()

def main():
    parser = argparse.ArgumentParser(description="相机外参微调工具")
    parser.add_argument("--mcap", type=str, required=True, help="采集的一个样本MCAP文件")
    parser.add_argument("--yaml", type=str, required=True, help="待优化的标定YAML文件")
    args = parser.parse_args()
    
    refiner = CalibrationRefiner(args.mcap, args.yaml)
    refiner.run()

if __name__ == "__main__":
    main()
