#!/usr/bin/env python3
"""
RGBD图像转点云工具

功能：
1. 从MCAP文件中读取RGBD数据
2. 使用相机内参将深度图转换为点云
3. 支持点云融合到统一坐标系
4. 保存为.ply格式

使用方法：
    # 单个MCAP文件
    python rgbd_to_pointcloud.py \\
        --mcap data/0.mcap \\
        --calibration calibration.yaml \\
        --output pointclouds/

    # 批量处理
    python rgbd_to_pointcloud.py \\
        --mcap-dir dataset/ \\
        --calibration calibration.yaml \\
        --output pointclouds/

日期：2026-01-07
"""

import argparse
import yaml
import numpy as np
import open3d as o3d
from pathlib import Path
from typing import Tuple, Optional, Dict, List
import json
from mcap.reader import make_reader
import struct
from tqdm import tqdm


class RGBDToPointCloud:
    """RGBD图像转点云转换器"""
    
    def __init__(
        self,
        depth_scale: float = 1.0,  # 深度值到米的缩放因子
        depth_threshold: Tuple[float, float] = (0.1, 5.0),  # 深度过滤范围（米）
        voxel_size: Optional[float] = None,  # 体素下采样大小（米），None表示不下采样
    ):
        """
        初始化转换器
        
        Args:
            depth_scale: 深度值转换为米的缩放因子
                        RealSense深度值单位为mm，所以depth_scale=0.001
            depth_threshold: 有效深度范围 (min_m, max_m)
            voxel_size: 点云下采样体素大小（米）
        """
        self.depth_scale = depth_scale
        self.depth_threshold = depth_threshold
        self.voxel_size = voxel_size
        
        print("=" * 60)
        print("RGBD图像转点云工具")
        print("=" * 60)
        print(f"深度缩放因子: {depth_scale}")
        print(f"深度有效范围: {depth_threshold[0]:.2f}m - {depth_threshold[1]:.2f}m")
        if voxel_size:
            print(f"体素下采样: {voxel_size*1000:.1f}mm")
        print("=" * 60)
    
    def read_mcap_file(self, mcap_path: str) -> Dict:
        """
        读取MCAP文件并提取RGBD数据
        
        Args:
            mcap_path: MCAP文件路径
            
        Returns:
            包含RGBD数据和元信息的字典
        """
        print(f"\n[1/3] 读取MCAP文件: {mcap_path}")
        
        data = {
            'rgb': [],
            'depth': [],
            'intrinsics': {},
            'camera_names': [],
            'timestamps': [],
            'metadata': {}
        }
        
        # ===== 读取MCAP数据 =====
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f)
            
            # 读取所有通道和消息
            for schema, channel, message in reader.iter_messages():
                topic = channel.topic
                
                # 提取RGB图像
                if '/color/' in topic or '/image_raw' in topic and 'depth' not in topic:
                    # 解析压缩或原始RGB图像
                    rgb_data = self._decode_image_message(message, topic)
                    if rgb_data is not None:
                        camera_name = self._extract_camera_name(topic)
                        data['rgb'].append({
                            'camera': camera_name,
                            'image': rgb_data,
                            'timestamp': message.publish_time
                        })
                        if camera_name not in data['camera_names']:
                            data['camera_names'].append(camera_name)
                
                # 提取深度图
                elif 'depth' in topic and 'image' in topic:
                    depth_data = self._decode_depth_message(message)
                    if depth_data is not None:
                        camera_name = self._extract_camera_name(topic)
                        data['depth'].append({
                            'camera': camera_name,
                            'image': depth_data,
                            'timestamp': message.publish_time
                        })
            
            # 读取元数据（相机内参）
            for metadata_name, metadata_dict in reader.metadata.items():
                try:
                    meta_dict = json.loads(metadata_dict.get('intrinsics', '{}'))
                    for camera_name, intrinsics in meta_dict.items():
                        data['intrinsics'][camera_name] = intrinsics
                except:
                    pass
        
        print(f"  ✓ 读取完成")
        print(f"  - RGB图像: {len(data['rgb'])} 帧")
        print(f"  - 深度图: {len(data['depth'])} 帧")
        print(f"  - 相机: {', '.join(data['camera_names'])}")
        
        return data
    
    def _extract_camera_name(self, topic: str) -> str:
        """从主题名称中提取相机名称"""
        # 示例: "camera_front_top/color/image_raw" -> "camera_front_top"
        parts = topic.split('/')
        if len(parts) > 0:
            return parts[0]
        return "unknown"
    
    def _decode_image_message(self, message, topic: str):
        """解码RGB图像消息"""
        try:
            # 这里简化处理，实际应根据message格式解析
            # 通常message.data是已编码的图像数据
            if hasattr(message, 'data'):
                # 如果是h264或jpeg编码，需要解码
                # 这里假设已经是解码后的numpy数组
                return message.data
        except Exception as e:
            print(f"  ⚠ 解码RGB图像失败: {e}")
        return None
    
    def _decode_depth_message(self, message) -> Optional[np.ndarray]:
        """解码深度图消息"""
        try:
            if hasattr(message, 'data'):
                # 深度数据应为uint16格式
                depth = np.frombuffer(message.data, dtype=np.uint16)
                return depth
        except Exception as e:
            print(f"  ⚠ 解码深度图失败: {e}")
        return None
    
    def rgbd_to_pointcloud(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        intrinsics: Dict
    ) -> o3d.geometry.PointCloud:
        """
        将RGBD图像转换为点云
        
        Args:
            rgb: RGB图像 (H, W, 3) uint8
            depth: 深度图 (H, W) uint16，单位mm
            intrinsics: 相机内参 {'fx': ..., 'fy': ..., 'cx': ..., 'cy': ...}
            
        Returns:
            Open3D点云对象
        """
        # ===== RGBD转点云 =====
        
        # 验证输入
        assert rgb.shape[:2] == depth.shape, "RGB和深度图尺寸不匹配"
        
        height, width = depth.shape
        
        # 转换深度图单位（mm → m）
        depth_m = depth.astype(np.float32) * self.depth_scale / 1000.0
        
        # 深度过滤
        valid_mask = (depth_m >= self.depth_threshold[0]) & (depth_m <= self.depth_threshold[1])
        
        # 创建RGBD图像对象
        rgb_o3d = o3d.geometry.Image(rgb)
        depth_o3d = o3d.geometry.Image(depth_m)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_o3d,
            depth_o3d,
            depth_scale=1.0,  # 已转换为米
            convert_rgb_to_intensity=False
        )
        
        # 创建相机内参对象
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=width,
            height=height,
            fx=intrinsics.get('fx', 615.0),
            fy=intrinsics.get('fy', 615.0),
            cx=intrinsics.get('cx', width/2),
            cy=intrinsics.get('cy', height/2)
        )
        
        # 生成点云
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
        
        # 应用有效性掩码过滤
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        
        valid_indices = valid_mask.flatten()
        filtered_points = points[valid_indices]
        filtered_colors = colors[valid_indices]
        
        pcd.points = o3d.utility.Vector3dVector(filtered_points)
        pcd.colors = o3d.utility.Vector3dVector(filtered_colors)
        
        # 体素下采样
        if self.voxel_size:
            pcd = pcd.voxel_down_sample(self.voxel_size)
        
        return pcd
    
    def fuse_pointclouds(
        self,
        pcd1: o3d.geometry.PointCloud,
        pcd2: o3d.geometry.PointCloud,
        T_2_1: np.ndarray
    ) -> o3d.geometry.PointCloud:
        """
        融合两个点云到统一坐标系
        
        Args:
            pcd1: 相机1的点云（参考坐标系）
            pcd2: 相机2的点云
            T_2_1: 从相机1到相机2的4x4变换矩阵
            
        Returns:
            融合后的点云
        """
        # ===== 添加代码标记 - 点云融合 =====
        print("  正在融合两个点云...")
        
        # 将pcd2变换到pcd1坐标系
        pcd2_transformed = pcd2.transform(T_2_1)
        
        # 合并点云
        fused_pcd = pcd1 + pcd2_transformed
        
        # 移除重复点（可选，但推荐）
        # fused_pcd = fused_pcd.remove_duplicated_points()
        
        return fused_pcd
    
    def save_pointcloud(
        self,
        pcd: o3d.geometry.PointCloud,
        output_path: str
    ):
        """
        保存点云为PLY格式
        
        Args:
            pcd: Open3D点云对象
            output_path: 输出文件路径
        """
        # ===== 添加代码标记 - 保存点云 =====
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # 保存为PLY格式（包含颜色和法向量）
        o3d.io.write_point_cloud(str(output_path), pcd)
        print(f"  ✓ 点云已保存: {output_path} ({len(pcd.points)} 点)")
    
    def process_single_mcap(
        self,
        mcap_path: str,
        output_dir: str,
        calibration_path: str
    ):
        """
        处理单个MCAP文件
        
        Args:
            mcap_path: MCAP文件路径
            output_dir: 输出目录
            calibration_path: 标定文件路径（必需）
        """
        # 读取MCAP数据
        data = self.read_mcap_file(mcap_path)
        
        print(f"\n[2/3] 转换为点云（双相机融合模式）...")
        
        # 读取标定结果
        with open(calibration_path, 'r') as f:
            calib_data = yaml.safe_load(f)
        
        T_2_1 = np.array(calib_data['extrinsics']['transformation_matrix'])
        
        # 获取两个相机的数据
        cameras = list(set([item['camera'] for item in data['rgb']]))
        if len(cameras) < 2:
            raise ValueError(f"需要双相机，但只找到 {len(cameras)} 个相机")
        
        camera1 = cameras[0]
        camera2 = cameras[1]
        
        # 获取RGB和深度
        rgb1 = next(r for r in data['rgb'] if r['camera'] == camera1)['image']
        depth1 = next(d for d in data['depth'] if d['camera'] == camera1)['image']
        
        rgb2 = next(r for r in data['rgb'] if r['camera'] == camera2)['image']
        depth2 = next(d for d in data['depth'] if d['camera'] == camera2)['image']
        
        # 获取相机内参
        intrinsics1 = data['intrinsics'].get(camera1, {})
        intrinsics2 = data['intrinsics'].get(camera2, {})
        
        # 转换为点云
        pcd1 = self.rgbd_to_pointcloud(rgb1, depth1, intrinsics1)
        pcd2 = self.rgbd_to_pointcloud(rgb2, depth2, intrinsics2)
        
        print(f"  点云1 ({camera1}): {len(pcd1.points)} 点")
        print(f"  点云2 ({camera2}): {len(pcd2.points)} 点")
        
        # 融合
        fused_pcd = self.fuse_pointclouds(pcd1, pcd2, T_2_1)
        print(f"  融合后: {len(fused_pcd.points)} 点")
        
        # 保存（PLY格式）
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        mcap_name = Path(mcap_path).stem
        output_file = output_dir / f"{mcap_name}_fused.ply"
        self.save_pointcloud(fused_pcd, str(output_file))
        
        print(f"\n[3/3] 完成！")
        print(f"  输出目录: {output_dir}")


def main():
    # ===== 添加代码标记 - 命令行参数解析 =====
    parser = argparse.ArgumentParser(
        description="RGBD图像转点云工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 单个MCAP文件（双相机融合）
  python rgbd_to_pointcloud.py \\
      --mcap data/0.mcap \\
      --calibration calibration.yaml \\
      --output pointclouds/
  
  # 批量处理（目录）
  python rgbd_to_pointcloud.py \\
      --mcap-dir dataset/ \\
      --calibration calibration.yaml \\
      --output pointclouds/
  
  # 指定体素下采样
  python rgbd_to_pointcloud.py \\
      --mcap data/0.mcap \\
      --calibration calibration.yaml \\
      --output pointclouds/ \\
      --voxel-size 0.005  # 5mm体素
        """
    )
    
    parser.add_argument('--mcap', type=str,
                       help='单个MCAP文件路径')
    parser.add_argument('--mcap-dir', type=str,
                       help='MCAP文件目录（用于批量处理）')
    parser.add_argument('--calibration', type=str, required=True,
                       help='相机标定文件路径（必需）')
    parser.add_argument('--output', type=str, required=True,
                       help='输出目录')
    parser.add_argument('--depth-scale', type=float, default=0.001,
                       help='深度值缩放因子（RealSense为0.001），默认: 0.001')
    parser.add_argument('--depth-min', type=float, default=0.1,
                       help='最小深度阈值（米），默认: 0.1')
    parser.add_argument('--depth-max', type=float, default=5.0,
                       help='最大深度阈值（米），默认: 5.0')
    parser.add_argument('--voxel-size', type=float,
                       help='体素下采样大小（米），None表示不下采样')
    
    args = parser.parse_args()
    
    # 验证参数
    if args.mcap is None and args.mcap_dir is None:
        parser.error("必须指定 --mcap 或 --mcap-dir 中的一个")
    
    if args.mcap and args.mcap_dir:
        parser.error("只能指定 --mcap 或 --mcap-dir 中的一个")
    
    # 创建转换器
    converter = RGBDToPointCloud(
        depth_scale=args.depth_scale,
        depth_threshold=(args.depth_min, args.depth_max),
        voxel_size=args.voxel_size
    )
    
    try:
        # 处理文件
        if args.mcap:
            # 单个MCAP文件
            converter.process_single_mcap(
                mcap_path=args.mcap,
                output_dir=args.output,
                calibration_path=args.calibration
            )
        
        else:  # args.mcap_dir
            # 批量处理
            mcap_dir = Path(args.mcap_dir)
            mcap_files = sorted(mcap_dir.glob('*.mcap'))
            
            print(f"找到 {len(mcap_files)} 个MCAP文件")
            
            for mcap_file in tqdm(mcap_files, desc="处理进度"):
                try:
                    converter.process_single_mcap(
                        mcap_path=str(mcap_file),
                        output_dir=args.output,
                        calibration_path=args.calibration
                    )
                except Exception as e:
                    print(f"\n  ✗ 处理 {mcap_file} 失败: {e}")
    
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
