#!/usr/bin/env python3
"""
MCAP数据处理工具

功能：
1. 从 MCAP 文件中读取 RGBD 数据和机械臂关节角数据
2. 使用相机内参将深度图转换为点云
3. 支持点云融合到统一坐标系
4. 提取并保存机械臂关节角数据

使用方法：
    # 单个 MCAP 文件
    python mcap_to_dataset.py \\
        --mcap data/0.mcap \\
        --calibration calibration.yaml \\
        --pointcloud-output pointclouds/ \\
        --joints-output joints/

    # 批量处理
    python mcap_to_dataset.py \\
        --mcap-dir dataset/ \\
        --calibration calibration.yaml \\
        --pointcloud-output pointclouds/ \\
        --joints-output joints/

日期：2026-01-08
"""

import argparse
import yaml
import numpy as np
import open3d as o3d
from pathlib import Path
from typing import Tuple, Optional, Dict, List
import json
import re
from mcap.reader import make_reader
import struct
from tqdm import tqdm

# 导入FlatBuffer schemas用于解码
try:
    from foxglove_schemas_flatbuffer.RawImage import RawImage
    from foxglove_schemas_flatbuffer.CompressedImage import CompressedImage
except ImportError:
    print("警告: 未安装 foxglove_schemas_flatbuffer，图像解码可能失败")
    RawImage = None
    CompressedImage = None

try:
    from turbojpeg import TurboJPEG
    jpeg_decoder = TurboJPEG()
except ImportError:
    print("警告: 未安装 turbojpeg ，JPEG 解码将使用 OpenCV")
    jpeg_decoder = None
    import cv2


class RGBDToPointCloud:
    """RGBD图像转点云转换器"""
    
    def __init__(
        self,
        depth_scale: float = 0.001,  # 深度值到米的缩放因子
        depth_threshold: Tuple[float, float] = (0.1, 5.0),  # 深度过滤范围（米）
        voxel_size: Optional[float] = None,  # 体素下采样大小（米），None表示不下采样
    ):
        """
        初始化转换器
        
        Args:
            depth_scale: 深度值转换为米的缩放因子
                        RealSense 深度值单位为 mm，所以depth_scale=0.001
            depth_threshold: 有效深度范围 (min_m, max_m)
            voxel_size: 点云下采样体素大小（米）
        """
        self.depth_scale = depth_scale
        self.depth_threshold = depth_threshold
        self.voxel_size = voxel_size
        
        print("=" * 60)
        print("MCAP 数据处理工具")
        print("=" * 60)
        print(f"深度缩放因子: {depth_scale}")
        print(f"深度有效范围: {depth_threshold[0]:.2f}m - {depth_threshold[1]:.2f}m")
        if voxel_size:
            print(f"体素下采样: {voxel_size*1000:.1f}mm")
        print("=" * 60)
    
    def read_mcap_file(self, mcap_path: str) -> Dict:
        """
        读取 MCAP 文件并提取 RGBD 数据
        
        Args:
            mcap_path: MCAP 文件路径
            
        Returns:
            包含 RGBD 数据和元信息的字典
        """
        print(f"\n[1/4] 读取 MCAP 文件: {mcap_path}")
        
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
                schema_name = schema.name if schema else "unknown"
                
                # 提取RGB图像
                if '/color/' in topic and 'image' in topic:
                    rgb_data = self._decode_image_message(message, schema_name)
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
                    depth_data = self._decode_depth_message(message, schema_name)
                    if depth_data is not None:
                        camera_name = self._extract_camera_name(topic)
                        data['depth'].append({
                            'camera': camera_name,
                            'image': depth_data,
                            'timestamp': message.publish_time
                        })
            
            # 读取元数据（相机内参）
            for record in reader.iter_metadata():
                metadata_name = record.name
                metadata_dict = record.metadata
                try:
                    if 'intrinsics' in metadata_dict:
                        meta_dict = json.loads(metadata_dict.get('intrinsics', '{}'))
                        for camera_name, intrinsics in meta_dict.items():
                            data['intrinsics'][camera_name] = intrinsics
                except:
                    pass
        
        print(f"  读取完成")
        print(f"  - RGB 图像: {len(data['rgb'])} 帧")
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
    
    def _decode_image_message(self, message, schema_name: str) -> Optional[np.ndarray]:
        """解码 RGB 图像消息"""
        try:
            # 解析FlatBuffer消息
            if schema_name == "foxglove.RawImage" and RawImage:
                raw_img = RawImage.GetRootAs(message.data, 0)
                width = raw_img.Width()
                height = raw_img.Height()
                encoding = raw_img.Encoding()
                if encoding:
                    encoding = encoding.decode('utf-8')
                
                # 提取图像数据
                data_length = raw_img.DataLength()
                data_bytes = raw_img.DataAsNumpy().tobytes()
                
                # 根据编码解析
                if '8UC3' in encoding or 'rgb8' in encoding or 'bgr8' in encoding:
                    img = np.frombuffer(data_bytes, dtype=np.uint8).reshape(height, width, 3)
                    # RealSense默认RGB格式
                    return img
                elif '8UC1' in encoding or 'mono8' in encoding:
                    img = np.frombuffer(data_bytes, dtype=np.uint8).reshape(height, width)
                    return img
                else:
                    print(f"未知图像编码: {encoding}")
                    return None
            
            elif schema_name == "foxglove.CompressedImage" and CompressedImage:
                compressed_img = CompressedImage.GetRootAs(message.data, 0)
                format_str = compressed_img.Format()
                if format_str:
                    format_str = format_str.decode('utf-8')
                
                # 提取压缩数据
                data_bytes = compressed_img.DataAsNumpy().tobytes()
                
                # 解码压缩图像
                if format_str == 'jpeg' or format_str == 'jpg':
                    if jpeg_decoder:
                        img = jpeg_decoder.decode(data_bytes)
                    else:
                        img = cv2.imdecode(np.frombuffer(data_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    return img
                elif format_str == 'png':
                    img = cv2.imdecode(np.frombuffer(data_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    return img
                else:
                    print(f"未知压缩格式: {format_str}")
                    return None
            else:
                print(f"未知Schema: {schema_name}")
                return None
        except Exception as e:
            print(f"解码 RGB 图像失败: {e}")
            import traceback
            traceback.print_exc()
        return None
    
    def _decode_depth_message(self, message, schema_name: str) -> Optional[np.ndarray]:
        """解码深度图消息"""
        try:
            if schema_name == "foxglove.RawImage" and RawImage:
                raw_img = RawImage.GetRootAs(message.data, 0)
                width = raw_img.Width()
                height = raw_img.Height()
                encoding = raw_img.Encoding()
                if encoding:
                    encoding = encoding.decode('utf-8')
                
                # 提取深度数据
                data_bytes = raw_img.DataAsNumpy().tobytes()
                
                # 深度图通常是16位无符号整数
                if '16UC1' in encoding or '16U' in encoding:
                    depth = np.frombuffer(data_bytes, dtype=np.uint16).reshape(height, width)
                    return depth
                else:
                    print(f"深度图编码不是 16UC1: {encoding}")
                    # 尝试作为uint16解析
                    depth = np.frombuffer(data_bytes, dtype=np.uint16)
                    # 尝试reshape
                    if len(depth) == width * height:
                        return depth.reshape(height, width)
                    else:
                        print(f"深度数据大小不匹配: {len(depth)} vs {width*height}")
                        return None
            else:
                print(f"未知深度图 Schema: {schema_name}")
                return None
        except Exception as e:
            print(f"解码深度图失败: {e}")
            import traceback
            traceback.print_exc()
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
        depth_m = depth.astype(np.float32) * self.depth_scale
        
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
            fx=intrinsics.get('fx'),
            fy=intrinsics.get('fy'),
            cx=intrinsics.get('cx'),
            cy=intrinsics.get('cy')
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
    
    def extract_joint_angles(self, mcap_path: str) -> Optional[Dict[str, np.ndarray]]:
        """
        从MCAP文件中提取机械臂关节角数据
        
        支持的主题模式：
        - 单臂系统: /arm/joint_state/position, /eef/joint_state/position
        - 双臂系统: /left_arm/joint_state/position, /right_arm/joint_state/position
        - 带前缀: mmk/observation/arm/joint_state/position
        
        数据格式：FlatBuffer airbot_fbs.FloatArray
        
        Args:
            mcap_path: MCAP文件路径
            
        Returns:
            关节角字典 {topic: array} 或 None
            示例: {'/arm/joint_state/position': array([0.1, 0.2, ...], dtype=float32)}
        """
        print(f"  提取关节角...")
        
        from airbot_data_collection.airbot.schemas.airbot_fbs.FloatArray import FloatArray
        
        # 目标主题模式 - 按优先级排序
        target_patterns = [
            # 观察主题（优先）
            r'.*/observation/.*/joint_state/position$',
            r'.*/observation/.*/joint_state$',
            # 标准主题
            r'^/[a-z_]+arm/joint_state/position$',
            r'^/eef/joint_state/position$',
            r'^/arm/joint_state/position$',
            # 动作主题（次优先）
            r'.*/action/.*/joint_state/position$',
        ]
        
        joint_data = {}  # {topic: [arrays]}
        schema_counts = {}  # 统计每个schema的消息数
        
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f)
            
            for schema, channel, message in reader.iter_messages():
                topic = channel.topic
                schema_name = schema.name if schema else "unknown"
                
                # 统计schema
                schema_counts[schema_name] = schema_counts.get(schema_name, 0) + 1
                
                # 检查是否匹配关节角主题
                is_joint_topic = any(
                    re.match(pattern, topic) 
                    for pattern in target_patterns
                )
                
                if not is_joint_topic:
                    continue
                
                try:
                    # 解析 FlatBuffer FloatArray
                    if schema_name == "airbot_fbs.FloatArray":
                        float_array = FloatArray.GetRootAs(message.data, 0)
                        values = float_array.ValuesAsNumpy()
                        
                        if topic not in joint_data:
                            joint_data[topic] = []
                        joint_data[topic].append(values)
                    
                    # 兼容 JSON 格式（备用）
                    elif schema_name in ["json", "unknown"]:
                        try:
                            data = json.loads(message.data.decode('utf-8'))
                            if isinstance(data, dict) and 'position' in data:
                                values = np.array(data['position'], dtype=np.float32)
                            elif isinstance(data, (list, tuple)):
                                values = np.array(data, dtype=np.float32)
                            else:
                                continue
                            
                            if topic not in joint_data:
                                joint_data[topic] = []
                            joint_data[topic].append(values)
                        except:
                            continue
                
                except Exception as e:
                    print(f"解析主题 {topic} 失败: {e}")
                    continue
        
        # 输出统计信息
        print(f"    Schema统计: {dict(schema_counts)}")
        
        # 处理结果
        if joint_data:
            result = {}
            for topic, arrays in joint_data.items():
                # 取最后一条消息（通常是最终状态）
                final_array = arrays[-1]
                result[topic] = final_array
                print(f"{topic}: {len(arrays)} 条消息, 最终状态 {len(final_array)} 维")
            return result
        else:
            print(f"未找到关节角数据")
            print(f"    提示: 检查 MCAP 文件中是否包含以下主题：")
            print(f"         - /arm/joint_state/position")
            print(f"         - mmk/observation/*/joint_state/position")
            return None
    
    def save_joint_angles(
        self,
        joint_angles: Dict[str, np.ndarray],
        output_path: str
    ):
        """
        ===== 保存关节角 (2026-01-08) =====
        
        将关节角数据保存为 JSON 文件
        
        Args:
            joint_angles: 关节角字典 {topic: array}
            output_path: 输出JSON文件路径
        """
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # 转换为可序列化格式
        serializable_data = {
            topic: angles.tolist()
            for topic, angles in joint_angles.items()
        }
        
        # 添加元数据
        output_data = {
            'topics': serializable_data,
            'metadata': {
                'num_topics': len(joint_angles),
                'topic_names': list(joint_angles.keys()),
                'dimensions': {
                    topic: len(angles)
                    for topic, angles in joint_angles.items()
                }
            }
        }
        
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=2, ensure_ascii=False)
        
        print(f"关节角已保存: {output_path}")
        for topic in joint_angles.keys():
            print(f"    - {topic}")
    
    def process_single_mcap(
        self,
        mcap_path: str,
        pointcloud_output_dir: str,
        joints_output_dir: str,
        calibration_path: str
    ):
        """
        处理单个 MCAP 文件
        
        Args:
            mcap_path: MCAP 文件路径
            pointcloud_output_dir: 点云输出目录
            joints_output_dir: 关节角输出目录
            calibration_path: 标定文件路径（必需）
        """
        # 读取MCAP数据
        data = self.read_mcap_file(mcap_path)
        
        print(f"\n[2/4] 转换为点云（双相机融合模式）...")
        
        # 读取标定结果
        with open(calibration_path, 'r') as f:
            calib_data = yaml.safe_load(f)
        
        T_2_1 = np.array(calib_data['extrinsics']['transformation_matrix'])
        
        # 获取两个相机的数据
        cameras = sorted(set([item['camera'] for item in data['rgb']]))
        if len(cameras) < 2:
            raise ValueError(f"需要双相机，但只找到 {len(cameras)} 个相机: {cameras}")
        
        camera1 = cameras[0]
        camera2 = cameras[1]
        
        # 验证每个相机都有 RGB 和深度数据
        rgb1_list = [r for r in data['rgb'] if r['camera'] == camera1]
        depth1_list = [d for d in data['depth'] if d['camera'] == camera1]
        rgb2_list = [r for r in data['rgb'] if r['camera'] == camera2]
        depth2_list = [d for d in data['depth'] if d['camera'] == camera2]
        
        if not rgb1_list:
            raise ValueError(f"相机 {camera1} 没有 RGB 数据")
        if not depth1_list:
            raise ValueError(f"相机 {camera1} 没有深度数据")
        if not rgb2_list:
            raise ValueError(f"相机 {camera2} 没有 RGB 数据")
        if not depth2_list:
            raise ValueError(f"相机 {camera2} 没有深度数据")
        
        # 获取 RGB 和深度
        rgb1 = rgb1_list[0]['image']
        depth1 = depth1_list[0]['image']
        rgb2 = rgb2_list[0]['image']
        depth2 = depth2_list[0]['image']
        
        # 获取相机内参
        intrinsics1 = data['intrinsics'].get(camera1)
        intrinsics2 = data['intrinsics'].get(camera2)
        
        if not intrinsics1:
            raise ValueError(f"相机 {camera1} 没有内参数据")
        if not intrinsics2:
            raise ValueError(f"相机 {camera2} 没有内参数据")
        
        print(f"  相机1: {camera1} - RGB{rgb1.shape}, Depth{depth1.shape}")
        print(f"  相机2: {camera2} - RGB{rgb2.shape}, Depth{depth2.shape}")
        
        # 转换为点云
        pcd1 = self.rgbd_to_pointcloud(rgb1, depth1, intrinsics1)
        pcd2 = self.rgbd_to_pointcloud(rgb2, depth2, intrinsics2)
        
        print(f"  点云1 ({camera1}): {len(pcd1.points)} 点")
        print(f"  点云2 ({camera2}): {len(pcd2.points)} 点")
        
        # 融合
        fused_pcd = self.fuse_pointclouds(pcd1, pcd2, T_2_1)
        print(f"  融合后: {len(fused_pcd.points)} 点")
        
        # 提取并保存关节角
        print(f"\n[3/4] 提取机械臂关节角...")
        joint_angles = self.extract_joint_angles(mcap_path)
        
        # 保存点云
        print(f"\n[4/4] 保存数据...")
        pointcloud_output_dir = Path(pointcloud_output_dir)
        pointcloud_output_dir.mkdir(parents=True, exist_ok=True)
        mcap_name = Path(mcap_path).stem
        
        pointcloud_file = pointcloud_output_dir / f"{mcap_name}_fused.ply"
        self.save_pointcloud(fused_pcd, str(pointcloud_file))
        
        # 保存关节角
        if joint_angles is not None:
            joints_output_dir = Path(joints_output_dir)
            joints_output_dir.mkdir(parents=True, exist_ok=True)
            joints_file = joints_output_dir / f"{mcap_name}.json"
            self.save_joint_angles(joint_angles, str(joints_file))
        
        print(f"\n处理完成！")
        print(f"  点云目录: {pointcloud_output_dir}")
        print(f"  关节角目录: {joints_output_dir}")


def main():
    # ===== 命令行参数解析 =====
    parser = argparse.ArgumentParser(
        description="MCAP 数据处理工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 单个 MCAP 文件（点云+关节角）
  python mcap_to_dataset.py \\
      --mcap data/0.mcap \\
      --calibration calibration.yaml \\
      --pointcloud-output pointclouds/ \\
      --joints-output joints/
  
  # 批量处理（目录）
  python mcap_to_dataset.py \\
      --mcap-dir dataset/ \\
      --calibration calibration.yaml \\
      --pointcloud-output pointclouds/ \\
      --joints-output joints/
  
  # 指定体素下采样
  python mcap_to_dataset.py \\
      --mcap data/0.mcap \\
      --calibration calibration.yaml \\
      --pointcloud-output pointclouds/ \\
      --joints-output joints/ \\
      --voxel-size 0.005  # 5mm体素
        """
    )
    
    parser.add_argument('--mcap', type=str,
                       help='单个 MCAP 文件路径')
    parser.add_argument('--mcap-dir', type=str,
                       help='MCAP 文件目录（用于批量处理）')
    parser.add_argument('--calibration', type=str, required=True,
                       help='相机标定文件路径（必需）')
    parser.add_argument('--pointcloud-output', type=str, required=True,
                       help='点云输出目录')
    parser.add_argument('--joints-output', type=str, required=True,
                       help='关节角输出目录')
    parser.add_argument('--depth-scale', type=float, default=0.001,
                       help='深度值缩放因子（RealSense为0.001），默认: 0.001')
    parser.add_argument('--depth-min', type=float, default=0.1,
                       help='最小深度阈值（米），默认: 0.1')
    parser.add_argument('--depth-max', type=float, default=5.0,
                       help='最大深度阈值（米），默认: 5.0')
    parser.add_argument('--voxel-size', type=float,
                       help='体素下采样大小（米），None 表示不下采样')
    
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
            # 单个 MCAP 文件
            converter.process_single_mcap(
                mcap_path=args.mcap,
                pointcloud_output_dir=args.pointcloud_output,
                joints_output_dir=args.joints_output,
                calibration_path=args.calibration
            )
        
        else:  # args.mcap_dir
            # 批量处理
            mcap_dir = Path(args.mcap_dir)
            mcap_files = sorted(mcap_dir.glob('*.mcap'))
            
            print(f"找到 {len(mcap_files)} 个 MCAP 文件")
            
            for mcap_file in tqdm(mcap_files, desc="处理进度"):
                try:
                    converter.process_single_mcap(
                        mcap_path=str(mcap_file),
                        pointcloud_output_dir=args.pointcloud_output,
                        joints_output_dir=args.joints_output,
                        calibration_path=args.calibration
                    )
                except Exception as e:
                    print(f"\n处理 {mcap_file} 失败: {e}")
    
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
