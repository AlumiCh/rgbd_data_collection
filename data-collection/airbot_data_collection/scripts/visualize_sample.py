#!/usr/bin/env python3
"""
采集数据可视化工具 - 交互式点云查看器

功能: 可视化单条采集数据
- 加载融合后的点云（PLY格式）
- 交互式3D显示
- 显示机械臂关节角数据（在终端输出）

使用方法:
    # 自动推导关节角文件
    python visualize_sample.py --pointcloud pointclouds/demo_0_fused.ply
    
    # 指定自定义关节角文件
    python visualize_sample.py \\
        --pointcloud pointclouds/demo_0_fused.ply \\
        --joints joints/demo_0.json

日期: 2026-01-08
"""

import argparse
import json
from pathlib import Path
from typing import Optional, Dict

import numpy as np
import open3d as o3d
from mcap.reader import make_reader
import matplotlib.pyplot as plt
import cv2

# 导入FlatBuffer schemas用于解码
try:
    from foxglove_schemas_flatbuffer.RawImage import RawImage
    from foxglove_schemas_flatbuffer.CompressedImage import CompressedImage
except ImportError:
    print("警告: 未安装foxglove_schemas_flatbuffer，图像解码可能失败")
    RawImage = None
    CompressedImage = None

try:
    from turbojpeg import TurboJPEG
    jpeg_decoder = TurboJPEG()
except ImportError:
    jpeg_decoder = None


class SampleVisualizer:
    """采集数据可视化器"""
    
    def __init__(self, pointcloud_path: str, joints_path: Optional[str] = None, 
                 mcap_path: Optional[str] = None):
        """
        初始化可视化器
        
        Args:
            pointcloud_path: 点云PLY文件路径
            joints_path: 关节角数据JSON文件路径（可选，会自动推导）
            mcap_path: MCAP文件路径（可选，用于提取RGB图像）
        """
        self.pointcloud_path = Path(pointcloud_path)
        self.mcap_path = Path(mcap_path) if mcap_path else None
        
        # 自动推导关节角文件路径
        if joints_path is None:
            # 从点云路径推导：pointclouds/sample_0_fused.ply → joints/sample_0.json
            joints_dir = self.pointcloud_path.parent.parent / "joints"
            stem = self.pointcloud_path.stem.replace("_fused", "")
            joints_path = joints_dir / f"{stem}.json"
        
        self.joints_path = Path(joints_path)
        
        # 验证点云文件存在
        if not self.pointcloud_path.exists():
            raise FileNotFoundError(f"点云文件不存在: {self.pointcloud_path}")
        
        # 验证MCAP文件存在（如果提供）
        if self.mcap_path and not self.mcap_path.exists():
            raise FileNotFoundError(f"MCAP文件不存在: {self.mcap_path}")
        
        self.pointcloud = None
        self.joint_angles = None
        self.rgb_images: Dict[str, np.ndarray] = {}
        
        print(f"[初始化] 点云文件: {self.pointcloud_path}")
        print(f"[初始化] 关节角文件: {self.joints_path}")
        if self.mcap_path:
            print(f"[初始化] MCAP文件: {self.mcap_path}")
    
    def extract_rgb_from_mcap(self):
        """
        ===== 从MCAP提取RGB图像 (2026-01-09) =====
        
        从MCAP文件中提取两个相机的RGB图像
        """
        if not self.mcap_path:
            return
        
        print(f"\n[RGB提取] 从MCAP提取RGB图像...")
        
        with open(self.mcap_path, 'rb') as f:
            reader = make_reader(f)
            
            for schema, channel, message in reader.iter_messages():
                topic = channel.topic
                schema_name = schema.name if schema else "unknown"
                
                # 寻找color/image相关主题
                if '/color/' in topic and 'image' in topic:
                    camera_name = topic.split('/')[0]
                    
                    try:
                        rgb_img = self._decode_rgb_image(message, schema_name)
                        
                        if rgb_img is not None:
                            self.rgb_images[camera_name] = rgb_img
                    
                    except Exception as e:
                        print(f"无法提取 {camera_name} 的 RGB: {e}")
        
        if self.rgb_images:
            print(f"提取了 {len(self.rgb_images)} 个相机的 RGB 图像")
            for cam, img in self.rgb_images.items():
                print(f"    - {cam}: {img.shape}")
        else:
            print(f"未找到 RGB 图像")
    
    def _decode_rgb_image(self, message, schema_name: str) -> Optional[np.ndarray]:
        """解码 RGB 图像消息"""
        try:
            if schema_name == "foxglove.RawImage" and RawImage:
                raw_img = RawImage.GetRootAs(message.data, 0)
                width = raw_img.Width()
                height = raw_img.Height()
                encoding = raw_img.Encoding()
                if encoding:
                    encoding = encoding.decode('utf-8')
                
                data_bytes = raw_img.DataAsNumpy().tobytes()
                
                if '8UC3' in encoding or 'rgb8' in encoding:
                    img = np.frombuffer(data_bytes, dtype=np.uint8).reshape(height, width, 3)
                    return img
                elif 'bgr8' in encoding:
                    img = np.frombuffer(data_bytes, dtype=np.uint8).reshape(height, width, 3)
                    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
            elif schema_name == "foxglove.CompressedImage" and CompressedImage:
                compressed_img = CompressedImage.GetRootAs(message.data, 0)
                format_str = compressed_img.Format()
                if format_str:
                    format_str = format_str.decode('utf-8')
                
                data_bytes = compressed_img.DataAsNumpy().tobytes()
                
                if format_str in ['jpeg', 'jpg']:
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
        
        except Exception as e:
            print(f"解码 RGB 失败: {e}")
        
        return None
        
    
    def load_data(self):
        """加载所有数据"""
        print("\n[数据加载]")
        
        # 加载点云
        print(f"  加载点云...")
        self.pointcloud = o3d.io.read_point_cloud(str(self.pointcloud_path))
        print(f"  ✓ 点云加载完成: {len(self.pointcloud.points)} 个点")
        
        # 加载关节角
        print(f"  加载关节角...")
        if self.joints_path.exists():
            try:
                with open(self.joints_path, 'r') as f:
                    data = json.load(f)
                    self.joint_angles = np.array(data, dtype=np.float32)
                    print(f"  ✓ 关节角加载完成: {len(self.joint_angles)} 维")
            except Exception as e:
                print(f"  ⚠ 关节角加载失败: {e}")
        else:
            print(f"  ⚠ 关节角文件不存在: {self.joints_path}")
        
        # 从MCAP提取RGB图像（如果提供）
        if self.mcap_path:
            self.extract_rgb_from_mcap()
    
    def visualize(self):
        """交互式可视化"""
        self.load_data()
        
        # 显示RGB图像窗口（如果有）
        if self.rgb_images:
            print("\n[显示] 显示RGB图像窗口...")
            self.show_rgb_images()
        
        print("\n[可视化] 启动交互式3D点云查看器...")
        
        # 创建Open3D可视化窗口
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="点云可视化", width=1400, height=900)
        
        # 添加坐标系
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.3, origin=[0, 0, 0])
        vis.add_geometry(mesh_frame)
        
        # 添加点云
        vis.add_geometry(self.pointcloud)
        
        # 配置渲染选项
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.2, 0.2, 0.2])
        opt.point_size = 3.0
        
        # 显示关节角信息
        print("\n" + "="*60)
        print("机械臂关节角:")
        print("="*60)
        
        if self.joint_angles is not None:
            joint_names = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Gripper']
            for i, angle in enumerate(self.joint_angles):
                joint_name = joint_names[i] if i < len(joint_names) else f'Joint{i}'
                
                if i == len(self.joint_angles) - 1:  # 夹爪
                    print(f"  {joint_name:12s}: {angle:10.4f} m")
                else:
                    print(f"  {joint_name:12s}: {angle:10.4f} rad ({np.degrees(angle):7.2f}°)")
        else:
            print("  [未加载关节角数据]")
        
        print("="*60)
        print("\n交互式操作:")
        print("  - 左键拖动: 旋转视图")
        print("  - 右键拖动: 平移视图")
        print("  - 滚轮: 缩放视图")
        print("  - 按 'Q' 关闭窗口")
        print("="*60 + "\n")
        
        # 显示窗口
        vis.run()
        vis.destroy_window()
    
    def show_rgb_images(self):
        """
        ===== 显示 RGB 图像 (2026-01-09) =====
        
        创建matplotlib窗口显示两个相机的RGB图像
        """
        try:
            camera_names = sorted(self.rgb_images.keys())
            
            if not camera_names:
                return
            
            # 创建图表
            fig, axes = plt.subplots(1, len(camera_names), figsize=(14, 6))
            
            # 如果只有一个相机，axes不是数组
            if len(camera_names) == 1:
                axes = [axes]
            
            # 显示每个相机的RGB图像
            for idx, camera_name in enumerate(camera_names):
                rgb_img = self.rgb_images[camera_name]
                
                if isinstance(rgb_img, np.ndarray) and len(rgb_img.shape) == 3:
                    axes[idx].imshow(rgb_img)
                    axes[idx].set_title(f'RGB - {camera_name}', fontsize=12, fontweight='bold')
                else:
                    axes[idx].text(0.5, 0.5, f'图像格式错误\nshape: {rgb_img.shape if hasattr(rgb_img, "shape") else "unknown"}', 
                                 ha='center', va='center',
                                 transform=axes[idx].transAxes)
                    axes[idx].set_title(f'{camera_name} - 错误', fontsize=12)
                
                axes[idx].axis('off')
            
            plt.tight_layout()
            plt.show(block=False)
        
        except Exception as e:
            print(f"显示 RGB 图像失败: {e}")
            import traceback
            traceback.print_exc()


def main():
    parser = argparse.ArgumentParser(
        description="采集数据可视化工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 基础使用（仅显示点云和关节角）
  python visualize_sample.py --pointcloud pointclouds/demo_0_fused.ply
  
  # 显示点云、关节角和RGB图像
  python visualize_sample.py \\
      --pointcloud pointclouds/demo_0_fused.ply \\
      --mcap data/demo_0.mcap
  
  # 指定自定义关节角文件
  python visualize_sample.py \\
      --pointcloud pointclouds/demo_0_fused.ply \\
      --joints custom_joints/demo_0.json \\
      --mcap data/demo_0.mcap
        """
    )
    
    parser.add_argument('--pointcloud', type=str, required=True,
                       help='点云PLY文件路径')
    parser.add_argument('--joints', type=str, default=None,
                       help='关节角JSON文件路径（可选，默认自动推导）')
    parser.add_argument('--mcap', type=str, default=None,
                       help='MCAP文件路径（可选，用于提取RGB图像显示）')
    
    args = parser.parse_args()
    
    try:
        visualizer = SampleVisualizer(
            pointcloud_path=args.pointcloud,
            joints_path=args.joints,
            mcap_path=args.mcap
        )
        visualizer.visualize()
    
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
