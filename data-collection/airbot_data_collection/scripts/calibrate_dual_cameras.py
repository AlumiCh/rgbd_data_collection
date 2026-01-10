#!/usr/bin/env python3
"""
双RealSense相机外参标定工具

功能：
1. 同时从两台D435i相机采集标定板（棋盘格）图像
2. 计算两个相机之间的相对位姿变换矩阵
3. 保存标定结果到YAML文件

使用方法：
    python calibrate_dual_cameras.py --serial1 123456789 --serial2 987654321 --board-size 9 6 --square-size 0.025

日期：2026-01-04
"""

import argparse
import yaml
import numpy as np
import cv2
import pyrealsense2 as rs
from pathlib import Path
from typing import Tuple, Optional, Dict, List
import time
from datetime import datetime


class DualCameraCalibrator:
    """双相机外参标定器"""
    
    def __init__(
        self,
        serial1: str,
        serial2: str,
        board_size: Tuple[int, int] = (9, 6),  # 棋盘格内角点数量 (宽, 高)
        square_size: float = 0.025,  # 方格尺寸（米）
        width: int = 640,
        height: int = 480,
        fps: int = 30
    ):
        """
        初始化标定器
        
        Args:
            serial1: 第一台相机序列号（作为参考相机）
            serial2: 第二台相机序列号
            board_size: 棋盘格内角点数量 (列数, 行数)
            square_size: 棋盘格每个方格的边长（米）
            width: 图像宽度
            height: 图像高度
            fps: 帧率
        """
        self.serial1 = serial1
        self.serial2 = serial2
        self.board_size = board_size
        self.square_size = square_size
        self.width = width
        self.height = height
        self.fps = fps
        
        # 存储采集的图像
        self.camera1_images = []  # RGB图像
        self.camera2_images = []
        
        # 存储检测到的角点
        self.camera1_corners = []
        self.camera2_corners = []
        
        # 存储相机内参
        self.camera1_intrinsics = None
        self.camera2_intrinsics = None
        
        # 标定结果
        self.R = None  # 旋转矩阵 (3x3)
        self.t = None  # 平移向量 (3x1)
        self.T = None  # 变换矩阵 (4x4)
        self.reprojection_error = 0.0
        
        # RealSense管道
        self.pipeline1 = None
        self.pipeline2 = None
        
        # 终止标志
        self.should_exit = False
        
        print("=" * 60)
        print("双RealSense相机外参标定工具")
        print("=" * 60)
        print(f"相机1序列号: {serial1}")
        print(f"相机2序列号: {serial2}")
        print(f"棋盘格配置: {board_size[0]}x{board_size[1]}, 方格大小={square_size*1000:.1f}mm")
        print("=" * 60)
    
    def initialize_cameras(self):
        """初始化两台RealSense相机"""
        print("\n[1/5] 正在初始化相机...")
        
        # ===== 初始化相机1 =====
        # 相机1配置
        config1 = rs.config()
        config1.enable_device(self.serial1)
        config1.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        
        self.pipeline1 = rs.pipeline()
        profile1 = self.pipeline1.start(config1)
        
        # 获取相机1内参
        color_stream1 = profile1.get_stream(rs.stream.color)
        intrinsics1 = color_stream1.as_video_stream_profile().get_intrinsics()
        self.camera1_intrinsics = {
            'fx': intrinsics1.fx,
            'fy': intrinsics1.fy,
            'cx': intrinsics1.ppx,
            'cy': intrinsics1.ppy,
            'width': intrinsics1.width,
            'height': intrinsics1.height,
            'distortion_model': intrinsics1.model.name,
            'distortion_coeffs': intrinsics1.coeffs
        }
        print(f"  ✓ 相机1 ({self.serial1}) 初始化成功")
        
        # ===== 初始化相机2 =====
        # 相机2配置
        config2 = rs.config()
        config2.enable_device(self.serial2)
        config2.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        
        self.pipeline2 = rs.pipeline()
        profile2 = self.pipeline2.start(config2)
        
        # 获取相机2内参
        color_stream2 = profile2.get_stream(rs.stream.color)
        intrinsics2 = color_stream2.as_video_stream_profile().get_intrinsics()
        self.camera2_intrinsics = {
            'fx': intrinsics2.fx,
            'fy': intrinsics2.fy,
            'cx': intrinsics2.ppx,
            'cy': intrinsics2.ppy,
            'width': intrinsics2.width,
            'height': intrinsics2.height,
            'distortion_model': intrinsics2.model.name,
            'distortion_coeffs': intrinsics2.coeffs
        }
        print(f"  ✓ 相机2 ({self.serial2}) 初始化成功")
        
        # 等待相机稳定
        print("  正在等待相机稳定...")
        for _ in range(30):
            self.pipeline1.wait_for_frames()
            self.pipeline2.wait_for_frames()
        
        print("  ✓ 相机初始化完成")
    
    def capture_images(self, min_pairs: int = 15):
        """
        交互式采集标定图像
        
        Args:
            min_pairs: 最少需要采集的图像对数量
        """
        print(f"\n[2/5] 开始采集标定图像（目标：至少{min_pairs}对）...")
        print("\n操作说明：")
        print("  - 将棋盘格放置在两个相机都能看到的位置")
        print("  - 按 SPACE 键：采集当前图像对")
        print("  - 按 Q 键：完成采集（需至少采集15对图像）")
        print("  - 按 ESC 键：退出程序")
        print("\n提示：")
        print("  1. 从不同角度、不同距离采集棋盘格图像")
        print("  2. 确保棋盘格填充画面的不同区域")
        print("  3. 采集时保持棋盘格静止")
        print("-" * 60)
        
        cv2.namedWindow("相机1 (参考)", cv2.WINDOW_NORMAL)
        cv2.namedWindow("相机2", cv2.WINDOW_NORMAL)
        
        captured_count = 0
        
        # ===== 图像采集主循环 =====
        while not self.should_exit:
            # 获取两个相机的帧
            frames1 = self.pipeline1.wait_for_frames()
            frames2 = self.pipeline2.wait_for_frames()
            
            color_frame1 = frames1.get_color_frame()
            color_frame2 = frames2.get_color_frame()
            
            if not color_frame1 or not color_frame2:
                continue
            
            # 转换为numpy数组
            image1 = np.asanyarray(color_frame1.get_data())
            image2 = np.asanyarray(color_frame2.get_data())
            
            # 检测棋盘格角点
            gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
            
            ret1, corners1 = cv2.findChessboardCorners(
                gray1, self.board_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            ret2, corners2 = cv2.findChessboardCorners(
                gray2, self.board_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            
            # 显示图像（带角点标记）
            display1 = image1.copy()
            display2 = image2.copy()
            
            if ret1:
                cv2.drawChessboardCorners(display1, self.board_size, corners1, ret1)
                cv2.putText(display1, "DETECTED", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(display1, "NOT DETECTED", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            if ret2:
                cv2.drawChessboardCorners(display2, self.board_size, corners2, ret2)
                cv2.putText(display2, "DETECTED", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(display2, "NOT DETECTED", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # 显示已采集数量
            status_text = f"Captured: {captured_count}/{min_pairs}"
            cv2.putText(display1, status_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(display2, status_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            cv2.imshow("相机1 (参考)", display1)
            cv2.imshow("相机2", display2)
            
            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' '):  # 空格键：采集
                if ret1 and ret2:
                    # 亚像素精化
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners1_refined = cv2.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), criteria)
                    corners2_refined = cv2.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), criteria)
                    
                    self.camera1_images.append(image1)
                    self.camera2_images.append(image2)
                    self.camera1_corners.append(corners1_refined)
                    self.camera2_corners.append(corners2_refined)
                    
                    captured_count += 1
                    print(f"  ✓ 已采集第 {captured_count} 对图像")
                else:
                    print("  ✗ 采集失败：至少一个相机未检测到棋盘格")
            
            elif key == ord('q') or key == ord('Q'):  # Q键：完成
                if captured_count >= min_pairs:
                    print(f"\n  ✓ 采集完成！共采集 {captured_count} 对图像")
                    break
                else:
                    print(f"  ⚠ 至少需要采集 {min_pairs} 对图像，当前仅 {captured_count} 对")
            
            elif key == 27:  # ESC键：退出
                print("\n  用户取消操作")
                self.should_exit = True
                break
        
        cv2.destroyAllWindows()
    
    def calibrate_stereo(self):
        """执行双目标定"""
        if self.should_exit:
            return False
        
        print("\n[3/5] 正在计算相机外参...")
        
        # ===== 生成世界坐标系角点 =====
        # 构建世界坐标系中的3D点
        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_size[0], 0:self.board_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        
        # 所有图像的3D点（相同）
        object_points = [objp] * len(self.camera1_corners)
        
        # 相机1内参矩阵
        K1 = np.array([
            [self.camera1_intrinsics['fx'], 0, self.camera1_intrinsics['cx']],
            [0, self.camera1_intrinsics['fy'], self.camera1_intrinsics['cy']],
            [0, 0, 1]
        ])
        D1 = np.array(self.camera1_intrinsics['distortion_coeffs'])
        
        # 相机2内参矩阵
        K2 = np.array([
            [self.camera2_intrinsics['fx'], 0, self.camera2_intrinsics['cx']],
            [0, self.camera2_intrinsics['fy'], self.camera2_intrinsics['cy']],
            [0, 0, 1]
        ])
        D2 = np.array(self.camera2_intrinsics['distortion_coeffs'])
        
        image_size = (self.width, self.height)
        
        # ===== 双目标定 =====
        # 执行双目标定
        print("  正在执行立体标定...")
        ret, K1, D1, K2, D2, R, t, E, F = cv2.stereoCalibrate(
            object_points,
            self.camera1_corners,
            self.camera2_corners,
            K1, D1, K2, D2,
            image_size,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5),
            flags=cv2.CALIB_FIX_INTRINSIC  # 固定内参，只优化外参
        )
        
        if ret:
            self.R = R  # 从相机1到相机2的旋转矩阵
            self.t = t  # 从相机1到相机2的平移向量
            
            # 构建4x4变换矩阵
            self.T = np.eye(4)
            self.T[:3, :3] = R
            self.T[:3, 3] = t.squeeze()
            self.reprojection_error = float(ret)
            
            print(f"  ✓ 标定成功！重投影误差: {ret:.4f} 像素")
            print(f"\n  旋转矩阵 R:")
            print(f"    {R[0]}")
            print(f"    {R[1]}")
            print(f"    {R[2]}")
            print(f"\n  平移向量 t (米):")
            print(f"    {t.squeeze()}")
            
            return True
        else:
            print("  ✗ 标定失败")
            return False
    
    def save_calibration(self, output_file: str = "dual_camera_calibration.yaml"):
        """
        保存标定结果到YAML文件
        
        Args:
            output_file: 输出文件路径
        """
        if self.should_exit or self.T is None:
            return
        
        print(f"\n[4/5] 正在保存标定结果...")
        
        # ===== 准备保存数据 =====
        calibration_data = {
            'calibration_date': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'board_config': {
                'size': list(self.board_size),
                'square_size_m': float(self.square_size)
            },
            'camera1': {
                'serial_number': self.serial1,
                'role': 'reference',  # 参考相机
                'intrinsics': {
                    'fx': float(self.camera1_intrinsics['fx']),
                    'fy': float(self.camera1_intrinsics['fy']),
                    'cx': float(self.camera1_intrinsics['cx']),
                    'cy': float(self.camera1_intrinsics['cy']),
                    'width': int(self.camera1_intrinsics['width']),
                    'height': int(self.camera1_intrinsics['height']),
                    'distortion_model': self.camera1_intrinsics['distortion_model'],
                    'distortion_coeffs': [float(x) for x in self.camera1_intrinsics['distortion_coeffs']]
                }
            },
            'camera2': {
                'serial_number': self.serial2,
                'role': 'secondary',
                'intrinsics': {
                    'fx': float(self.camera2_intrinsics['fx']),
                    'fy': float(self.camera2_intrinsics['fy']),
                    'cx': float(self.camera2_intrinsics['cx']),
                    'cy': float(self.camera2_intrinsics['cy']),
                    'width': int(self.camera2_intrinsics['width']),
                    'height': int(self.camera2_intrinsics['height']),
                    'distortion_model': self.camera2_intrinsics['distortion_model'],
                    'distortion_coeffs': [float(x) for x in self.camera2_intrinsics['distortion_coeffs']]
                }
            },
            'extrinsics': {
                'description': 'Transformation from camera1 to camera2 (T_2_1)',
                'rotation_matrix': self.R.tolist(),  # 3x3
                'translation_vector': self.t.squeeze().tolist(),  # 3x1 -> [x, y, z]
                'transformation_matrix': self.T.tolist(),  # 4x4
                'rotation_euler_deg': self._rotation_matrix_to_euler(self.R).tolist()  # [roll, pitch, yaw]
            },
            'statistics': {
                'num_image_pairs': len(self.camera1_images),
                'reprojection_error_px': self.reprojection_error
            }
        }
        
        # 保存到文件
        output_path = Path(output_file)
        with open(output_path, 'w', encoding='utf-8') as f:
            yaml.dump(calibration_data, f, default_flow_style=False, allow_unicode=True)
        
        print(f"  ✓ 标定结果已保存到: {output_path.absolute()}")
        print(f"\n[5/5] 标定完成！")
        print("\n使用说明：")
        print(f"  1. 标定文件: {output_file}")
        print(f"  2. 在点云融合时，使用 'transformation_matrix' 将相机2的点云变换到相机1坐标系")
        print(f"  3. 变换公式: P1 = inv(T_2_1) @ P2 （P1为相机1坐标，P2为相机2坐标）")
    
    def _rotation_matrix_to_euler(self, R: np.ndarray) -> np.ndarray:
        """
        将旋转矩阵转换为欧拉角 (roll, pitch, yaw)
        
        Args:
            R: 3x3旋转矩阵
            
        Returns:
            欧拉角 [roll, pitch, yaw] (度)
        """
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6
        
        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0
        
        return np.degrees([roll, pitch, yaw])
    
    def cleanup(self):
        """清理资源"""
        if self.pipeline1:
            self.pipeline1.stop()
        if self.pipeline2:
            self.pipeline2.stop()
        cv2.destroyAllWindows()


def main():
    # ===== 命令行参数解析 =====
    parser = argparse.ArgumentParser(
        description="双RealSense相机外参标定工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  python calibrate_dual_cameras.py --serial1 123456789 --serial2 987654321
  python calibrate_dual_cameras.py --serial1 123456789 --serial2 987654321 --board-size 9 6 --square-size 0.025
  python calibrate_dual_cameras.py --serial1 123456789 --serial2 987654321 --output my_calibration.yaml

注意事项:
  1. 准备一个标准的棋盘格标定板（黑白相间）
  2. 确保两台相机都能清晰看到棋盘格
  3. 从多个角度和距离采集图像（建议15-30对）
  4. 棋盘格应覆盖视野的不同区域
        """
    )
    
    parser.add_argument('--serial1', type=str, required=True,
                       help='第一台相机序列号（参考相机）')
    parser.add_argument('--serial2', type=str, required=True,
                       help='第二台相机序列号')
    parser.add_argument('--board-size', type=int, nargs=2, default=[9, 6],
                       metavar=('COLS', 'ROWS'),
                       help='棋盘格内角点数量 (列数 行数)，默认: 9 6')
    parser.add_argument('--square-size', type=float, default=0.025,
                       help='棋盘格方格边长（米），默认: 0.025 (25mm)')
    parser.add_argument('--min-pairs', type=int, default=15,
                       help='最少采集的图像对数量，默认: 15')
    parser.add_argument('--output', type=str, default='dual_camera_calibration.yaml',
                       help='输出文件路径，默认: dual_camera_calibration.yaml')
    parser.add_argument('--width', type=int, default=640,
                       help='图像宽度，默认: 640')
    parser.add_argument('--height', type=int, default=480,
                       help='图像高度，默认: 480')
    parser.add_argument('--fps', type=int, default=30,
                       help='帧率，默认: 30')
    
    args = parser.parse_args()
    
    # 创建标定器
    calibrator = DualCameraCalibrator(
        serial1=args.serial1,
        serial2=args.serial2,
        board_size=tuple(args.board_size),
        square_size=args.square_size,
        width=args.width,
        height=args.height,
        fps=args.fps
    )
    
    try:
        # 执行标定流程
        calibrator.initialize_cameras()
        calibrator.capture_images(min_pairs=args.min_pairs)
        
        if calibrator.calibrate_stereo():
            calibrator.save_calibration(args.output)
        else:
            print("\n标定失败，请重试")
    
    except KeyboardInterrupt:
        print("\n\n用户中断操作")
    
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        calibrator.cleanup()


if __name__ == "__main__":
    main()
