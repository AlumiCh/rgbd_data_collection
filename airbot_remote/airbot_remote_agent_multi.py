import time
import json
import numpy as np
import requests
from collections import deque
from PIL import Image
import io
from dataclasses import dataclass
from jsonargparse import ArgumentParser
import sys
import select
import cv2
import os
import threading
from termcolor import cprint
from flask import Flask, request, jsonify

from airbot_robot import AIRBOTPlay
from airbot_py.arm import RobotMode

# 用于控制终端的ANSI转义码
CURSOR_UP_ONE = "\x1b[1A"
ERASE_LINE = "\x1b[2K\r"
HIDE_CURSOR = "\x1b[?25l"
SHOW_CURSOR = "\x1b[?25h"


@dataclass
class RemoteConfig:
    """远程控制配置参数"""

    target_ip: str = "192.168.3.101"
    task_ports_json: str = (
        '{"pick_bowls": "6160", "put_bowls": "6161","pick_mouthwash_table": "6162","pick_mouthwash_cabinet": "6163", "pick_cup": "6164","put_cup": "6165"}'
    )

    # --- MODIFIED: 阈值配置现在也包含 max_steps ---
    task_thresholds_json: str = (
        '{"pick_bowls": {"thresh": 0.001, "frames": 100, "max_steps": 1000}, '
        '"put_bowls": {"thresh": 0.005, "frames": 50, "max_steps": 600}, '
        '"pick_mouthwash_table": {"thresh": 0.002, "frames": 50, "max_steps": 500}, '
        '"pick_mouthwash_cabinet": {"thresh": 0.002, "frames": 50, "max_steps": 500}, '
        '"pick_cup": {"thresh": 0.002, "frames": 50, "max_steps": 51000}, '
        '"put_cup": {"thresh": 0.005, "frames": 500, "max_steps": 500}}'
    )
    default_thresh: float = 0.001  # 默认阈值
    default_stable_frames: int = 100  # 默认稳定帧数
    default_max_steps: int = 800  # --- MODIFIED: 重命名 (原 max_steps) ---

    default_task: str = "pick_bowls"
    listener_port: int = 10002  # 本地监听端口，用于接收任务切换命令

    # max_steps: int = 800 # --- REMOVED: 已被 default_max_steps 替代 ---
    control_freq: int = 20
    jpg_quality: int = 80

    camera_names: str = "cam_head cam_left cam_right"
    display_order: str = "cam_left cam_head cam_right"

    def __post_init__(self):
        # 自动转换字符串为列表
        if isinstance(self.camera_names, str):
            self.camera_names = self.camera_names.split()
        if isinstance(self.display_order, str):
            self.display_order = self.display_order.split()

        if isinstance(self.task_ports_json, str):
            self.task_ports = json.loads(self.task_ports_json)
        else:
            self.task_ports = self.task_ports_json

        # --- NEW: 解析阈值 JSON ---
        if isinstance(self.task_thresholds_json, str):
            self.task_thresholds = json.loads(self.task_thresholds_json)
        else:
            self.task_thresholds = self.task_thresholds_json
        # --- END NEW ---


class AIRBOT_Controller:
    """
    AIRBOT 机器人远程控制器
    通过后台线程实时采集和显示图像，主线程负责与服务器通信和执行控制逻辑。
    """

    WINDOW_NAME = "AIRBOT Real-time Feeds"

    def __init__(
        self,
        target_ip: str,
        task_ports: dict,
        task_thresholds: dict,
        default_thresh: float,
        default_stable_frames: int,
        default_max_steps: int,
        default_task: str,
        control_freq,
        camera_names,
        display_order,
        jpg_quality,
    ):
        self.target_ip = target_ip
        self.task_ports = task_ports

        # --- 存储阈值/步数配置 ---
        self.task_thresholds = task_thresholds
        self.default_thresh = default_thresh
        self.default_stable_frames = default_stable_frames
        self.default_max_steps = default_max_steps

        # 当前任务的动态配置
        self._current_thresh = self.default_thresh
        self._current_stable_frames = self.default_stable_frames
        self._current_max_steps = self.default_max_steps

        self._current_task = default_task
        self._current_server_url = self._build_server_url(default_task)
        if not self._current_server_url:
            raise ValueError(
                f"默认任务 '{default_task}' 不在任务端口列表 {task_ports} 中。"
            )

        # --- 立即为默认任务设置正确的初始配置 ---
        default_task_settings = self.task_thresholds.get(default_task, {})
        self._current_thresh = default_task_settings.get("thresh", self.default_thresh)
        self._current_stable_frames = default_task_settings.get(
            "frames", self.default_stable_frames
        )
        self._current_max_steps = default_task_settings.get(
            "max_steps", self.default_max_steps
        )

        self.control_freq = control_freq
        self.camera_names = camera_names
        self.display_order = display_order
        self.jpg_quality = jpg_quality

        self.session = requests.Session()

        cprint("[*] 初始化 AIRBOT 机器人...", "cyan")
        self.robot = AIRBOTPlay(web_camera=True)

        for k in [
            "HTTP_PROXY",
            "http_proxy",
            "HTTPS_PROXY",
            "https_proxy",
            "ALL_PROXY",
            "all_proxy",
        ]:
            os.environ.pop(k, None)

        # --- 线程与缓存初始化 ---
        self.latest_observation_cache = None
        self.cache_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.capture_thread = None
        self.print_lock = threading.Lock()
        self.action_buffer = deque()  # 动作缓存区

        # ---  状态管理 ---
        self.is_running = False  # 标志是否在运行控制循环
        self.state_lock = threading.Lock()  # 保护 is_running 标志
        self.control_loop_stop_event = threading.Event()  # 用于从外部停止控制循环
        self.control_loop_thread = None  # 指向控制循环的线程
        self.url_lock = threading.Lock()  # 保护 _current_server_url 和 _current_task
        self.buffer_lock = threading.Lock()  # 保护 action_buffer

        cprint(f"[*] AIRBOT PLAY 控制器初始化完成, 默认任务: '{default_task}'", "green")
        cprint("[*] 状态: IDLE (空闲). 等待 /start_task 命令...", "yellow")
        self.start_capture_thread()

    def safe_cprint(self, *args, **kwargs):
        with self.print_lock:
            # 强制 cprint 打印到 sys.stderr，以绕过 Flask 的 stdout 重定向
            cprint(*args, **kwargs, file=sys.stderr)

    # ----------------------------------------------------------------
    # --- 0. 任务端口管理 ---
    # ----------------------------------------------------------------
    def _build_server_url(self, task_name: str) -> str | None:
        """根据任务名构建完整的服务器URL"""
        port = self.task_ports.get(task_name)
        if port is None:
            self.safe_cprint(f"[!] 警告: 任务 '{task_name}' 没有找到对应的端口!", "red")
            return None
        return f"http://{self.target_ip}:{port}"

    def _prepare_task(self, task_name: str):
        """线程安全地准备任务（设置URL、动态配置并清空缓存）"""
        if task_name not in self.task_ports:
            self.safe_cprint(f"[!] 任务准备失败: 未知的任务 '{task_name}'", "red")
            return False, f"Unknown task: {task_name}"

        new_url = self._build_server_url(task_name)
        if not new_url:
            return False, f"Failed to build URL for task {task_name}"

        with self.url_lock:
            self.safe_cprint(f"\n[*] 任务准备: '{task_name}'", "cyan")
            self.safe_cprint(f"[*] 目标 URL 设置为: {new_url}", "cyan")
            self._current_task = task_name
            self._current_server_url = new_url

            # --- 动态设置当前任务的配置 (阈值 + 最大步数) ---
            # 1. 从配置中获取该任务的特定设置，如果不存在则返回空字典
            task_settings = self.task_thresholds.get(task_name, {})

            # 2. 获取特定阈值，如果未定义，则使用默认值
            self._current_thresh = task_settings.get("thresh", self.default_thresh)
            self._current_stable_frames = task_settings.get(
                "frames", self.default_stable_frames
            )
            # 3. 获取特定最大步数，如果未定义，则使用默认值
            self._current_max_steps = task_settings.get(
                "max_steps", self.default_max_steps
            )  # --- NEW ---

            self.safe_cprint(
                f"[*] 任务阈值设置为: THRESH={self._current_thresh}, STABLE_FRAMES={self._current_stable_frames}",
                "cyan",
            )
            self.safe_cprint(
                f"[*] 任务最大步数设置为: MAX_STEPS={self._current_max_steps}",
                "cyan",
            )
            # --- END ---

        with self.buffer_lock:
            self.action_buffer.clear()
        self.safe_cprint(f"[*] 动作缓存已清空。", "cyan")
        return True, f"Task {task_name} prepared"

    def get_current_server_url(self):
        with self.url_lock:
            return self._current_server_url

    def get_current_task(self):
        with self.url_lock:
            return self._current_task

    # ----------------------------------------------------------------
    # --- 1. 后台采集线程管理 (生产者) ---
    # ----------------------------------------------------------------
    def start_capture_thread(self):
        if self.capture_thread is not None and self.capture_thread.is_alive():
            self.safe_cprint("[!] 采集线程已在运行", "yellow")
            return

        self.safe_cprint("[*] 启动后台摄像头采集和显示线程...", "cyan")
        self.stop_event.clear()
        self.capture_thread = threading.Thread(target=self._capture_and_display_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()

    def stop_capture_thread(self):
        if self.capture_thread is None or not self.capture_thread.is_alive():
            return

        self.safe_cprint("[*] 正在停止后台采集线程...", "yellow")
        self.stop_event.set()
        self.capture_thread.join(timeout=2)
        if self.capture_thread.is_alive():
            self.safe_cprint("[!] 警告: 采集线程未能正常停止", "red")
        self.capture_thread = None

    def _capture_and_display_loop(self):
        """后台线程的主循环函数，持续采集、处理、缓存和显示数据。"""
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        while not self.stop_event.is_set():
            try:
                obs = self.robot.capture_observation()
                if not obs:
                    time.sleep(0.1)
                    continue

                files_to_send, data_to_send, raw_images = self._process_observation(obs)

                with self.cache_lock:
                    self.latest_observation_cache = {
                        "files": files_to_send,
                        "data": data_to_send,
                        "raw_images": raw_images,
                    }

                if raw_images:
                    self._display_images(raw_images)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self.safe_cprint(
                        "\n[*] 在预览窗口中按下了'q'，将停止预览线程...", "yellow"
                    )
                    self.safe_cprint(
                        "[*] 注意：主程序仍在运行，请在终端菜单中选择 '0' 退出。",
                        "yellow",
                    )
                    break

            except Exception as e:
                self.safe_cprint(f"\n[!] 后台采集线程发生错误: {e}", "red")
                time.sleep(1)

        self.safe_cprint("[*] 后台采集线程已停止。", "green")
        cv2.destroyAllWindows()
        for _ in range(5):
            cv2.waitKey(1)

    def _display_images(self, raw_images):
        """根据配置的顺序拼接并显示图像"""
        bgr_images = []
        # 使用一个有效的图像作为占位符的模板
        placeholder_template = next(
            (img for img in raw_images.values() if img is not None), None
        )

        for name in self.display_order:
            img = raw_images.get(name)
            if img is not None:
                # 确保 img 是 numpy array
                if isinstance(img, Image.Image):
                    img = np.array(img)
                # 确保是 BGR
                if img.ndim == 3 and img.shape[2] == 3:
                    bgr_images.append(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                elif img.ndim == 2:  # 灰度图
                    bgr_images.append(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
                else:  # 假设是已有BGR
                    bgr_images.append(img)
            else:
                # 如果某个摄像头图像不存在，创建一个黑色占位符
                if placeholder_template is not None:
                    # 确保模板是 numpy array
                    if isinstance(placeholder_template, Image.Image):
                        template_shape = np.array(placeholder_template).shape
                    else:
                        template_shape = placeholder_template.shape
                    placeholder = np.zeros(template_shape, dtype=np.uint8)
                else:  # 极端情况，所有摄像头都失效
                    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                bgr_images.append(placeholder)

        if bgr_images:
            try:
                # 过滤掉 None 或者 shape 不正确的
                valid_images = []
                ref_shape = None
                for img in bgr_images:
                    if (
                        img is not None
                        and isinstance(img, np.ndarray)
                        and img.ndim == 3
                    ):
                        if ref_shape is None:
                            ref_shape = img.shape[:2]  #
                        if img.shape[:2] == ref_shape:
                            valid_images.append(img)
                        else:  # 形状不匹配, 创建一个匹配的占位符
                            self.safe_cprint(
                                f"[!] 警告: 图像形状不匹配 {img.shape} vs {ref_shape}",
                                "yellow",
                            )
                            valid_images.append(
                                np.zeros(
                                    (ref_shape[0], ref_shape[1], 3), dtype=np.uint8
                                )
                            )

                if valid_images:
                    combined_image = np.hstack(valid_images)
                    cv2.imshow(self.WINDOW_NAME, combined_image)
            except Exception as e:
                self.safe_cprint(f"[!] 图像显示错误: {e}", "red")
                # self.safe_cprint(f"[Debug] 图像列表: {[img.shape for img in bgr_images]}", "grey")

    # ----------------------------------------------------------------
    # --- 2. 数据处理与获取 (生产者-消费者接口) ---
    # ----------------------------------------------------------------
    def get_observation_from_cache(self):
        """从缓存中线程安全地获取最新的一帧观测数据"""
        with self.cache_lock:
            if self.latest_observation_cache is None:
                return None, None, None
            c = self.latest_observation_cache.copy()
            return c.get("files"), c.get("data"), c.get("raw_images")

    def _process_observation(self, obs: dict):
        """将原始obs字典处理成发送和显示所需的数据格式"""
        files_to_send, raw_images = [], {}
        camera_mapping = {
            "cam_head": "cam_high",
            "cam_right": "cam_right_wrist",
            "cam_left": "cam_left_wrist",
        }

        for cam_name in self.camera_names:
            logical_name = camera_mapping.get(cam_name)
            img = (
                obs.get(f"observation.images.{logical_name}") if logical_name else None
            )
            if img is not None:
                raw_images[cam_name] = img  # 存储原始图像 (np.array or PIL.Image)
                img_bytes = self._image_to_jpeg_bytes(img)
                files_to_send.append(
                    ("images", (f"{cam_name}.jpg", img_bytes, "image/jpeg"))
                )

        qpos = obs.get("observation.state", [])
        data_to_send = {"qpos": json.dumps(qpos)}
        return files_to_send, data_to_send, raw_images

    def _image_to_jpeg_bytes(self, image):
        if isinstance(image, (bytes, bytearray)):
            return bytes(image)
        if isinstance(image, np.ndarray):
            if image.dtype != np.uint8:
                image = (image * 255).astype(np.uint8)
            # 确保是RGB格式给PIL
            if image.ndim == 3 and image.shape[2] == 3:
                image = Image.fromarray(image, "RGB")
            elif image.ndim == 2:  # 灰度图
                image = Image.fromarray(image, "L")
            else:
                self.safe_cprint(
                    f"[!] 警告: 无法处理的Numpy图像形状: {image.shape}", "red"
                )
                image = Image.new("RGB", (640, 480), color="red")

        buffer = io.BytesIO()
        # 确保 image 是 PIL.Image
        if not isinstance(image, Image.Image):
            self.safe_cprint(
                f"[!] 警告: 传入 _image_to_jpeg_bytes 的不是有效图像类型: {type(image)}",
                "red",
            )
            # 创建一个占位符
            image = Image.new("RGB", (640, 480), color="red")

        image.save(buffer, format="JPEG", quality=self.jpg_quality)
        return buffer.getvalue()

    # --- 3. 机器人控制与通信 (消费者) ---
    def start_task(self, task_name: str):
        """由Flask线程调用，用于启动一个新任务"""
        current_url = ""  # 在锁外部存储URL

        with self.state_lock:
            if self.is_running:
                self.safe_cprint(
                    f"[!] 拒绝启动: 任务 '{self.get_current_task()}' 已在运行。", "red"
                )
                return False, f"A task ({self.get_current_task()}) is already running"

            # 准备任务（设置URL, 清空缓存, 设置动态配置）
            success, message = self._prepare_task(task_name)
            if not success:
                return False, message
            current_url = self.get_current_server_url()

        self.safe_cprint(f"[*] 正在测试连接到: {current_url}", "cyan")
        if not self.test_connection(current_url):
            self.safe_cprint("[!] 服务器连接失败，退出控制循环。", "red")
            return False, "Server connection test failed"

        with self.state_lock:
            # 标记为运行中，并清除停止标志
            self.control_loop_stop_event.clear()
            self.is_running = True

            # 在新线程中启动控制循环
            self.control_loop_thread = threading.Thread(
                target=self.run_control_loop, daemon=True
            )
            self.control_loop_thread.start()

        self.safe_cprint(f"[✔] 任务 '{task_name}' 已启动!", "green")
        return True, f"Task {task_name} started"

    def stop_task(self):
        """
        (修改版) 停止当前任务，并等待控制线程真正退出后再返回。
        """
        thread_to_wait = None  # 用于存储我们要等待的线程

        with self.state_lock:
            if not self.is_running:
                self.safe_cprint("[!] 拒绝停止: 没有任务在运行。", "red")
                return False, "No task is currently running"

            self.safe_cprint("\n[!] 收到 /stop_task 命令, 正在发送停止信号...", "cyan")
            self.control_loop_stop_event.set()  # 1. 发送停止信号
            thread_to_wait = self.control_loop_thread  # 2. 获取对该线程的引用

            # 注意：我们在这里 *不* 改变 is_running 的状态
            # 我们让 _cleanup_control_loop 函数去改变它

        # 3. (关键) 在锁之外等待线程结束
        if thread_to_wait:
            self.safe_cprint(
                "[*] 正在等待控制循环彻底停止 (执行 back_home...)...", "cyan"
            )
            thread_to_wait.join(timeout=10)  # 等待最多10秒
            if thread_to_wait.is_alive():
                self.safe_cprint("[!] 警告: 控制线程在10秒内未能停止。", "red")
                return False, "Task thread timed out."

        self.safe_cprint("[✔] 控制循环已确认停止。", "cyan")
        return True, "Task successfully stopped and cleaned up."

    def request_inference(self, files_to_send: list, data_to_send: dict):
        """向推理服务器请求动作预测"""
        try:
            current_url = self.get_current_server_url()
            resp = self.session.post(
                current_url + "/predict",  # 使用动态更新的URL
                files=files_to_send,
                data=data_to_send,
                proxies={"http": None, "https": None},
                timeout=5.0,  # 设置5秒超时
            )
            if resp.status_code == 200:
                return resp.json(), None
            return None, f"请求失败: {resp.status_code}, {resp.text}"
        except requests.exceptions.Timeout:
            # 超时是一个“可接受”的错误，循环会继续
            return None, "网络请求超时 (timeout)"
        except requests.exceptions.ConnectionError as e:
            return None, f"网络连接异常: {e}"
        except requests.RequestException as e:
            return None, f"网络请求异常: {e}"

    def get_action(self):
        """从缓存获取数据并发起推理请求"""
        files_to_send, data_to_send, raw_images = self.get_observation_from_cache()
        if not files_to_send or not data_to_send:
            return 0, [], None, None, "缓存为空或数据不完整"

        start_time = time.perf_counter()
        result, error = self.request_inference(files_to_send, data_to_send)
        cost_time = time.perf_counter() - start_time

        qpos_str = data_to_send.get("qpos")
        qpos = json.loads(qpos_str) if qpos_str else []

        if result and "actions" in result:
            return cost_time, qpos, result["actions"], raw_images, None
        return cost_time, qpos, None, raw_images, error or "推理服务器未返回有效动作"

    def _check_task_complete(self, qpos):
        """
        检测任务是否完成（仅基于关节姿态）
        当连续若干帧的平均关节角变化幅度低于阈值时认为任务完成
        """
        # 提取关节角度
        if qpos is None or len(qpos) == 0:
            return False, 0.0
        q = np.array(qpos, dtype=np.float32)

        # 初始化历史记录
        if not hasattr(self, "_joint_history"):
            self._joint_history = deque(maxlen=20)  # 20帧大约1~2秒
            self._stable_counter = 0

        self._joint_history.append(q)

        # 如果帧数太少，先不判断
        if len(self._joint_history) < 5:
            return False, 0.0

        # 计算每帧的均方变化（防止个别关节干扰）
        try:
            diffs = np.linalg.norm(
                np.diff(np.stack(self._joint_history), axis=0), axis=1
            )
            avg_speed = np.mean(diffs)
        except Exception as e:
            # 在历史记录被并发修改时可能出错（虽然不太可能）
            self.safe_cprint(f"[!] _check_task_complete 计算错误: {e}", "red")
            return False, 0.0

        # 阈值设置（单位：弧度/帧）
        # 从 self 中动态读取当前任务的阈值
        THRESH = self._current_thresh

        # 连续稳定帧的数量要求
        STABLE_FRAMES = self._current_stable_frames

        # 如果当前平均关节速度很小，则计数+1，否则清零
        if avg_speed < THRESH:
            self._stable_counter += 1
        else:
            self._stable_counter = 0

        # 如果连续多帧都很稳定，认为任务完成
        if self._stable_counter >= STABLE_FRAMES:
            # 重置计数器，防止立即再次触发
            self._stable_counter = 0
            self._joint_history.clear()
            return True, avg_speed

        return False, avg_speed

    def run_control_loop(self):
        """
        !! 此方法现在由 start_task 在一个单独的线程中启动 !!
        """
        self.safe_cprint("\n[*] 进入远程推理控制...", "magenta")

        self.safe_cprint("[*] 切换到伺服模式...", "cyan")
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        self.go_home()

        # --- 重置任务完成检测器 ---
        # 确保每次开始新任务时，历史记录和计数器都被清空
        if hasattr(self, "_joint_history"):
            self._joint_history.clear()
        self._stable_counter = 0
        # ---

        with self.buffer_lock:
            self.action_buffer.clear()
        self.safe_cprint("[*] 动作缓存已清空。", "cyan")

        self.safe_cprint("提示：在控制过程中按 q + 回车 可随时退出", "yellow")
        self.safe_cprint("提示：也可通过 /stop_task API 停止任务", "yellow")
        print("\n" * 9)  # --- MODIFIED: 仪表盘增加到 9 行 ---

        sys.stdout.write(HIDE_CURSOR)
        sys.stdout.flush()

        cost_time_window = deque(maxlen=100)
        step = 0
        last_request_cost_time = 0.0

        # --- NEW: 用于显示原始 avg_speed 的持久化变量 ---
        last_avg_speed = 0.0
        # --- END NEW ---

        try:
            while (
                step < self._current_max_steps
                and not self.control_loop_stop_event.is_set()
            ):
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    if sys.stdin.readline().strip().lower() == "q":
                        self.safe_cprint(
                            "\n[*] 'q'被按下, 正在停止... (将由API处理为停止)"
                        )
                        self.control_loop_stop_event.set()  # 触发停止
                        break

                start_loop_t = time.perf_counter()
                action, error, cost_time, qpos = None, None, 0, []

                # --- 动作获取逻辑 (带锁) ---
                action_from_buffer = None
                with self.buffer_lock:
                    if self.action_buffer:
                        action_from_buffer = self.action_buffer.popleft()

                if action_from_buffer:
                    action = action_from_buffer
                    _, data_to_send, _ = self.get_observation_from_cache()
                    qpos_str = data_to_send.get("qpos") if data_to_send else None
                    qpos = json.loads(qpos_str) if qpos_str else []
                else:
                    cost_time, qpos, actions, _, req_error = self.get_action()
                    if req_error:
                        error = req_error
                    elif actions is not None:
                        is_2d = (
                            isinstance(actions, list)
                            and len(actions) > 0
                            and isinstance(actions[0], list)
                        )
                        if is_2d:
                            with self.buffer_lock:
                                self.action_buffer.extend(actions)
                                # 确保 buffer 不为空
                                if self.action_buffer:
                                    action = self.action_buffer.popleft()
                                else:
                                    error = "推理返回空的多维动作列表"
                        else:
                            action = actions
                    else:
                        error = "推理服务器未返回有效动作 (actions: None)"

                with self.url_lock:
                    current_task_str = f"   - 当前任务: {self._current_task}"

                if error or action is None:
                    final_error = error or "未能获取有效动作"
                    status_msg = (
                        f"🔴 控制状态: 错误! {str(final_error).replace(chr(10), ' ')}"
                    )
                    action_str = "   - 动作(action): [无]"
                    fps_str = ""
                    # 如果出错，稍微等待一下，避免刷屏
                    time.sleep(0.1)
                else:
                    self.robot.send_action(action)
                    step += 1
                    if cost_time > 0:  # 仅在实际网络请求时才记录时间
                        cost_time_window.append(cost_time)
                        last_request_cost_time = cost_time

                    net_requests = [c for c in cost_time_window if c > 0]
                    avg_cost_time = (
                        sum(net_requests) / len(net_requests) if net_requests else 0
                    )

                    # --- MODIFIED: 使用动态最大步数 ---
                    status_msg = f"🟢 控制状态: 运行中... (第 {step} / {self._current_max_steps} 步)"
                    # --- END MODIFIED ---

                    action_str = f"   - 动作(action): {[f'{x:.2f}' for x in action]}"
                    fps_str = (
                        f" .  - 平均请求帧数: {1/avg_cost_time:.2f} hz"
                        if avg_cost_time > 0
                        else ""
                    )

                    # --- MODIFIED: 获取原始 avg_speed 并更新 ---
                    if_finished, avg_speed = self._check_task_complete(qpos)
                    last_avg_speed = avg_speed
                    # --- END MODIFIED ---

                    # 检查是否任务完成
                    if step > 100 and if_finished:  # 至少执行100步后再检查
                        self.safe_cprint(
                            "\n[✔] 检测到任务完成 (关节稳定)，正在停止...", "green"
                        )
                        self.control_loop_stop_event.set()  # 触发停止
                        break

                # --- 统一的状态信息准备 ---
                qpos_str = f"   - 观测(qpos): {[f'{x:.2f}' for x in qpos]}"
                buffer_str = f"   - 动作缓存: {len(self.action_buffer)} 个"
                inference_time_str = (
                    f"   - 本次请求耗时: {last_request_cost_time*1000:.2f} ms"
                )
                config_str = f"   - 任务配置: Thresh={self._current_thresh} | Frames={self._current_stable_frames}"
                avg_speed_str = f"   - 关节均速(avg_speed): {last_avg_speed:.7f}"

                lines = [
                    status_msg,
                    current_task_str,
                    config_str,
                    qpos_str,
                    action_str,
                    buffer_str,
                    inference_time_str,
                    avg_speed_str,
                    fps_str,
                ]
                sys.stdout.write(CURSOR_UP_ONE * 9)
                for line in lines:
                    sys.stdout.write(f"\r{ERASE_LINE}{line}\n")
                sys.stdout.flush()

                dt_s = time.perf_counter() - start_loop_t
                time.sleep(max(0, 1 / self.control_freq - dt_s))

            # 循环结束后的检查
            if step >= self._current_max_steps:
                self.safe_cprint(
                    f"\n[!] 达到最大步数 {self._current_max_steps}，任务停止。",
                    "yellow",
                )

        except KeyboardInterrupt:
            self.safe_cprint("\n[*] 收到中断信号, 停止控制循环。", "yellow")
        except Exception as e:
            self.safe_cprint(
                f"\n[!!!] 控制循环发生严重错误: {e}", "red", attrs=["bold"]
            )
            import traceback

            traceback.print_exc(file=sys.stderr)
        finally:
            self._cleanup_control_loop()

    def _cleanup_control_loop(self):
        """run_control_loop退出时的清理工作"""
        sys.stdout.write(SHOW_CURSOR)
        sys.stdout.write("\n" * 8)  # --- MODIFIED: (9行 - 1 = 8) ---
        self.safe_cprint("🚪 已退出推理控制...", "green")
        self.safe_cprint("[*] 机器人回到初始位置...", "cyan")
        self.robot.switch_mode(RobotMode.PLANNING_POS)
        self.robot.back_home()
        sys.stdout.flush()

        with self.state_lock:
            self.is_running = False
            self.control_loop_thread = None
        self.safe_cprint("[*] 状态: IDLE (空闲). 等待 /start_task 命令...", "yellow")

    # ----------------------------------------------------------------
    # --- 4. 辅助与测试功能 ---
    # ----------------------------------------------------------------
    def go_home(self):
        self.safe_cprint("[*] 命令机器人回归初始位置...", "cyan")
        self.robot.back_home()
        self.safe_cprint("✅ 已发送回归指令。", "green")

    def test_connection(self, current_url: str):
        self.safe_cprint("\n[*] 测试服务器连接...", "cyan")
        try:
            total_time = 0
            for _ in range(10):
                start_time = time.perf_counter()
                resp = requests.get(
                    current_url + "/health",
                    proxies={"http": None, "https": None},
                    timeout=2.0,  # 2秒超时
                )
                if resp.status_code != 200:
                    self.safe_cprint(f"❌ 服务器异常: {resp.status_code}", "red")
                    return False
                total_time += (time.perf_counter() - start_time) * 1000
            self.safe_cprint(
                f"✅ 10次请求成功! 平均耗时 {total_time / 10:.2f}ms", "green"
            )
            return True
        except requests.RequestException as e:
            self.safe_cprint(f"❌ 连接失败: {e}", "red")
            return False

    # ----------------------------------------------------------------
    # --- 5. 程序生命周期管理 ---
    # ----------------------------------------------------------------
    def shutdown(self):
        self.safe_cprint("\n[*] 开始关闭 AIRBOT Controller...", "yellow")
        if self.is_running:
            self.safe_cprint("[*] 检测到任务仍在运行, 正在发送停止信号...", "yellow")
            self.stop_task()  # stop_task() 现在会阻塞直到线程退出

        self.stop_capture_thread()
        if hasattr(self, "robot"):
            self.robot.disconnect()
        self.safe_cprint("[*] Controller 已成功关闭。", "green")

    def __del__(self):
        # 作为最后的保障，但主要依赖 main 中的显式调用
        if (
            hasattr(self, "capture_thread")
            and self.capture_thread
            and self.capture_thread.is_alive()
        ):
            self.shutdown()


def create_task_listener(app: Flask, controller: AIRBOT_Controller):

    # self._send_request('POST', '/start_task', data={"task": task_to_start})
    @app.route("/start_task", methods=["POST"])
    def handle_start_task():
        data = request.json
        # print("--- [Flask 线程] /start_task 路由被触发0 ---", flush=True)
        if not data or "task" not in data:
            return (
                jsonify({"success": False, "message": "Missing 'task' in JSON body"}),
                400,
            )

        task_name = data.get("task")
        success, message = controller.start_task(task_name)

        if success:
            return jsonify({"success": True, "message": message}), 200
        else:
            return (
                jsonify({"success": False, "message": message}),
                409,
            )  # 409 Conflict (already running or failed)

    @app.route("/stop_task", methods=["POST"])
    def handle_stop_task():
        success, message = controller.stop_task()
        if success:
            return jsonify({"success": True, "message": message}), 200
        else:
            return (
                jsonify({"success": False, "message": message}),
                404,
            )  # 404 Not Found (not running)

    @app.route("/get_status", methods=["GET"])
    def handle_get_status():
        with controller.state_lock:
            is_running = controller.is_running
        current_task = controller.get_current_task()
        return (
            jsonify(
                {
                    "success": True,
                    "is_running": is_running,
                    "current_task": current_task,
                }
            ),
            200,
        )

    return app


def main():
    np.set_printoptions(linewidth=200, suppress=True)
    parser = ArgumentParser()
    parser.add_class_arguments(RemoteConfig, as_group=False)
    args = parser.parse_args()
    opt = RemoteConfig(**vars(args))

    # --- 初始化 Flask app ---
    # (关闭 Flask 的启动日志，避免与 Cprint 冲突)
    cli = sys.modules["flask.cli"]
    cli.show_server_banner = lambda *x: None
    app = Flask(__name__)

    controller = None
    try:
        controller = AIRBOT_Controller(
            target_ip=opt.target_ip,
            task_ports=opt.task_ports,
            task_thresholds=opt.task_thresholds,
            default_thresh=opt.default_thresh,
            default_stable_frames=opt.default_stable_frames,
            default_max_steps=opt.default_max_steps,
            default_task=opt.default_task,
            control_freq=opt.control_freq,
            camera_names=opt.camera_names,
            display_order=opt.display_order,
            jpg_quality=opt.jpg_quality,
        )

        # --- 创建并启动任务监听线程 ---
        create_task_listener(app, controller)
        listener_thread = threading.Thread(
            target=lambda: app.run(
                host="0.0.0.0", port=opt.listener_port, debug=False, use_reloader=False
            ),
            daemon=True,  # 设置为守护线程，主程序退出时它也会退出
        )
        cprint(
            f"\n[*] 🚀 任务监听服务已启动于 http://[Your_IP]:{opt.listener_port}",
            "cyan",
        )
        cprint(f"    - GET /get_status (查询当前状态)", "cyan")
        cprint(
            f'    - POST /start_task (切换任务, \'{{"task": "task_name"}}\')', "cyan"
        )
        cprint(f"    - POST /stop_task (停止当前任务)", "cyan")
        listener_thread.start()

        cprint("\n[*] 等待后台线程捕获第一帧数据...", "yellow")
        cprint("[*] Ready...", "yellow")
        while controller.latest_observation_cache is None:
            if not controller.capture_thread.is_alive():
                cprint("[!] 采集线程意外退出，请检查摄像头或机器人连接。", "red")
                return
            if not listener_thread.is_alive():
                cprint("[!] 任务监听服务启动失败，请检查端口是否被占用。", "red")
                return
            time.sleep(0.5)

        cprint("[✔] GO! 控制器处于 IDLE 状态，等待API命令...", "green")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        cprint("\n\n[*] 用户中断，准备退出程序...", "yellow")
    except Exception as e:
        cprint(f"\n[!!!] 发生未处理的严重错误: {e}", "red", attrs=["bold"])
        import traceback

        traceback.print_exc(file=sys.stderr)
    finally:
        if controller:
            controller.shutdown()
        cprint("\n程序已安全退出。", "green")


if __name__ == "__main__":
    main()
