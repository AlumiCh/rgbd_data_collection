import time
import json
import numpy as np
from collections import deque
from PIL import Image
import io
from dataclasses import dataclass
from jsonargparse import ArgumentParser
import termios
import sys
import select
import cv2
import os
import threading
from termcolor import cprint

# 导入 websocket, msgpack, 和 requests (用于简单的健康检查)
import websockets.sync.client
import msgpack
import msgpack_numpy
import requests

from airbot_robot import AIRBOTPlay
from airbot_py.arm import RobotMode

# 为 msgpack 打补丁以支持 NumPy
msgpack_numpy.patch()

# --- 终端打印相关的代码保持不变 ---
CURSOR_UP_ONE = "\x1b[1A"
ERASE_LINE = "\x1b[2K\r"
HIDE_CURSOR = "\x1b[?25l"
SHOW_CURSOR = "\x1b[?25h"


@dataclass
class RemoteConfig:
    """远程控制配置参数"""

    target_ip: str = "192.168.3.103"
    target_port: str = "6160"
    max_steps: int = 1000
    control_freq: int = 20
    jpg_quality: int = 80
    camera_names: str = "cam_head cam_left cam_right"
    display_order: str = "cam_left cam_head cam_right"

    def __post_init__(self):
        if isinstance(self.camera_names, str):
            self.camera_names = self.camera_names.split()
        if isinstance(self.display_order, str):
            self.display_order = self.display_order.split()


class AIRBOT_WS_Controller:
    """
    AIRBOT 机器人远程控制器 (纯 WebSocket 版本)
    保留了后台采集线程和完整的、通过WebSocket实现的测试功能。
    """

    WINDOW_NAME = "AIRBOT Real-time Feeds (WebSocket)"

    def __init__(
        self,
        server_uri,  # ws:// URI for WebSocket
        http_server_url,  # http:// URL for health checks
        max_steps,
        control_freq,
        camera_names,
        display_order,
        jpg_quality,
    ):
        self.server_uri = server_uri
        self.http_server_url = http_server_url  # 新增
        self.max_steps = max_steps
        self.control_freq = control_freq
        self.camera_names = camera_names
        self.display_order = display_order
        self.jpg_quality = jpg_quality

        self.ws_connection = None

        cprint("[*] 初始化 AIRBOT 机器人...", "cyan")
        self.robot = AIRBOTPlay()

        # --- 线程与缓存初始化 (保持不变) ---
        self.latest_observation_cache = None
        self.cache_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.capture_thread = None
        self.print_lock = threading.Lock()
        self.action_buffer = deque()

        cprint("[*] AIRBOT WebSocket 控制器初始化完成", "green")
        self.start_capture_thread()

    def safe_cprint(self, *args, **kwargs):
        with self.print_lock:
            cprint(*args, **kwargs)

    # ----------------------------------------------------------------
    # --- 1. 后台采集线程 (完全不变) ---
    # ----------------------------------------------------------------
    def start_capture_thread(self):
        if self.capture_thread and self.capture_thread.is_alive():
            return
        self.safe_cprint("[*] 启动后台摄像头采集和显示线程...", "cyan")
        self.stop_event.clear()
        self.capture_thread = threading.Thread(target=self._capture_and_display_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()

    def stop_capture_thread(self):
        if not (self.capture_thread and self.capture_thread.is_alive()):
            return
        self.safe_cprint("[*] 正在停止后台采集线程...", "yellow")
        self.stop_event.set()
        self.capture_thread.join(timeout=2)

    def _capture_and_display_loop(self):
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        while not self.stop_event.is_set():
            try:
                obs = self.robot.capture_observation()
                if not obs:
                    time.sleep(0.1)
                    continue
                payload, raw_images = self._process_observation(obs)
                with self.cache_lock:
                    self.latest_observation_cache = {
                        "payload": payload,
                        "raw_images": raw_images,
                    }
                if raw_images:
                    self._display_images(raw_images)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            except Exception as e:
                self.safe_cprint(f"\n[!] 后台采集线程发生错误: {e}", "red")
                time.sleep(1)
        cv2.destroyAllWindows()
        [cv2.waitKey(1) for _ in range(5)]

    def _display_images(self, raw_images):
        bgr_images = []
        template = next((img for img in raw_images.values() if img is not None), None)
        for name in self.display_order:
            img = raw_images.get(name)
            if img is not None:
                bgr_images.append(cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR))
            else:
                bgr_images.append(
                    np.zeros_like(template)
                    if template is not None
                    else np.zeros((480, 640, 3), dtype=np.uint8)
                )
        if bgr_images:
            cv2.imshow(self.WINDOW_NAME, np.hstack(bgr_images))

    # ----------------------------------------------------------------
    # --- 2. 数据处理与获取 (完全不变) ---
    # ----------------------------------------------------------------
    def get_observation_from_cache(self):
        with self.cache_lock:
            if not self.latest_observation_cache:
                return None, None
            c = self.latest_observation_cache.copy()
            return c.get("payload"), c.get("raw_images")

    def _process_observation(self, obs: dict):
        payload = {"images": {}, "qpos": []}
        raw_images = {}
        mapping = {
            "cam_head": "cam_high",
            "cam_right": "cam_right_wrist",
            "cam_left": "cam_left_wrist",
        }
        for name in self.camera_names:
            logical_name = mapping.get(name)
            img = obs.get(f"observation.images.{logical_name}")
            if img is not None:
                raw_images[name] = img
                payload["images"][name] = self._image_to_jpeg_bytes(img)
        qpos = obs.get("observation.state")
        if qpos is not None:
            payload["qpos"] = qpos
        return payload, raw_images

    def _image_to_jpeg_bytes(self, image):
        if isinstance(image, (bytes, bytearray)):
            return bytes(image)
        if isinstance(image, np.ndarray):
            if image.dtype != np.uint8:
                image = (image * 255).astype(np.uint8)
            image = Image.fromarray(image)
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=self.jpg_quality)
        return buffer.getvalue()

    # ----------------------------------------------------------------
    # --- 3. 机器人控制与通信 (核心修改) ---
    # ----------------------------------------------------------------
    def connect(self):
        if self.ws_connection:
            return True
        try:
            self.safe_cprint(
                f"[*] 正在连接到 WebSocket 服务器: {self.server_uri}...", "cyan"
            )
            self.ws_connection = websockets.sync.client.connect(
                self.server_uri, max_size=None, compression=None
            )
            self.safe_cprint("✅ WebSocket 连接成功!", "green")
            return True
        except Exception as e:
            self.safe_cprint(f"❌ WebSocket 连接失败: {e}", "red")
            return False

    def disconnect(self):
        if self.ws_connection:
            self.ws_connection.close()
            self.ws_connection = None

    def _send_receive(self, conn, msg_type, payload=None):
        """通用发送和接收函数"""
        message = {"type": msg_type, "payload": payload}
        message_bytes = msgpack.packb(message, use_bin_type=True)
        conn.send(message_bytes)
        response_bytes = conn.recv()
        return msgpack.unpackb(response_bytes, raw=False)

    def run_control_loop(self):
        self.safe_cprint("\n[*] 进入远程(WebSocket)推理控制...", "magenta")
        if not self.connect():
            return

        self.safe_cprint("[*] 切换到伺服模式...", "cyan")
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        self.go_home()
        self.action_buffer.clear()

        self.safe_cprint("提示：在控制过程中按 q + 回车 可随时退出", "yellow")
        print("\n" * 6)
        sys.stdout.write(HIDE_CURSOR)
        sys.stdout.flush()

        cost_time_window = deque(maxlen=100)
        step = 0
        last_request_cost_time = 0.0
        try:
            while step < self.max_steps:
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    if sys.stdin.readline().strip().lower() == "q":
                        break

                start_loop_t = time.perf_counter()

                payload, _ = self.get_observation_from_cache()
                if not payload:
                    time.sleep(0.01)
                    continue

                start_req_time = time.perf_counter()
                response = self._send_receive(self.ws_connection, "infer", payload)
                cost_time = time.perf_counter() - start_req_time

                qpos = payload.get("qpos", [])
                action = None
                error = None

                if response.get("type") == "infer_result":
                    actions = response["payload"].get("actions")

                    # --- 核心修正：判断 actions 是一维还是二维 ---
                    if isinstance(actions, np.ndarray) and actions.size > 0:
                        if actions.ndim == 1:
                            # 服务端返回单个动作 (一维数组)，直接添加到缓存
                            self.action_buffer.append(actions)
                        elif actions.ndim == 2:
                            # 服务端返回多个动作 (二维数组)，逐行添加到缓存
                            self.action_buffer.extend(actions)

                    # 只有当缓存非空时才尝试取出动作
                    if self.action_buffer:
                        action = self.action_buffer.popleft()
                    else:
                        error = "收到空的 'actions' 数组"
                else:
                    error = response.get("message", "未知错误")

                if error or action is None:
                    final_error = error or "未能获取有效动作"
                    status_msg = (
                        f"🔴 控制状态: 错误! {str(final_error).replace(chr(10), ' ')}"
                    )
                    action_str = "   - 动作(action): [无]"
                    fps_str = ""
                else:
                    self.robot.send_action(action)
                    step += 1
                    cost_time_window.append(cost_time)
                    net_requests = [c for c in cost_time_window if c > 0]
                    avg_cost_time = (
                        sum(net_requests) / len(net_requests) if net_requests else 0
                    )

                    status_msg = f"🟢 控制状态: 运行中... (第 {step} 步)"
                    action_str = f"   - 动作(action): {[f'{x:.2f}' for x in action]}"
                    fps_str = (
                        f"   - 平均请求帧数: {1/avg_cost_time:.2f} hz"
                        if avg_cost_time > 0
                        else ""
                    )

                last_request_cost_time = cost_time

                qpos_str = f"   - 观测(qpos): {[f'{x:.2f}' for x in qpos]}"
                buffer_str = f"   - 动作缓存: {len(self.action_buffer)} 个"
                inference_time_str = (
                    f"   - 本次请求耗时: {last_request_cost_time*1000:.2f} ms"
                )

                lines = [
                    status_msg,
                    qpos_str,
                    action_str,
                    buffer_str,
                    inference_time_str,
                    fps_str,
                ]
                sys.stdout.write(CURSOR_UP_ONE * 6)
                for line in lines:
                    sys.stdout.write(f"\r{ERASE_LINE}{line}\n")
                sys.stdout.flush()

                dt_s = time.perf_counter() - start_loop_t
                time.sleep(max(0, 1 / self.control_freq - dt_s))
        finally:
            self._cleanup_control_loop()

    def _cleanup_control_loop(self):
        sys.stdout.write(SHOW_CURSOR)
        sys.stdout.write("\n" * 6)
        self.safe_cprint("🚪 已退出推理控制...", "green")
        self.disconnect()  # 断开连接
        self.robot.switch_mode(RobotMode.PLANNING_POS)
        self.go_home()

    # ----------------------------------------------------------------
    # --- 4. 辅助与测试功能 (适配WebSocket) ---
    # ----------------------------------------------------------------
    def test_connection(self):
        self.safe_cprint("\n[*] 测试 WebSocket 连接 (ping/pong)...", "cyan")
        try:
            with websockets.sync.client.connect(
                self.server_uri, open_timeout=3
            ) as conn:
                start_time = time.perf_counter()
                response = self._send_receive(conn, "ping")
                rtt = (time.perf_counter() - start_time) * 1000
                if response.get("type") == "pong":
                    self.safe_cprint(
                        f"✅ 连接成功! (pong) 往返耗时: {rtt:.2f}ms", "green"
                    )
                else:
                    self.safe_cprint(f"❌ 测试失败: 收到未知响应 {response}", "red")
        except Exception as e:
            self.safe_cprint(f"❌ 测试失败: {e}", "red")

    def go_home(self):
        self.safe_cprint("\n[*] 命令机器人回归初始位置...", "cyan")
        self.robot.back_home()

    def test_send_data(self):
        self.safe_cprint("\n[*] 测试单次WebSocket数据往返耗时...", "cyan")
        payload, _ = self.get_observation_from_cache()
        if not payload:
            self.safe_cprint("❌ 缓存为空，无法测试。", "red")
            return
        try:
            with websockets.sync.client.connect(self.server_uri) as conn:
                start_time = time.perf_counter()
                response = self._send_receive(conn, "cost_time_test", payload)
                rtt = (time.perf_counter() - start_time) * 1000
                if response.get("type") == "cost_time_ack":
                    self.safe_cprint(f"✅ 数据往返成功! 耗时: {rtt:.2f} ms", "green")
                else:
                    self.safe_cprint(f"❌ 测试失败: 收到未知响应 {response}", "red")
        except Exception as e:
            self.safe_cprint(f"❌ 测试失败: {e}", "red")

    def test_inference_request(self):
        self.safe_cprint("\n[*] 测试 WebSocket 推理请求...", "cyan")
        payload, _ = self.get_observation_from_cache()
        if not payload:
            self.safe_cprint("❌ 缓存为空，无法测试。", "red")
            return

        try:
            with websockets.sync.client.connect(self.server_uri) as conn:
                # --- 第一次单次测试 ---
                self.safe_cprint("[*] 正在进行单次推理测试...", "cyan")
                start_time = time.perf_counter()
                response = self._send_receive(conn, "infer", payload)
                rtt = (time.perf_counter() - start_time) * 1000

                if response.get("type") != "infer_result":
                    self.safe_cprint(
                        f"❌ 单次推理失败: {response.get('message', '未知错误')}", "red"
                    )
                    return

                self.safe_cprint("✅ 单次推理成功:", "green")
                actions = response["payload"].get("actions").tolist()
                first_action = actions[0] if isinstance(actions[0], list) else actions
                print(
                    f"   - 观测(qpos): {[f'{x:.2f}' for x in payload.get('qpos', [])]}"
                )
                print(f"   - 动作(action): {[f'{x:.2f}' for x in first_action]}")
                print(f"   - 通信+推理总耗时: {rtt:.2f} ms")

                # --- 连续10次测试 ---
                self.safe_cprint("[*] 正在进行10次连续推理测试...", "cyan")
                total_time, success_count = 0, 0
                for i in range(10):
                    start_cont_time = time.perf_counter()

                    cont_payload, _ = self.get_observation_from_cache()
                    if not cont_payload:
                        continue

                    cont_response = self._send_receive(conn, "infer", cont_payload)

                    if cont_response.get("type") == "infer_result":
                        total_time += time.perf_counter() - start_cont_time
                        success_count += 1

                    time.sleep(
                        max(
                            0,
                            1 / self.control_freq
                            - (time.perf_counter() - start_cont_time),
                        )
                    )

                if success_count > 0:
                    avg_ms = (total_time * 1000) / success_count
                    self.safe_cprint(
                        f"✅ {success_count}/10 次成功, 平均耗时: {avg_ms:.2f} ms",
                        "green",
                    )
                else:
                    self.safe_cprint("❌ 10次连续推理全部失败。", "red")
        except Exception as e:
            self.safe_cprint(f"❌ 测试失败: {e}", "red")

    def clear_cache(self):
        self.safe_cprint(
            "\n[*] 提示: WebSocket方案的服务端暂无清理缓存的直接接口。", "yellow"
        )
        self.safe_cprint("[*] 通常重新启动服务端即可达到同样效果。", "yellow")

    # ----------------------------------------------------------------
    # --- 5. 程序生命周期管理 ---
    # ----------------------------------------------------------------
    def shutdown(self):
        self.safe_cprint("\n[*] 开始关闭 AIRBOT Controller...", "yellow")
        self.stop_capture_thread()
        self.disconnect()
        if hasattr(self, "robot"):
            self.robot.disconnect()
        self.safe_cprint("[*] Controller 已成功关闭。", "green")

    def __del__(self):
        if self.capture_thread and self.capture_thread.is_alive():
            self.shutdown()


def print_menu():
    """打印功能菜单"""
    menu = f"""
================= 功能菜单 =================
  1️⃣  测试 WebSocket 连接 (ping/pong)
  2️⃣  测试发送数据用时 (WebSocket RTT)
  3️⃣  机械臂复位
  4️⃣  测试推理请求 (单次 + 连续10次)
  5️⃣  开始推理控制 🦾（按 q + 回车 退出）
  6️⃣  清空模型缓存 (提示)
  0️⃣  退出程序 🚪
============================================
请输入对应数字后回车 👉 """
    print(menu, end="")


def flush_input():
    """清空stdin输入缓冲区，仅适用于Unix系统。"""
    try:
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except:
        pass


def main():
    np.set_printoptions(linewidth=200, suppress=True)
    parser = ArgumentParser()
    parser.add_class_arguments(RemoteConfig, as_group=False)
    args = parser.parse_args()
    opt = RemoteConfig(**vars(args))

    controller = None
    try:
        controller = AIRBOT_WS_Controller(
            server_uri=f"ws://{opt.target_ip}:{opt.target_port}/ws",
            http_server_url=f"http://{opt.target_ip}:{opt.target_port}",
            max_steps=opt.max_steps,
            control_freq=opt.control_freq,
            camera_names=opt.camera_names,
            display_order=opt.display_order,
            jpg_quality=opt.jpg_quality,
        )

        cprint("\n[*] 等待后台线程捕获第一帧数据...", "yellow")
        while controller.latest_observation_cache is None:
            if not controller.capture_thread.is_alive():
                cprint("[!] 采集线程意外退出，请检查摄像头或机器人连接。", "red")
                return
            time.sleep(0.5)
        cprint("[✔] 准备就绪!", "green")

        while True:
            print_menu()
            flush_input()
            choice = input()
            if choice == "0":
                break
            elif choice == "1":
                controller.test_connection()
            elif choice == "2":
                controller.test_send_data()
            elif choice == "3":
                controller.go_home()
            elif choice == "4":
                controller.test_inference_request()
            elif choice == "5":
                cprint("⚠️  警告: 这将开始真实的机器人控制!", "yellow", attrs=["bold"])
                if input("确认开始吗? (y/N): ").strip().lower() == "y":
                    controller.run_control_loop()
                else:
                    cprint("操作已取消。", "yellow")
            elif choice == "6":
                controller.clear_cache()
            else:
                cprint("无效输入，请重试。", "yellow")

    except KeyboardInterrupt:
        cprint("\n\n[*] 用户中断，准备退出程序...", "yellow")
    except Exception as e:
        cprint(f"\n[!!!] 发生未处理的严重错误: {e}", "red", attrs=["bold"])
    finally:
        if controller:
            controller.shutdown()
        cprint("\n程序已安全退出。", "green")


if __name__ == "__main__":
    main()
