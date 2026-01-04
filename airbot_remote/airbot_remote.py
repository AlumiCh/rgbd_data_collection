import time
import json
import numpy as np
import requests
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
    target_port: str = "6161"
    max_steps: int = 1000
    control_freq: int = 20
    jpg_quality: int = 80

    # camera_names: str = "cam_head cam_left cam_right"
    camera_names: str = "cam_head cam_low cam_left cam_right"
    # camera_names: str = "cam_head"
    display_order: str = "cam_left cam_head cam_low cam_right"

    def __post_init__(self):
        # 自动转换字符串为列表
        if isinstance(self.camera_names, str):
            self.camera_names = self.camera_names.split()
        if isinstance(self.display_order, str):
            self.display_order = self.display_order.split()


class AIRBOT_Controller:
    """
    AIRBOT 机器人远程控制器
    通过后台线程实时采集和显示图像，主线程负责与服务器通信和执行控制逻辑。
    """

    # 将窗口名称定义为类常量，避免硬编码字符串重复出现
    WINDOW_NAME = "AIRBOT Real-time Feeds"

    def __init__(
        self,
        server_url,
        max_steps,
        control_freq,
        camera_names,
        display_order,
        jpg_quality,
    ):
        self.server_url = server_url
        self.max_steps = max_steps
        self.control_freq = control_freq
        self.camera_names = camera_names
        self.display_order = display_order
        self.jpg_quality = jpg_quality

        self.session = requests.Session()

        cprint("[*] 初始化 AIRBOT 机器人...", "cyan")
        self.robot = AIRBOTPlay()

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

        cprint("[*] AIRBOT PLAY 控制器初始化完成", "green")
        self.start_capture_thread()

    def safe_cprint(self, *args, **kwargs):
        with self.print_lock:
            cprint(*args, **kwargs)

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
                bgr_images.append(cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR))
            else:
                # 如果某个摄像头图像不存在，创建一个黑色占位符
                if placeholder_template is not None:
                    placeholder = np.zeros_like(placeholder_template)
                else:  # 极端情况，所有摄像头都失效
                    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                bgr_images.append(placeholder)

        if bgr_images:
            combined_image = np.hstack(bgr_images)
            combined_image = cv2.resize(
                combined_image, None,
                fx=0.5, fy=0.5,
                interpolation=cv2.INTER_AREA
            )
            cv2.imshow(self.WINDOW_NAME, combined_image)

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
            "cam_low": "cam_low",
        }

        for cam_name in self.camera_names:
            logical_name = camera_mapping.get(cam_name)
            img = (
                obs.get(f"observation.images.{logical_name}") if logical_name else None
            )
            if img is not None:
                raw_images[cam_name] = img
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
            image = Image.fromarray(image)
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=self.jpg_quality)
        return buffer.getvalue()

    # ----------------------------------------------------------------
    # --- 3. 机器人控制与通信 (消费者) ---
    # ----------------------------------------------------------------
    def request_inference(self, files_to_send: list, data_to_send: dict):
        """向推理服务器请求动作预测"""
        try:
            resp = self.session.post(
                self.server_url + "/predict",
                files=files_to_send,
                data=data_to_send,
                proxies={"http": None, "https": None},
            )
            if resp.status_code == 200:
                return resp.json(), None
            return None, f"请求失败: {resp.status_code}, {resp.text}"
        except requests.RequestException as e:
            return None, f"网络请求异常: {e}"

    def get_action(self):
        """从缓存获取数据并发起推理请求"""
        files_to_send, data_to_send, raw_images = self.get_observation_from_cache()
        if not files_to_send:
            return 0, [], None, None, "缓存为空或图像数据不完整"

        start_time = time.perf_counter()
        result, error = self.request_inference(files_to_send, data_to_send)
        cost_time = time.perf_counter() - start_time

        qpos = json.loads(data_to_send["qpos"])
        if result and "actions" in result:
            return cost_time, qpos, result["actions"], raw_images, None
        return cost_time, qpos, None, raw_images, error or "推理服务器未返回有效动作"

    def run_control_loop(self):
        self.safe_cprint("\n[*] 进入远程推理控制...", "magenta")
        if not self.test_connection():
            self.safe_cprint("[!] 服务器连接失败，退出控制循环。", "red")
            return

        self.safe_cprint("[*] 切换到伺服模式...", "cyan")
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        self.go_home()

        self.action_buffer.clear()
        self.safe_cprint("[*] 动作缓存已清空。", "cyan")

        self.safe_cprint("提示：在控制过程中按 q + 回车 可随时退出", "yellow")
        print("\n" * 6)  # 为状态显示留出空间

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
                action, error, cost_time, qpos = None, None, 0, []

                # --- 从服务器或者动作缓存中获取动作 ---
                if self.action_buffer:
                    action = self.action_buffer.popleft()
                    _, data_to_send, _ = self.get_observation_from_cache()
                    qpos = json.loads(data_to_send["qpos"]) if data_to_send else []
                else:
                    cost_time, qpos, actions, _, req_error = self.get_action()
                    if req_error:
                        error = req_error
                    else:
                        is_2d = (
                            isinstance(actions, list)
                            and len(actions) > 0
                            and isinstance(actions[0], list)
                        )
                        if is_2d:
                            # cprint(f"[*] 接收到 {len(actions)} 个动作块, 已存入缓存。", "magenta", end='\r')
                            self.action_buffer.extend(actions)
                            action = self.action_buffer.popleft()
                        else:
                            action = actions

                if error or action is None:
                    final_error = error or "未能获取有效动作"
                    status_msg = (
                        f"🔴 控制状态: 错误! {str(final_error).replace(chr(10), ' ')}"
                    )
                    action_str = "   - 动作(action): [无]"
                    fps_str = ""
                else:
                    # 只有在成功获取action时才执行
                    # if len(action) >= 14:
                    #     # 处理第7个数字 (索引6)
                    #     action[6] = 0.07 if action[6] > 0.04 else 0.0
                    #     # 处理第14个数字 (索引13)
                    #     action[13] = 0.07 if action[13] > 0.04 else 0.0

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

                if cost_time > 0:
                    last_request_cost_time = cost_time

                # --- 统一的状态信息准备 ---
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

        except KeyboardInterrupt:
            self.safe_cprint("\n[*] 收到中断信号, 停止控制循环。", "yellow")
        finally:
            self._cleanup_control_loop()

    def _cleanup_control_loop(self):
        """run_control_loop退出时的清理工作"""
        sys.stdout.write(SHOW_CURSOR)
        sys.stdout.write("\n" * 6)
        self.safe_cprint("🚪 已退出推理控制...", "green")
        self.safe_cprint("[*] 机器人回到初始位置...", "cyan")
        self.robot.switch_mode(RobotMode.PLANNING_POS)
        self.robot.back_home()
        sys.stdout.flush()

    # ----------------------------------------------------------------
    # --- 4. 辅助与测试功能 ---
    # ----------------------------------------------------------------
    def test_connection(self):
        self.safe_cprint("\n[*] 测试服务器连接...", "cyan")
        # ... (内部逻辑不变)
        try:
            total_time = 0
            for _ in range(10):
                start_time = time.perf_counter()
                resp = requests.get(
                    self.server_url + "/health", proxies={"http": None, "https": None}
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

    def go_home(self):
        self.safe_cprint("[*] 命令机器人回归初始位置...", "cyan")
        self.robot.back_home()
        self.safe_cprint("✅ 已发送回归指令。", "green")

    def test_send_data(self):
        self.safe_cprint("\n[*] 测试携带数据的单次通信用时...", "cyan")
        # ... (内部逻辑不变)
        files_to_send, data_to_send, _ = self.get_observation_from_cache()
        if not files_to_send:
            self.safe_cprint("❌ 缓存中没有数据，无法测试。", "red")
            return
        try:
            start_time = time.perf_counter()
            resp = requests.post(
                self.server_url + "/cost_time",
                files=files_to_send,
                data=data_to_send,
                proxies={"http": None, "https": None},
            )
            if resp.status_code == 200:
                self.safe_cprint(
                    f"✅ 发送数据耗时: {(time.perf_counter() - start_time)*1000:.2f} ms",
                    "green",
                )
            else:
                self.safe_cprint(f"❌ 请求失败: {resp.status_code}, {resp.text}", "red")
        except requests.RequestException as e:
            self.safe_cprint(f"❌ 请求异常: {e}", "red")

    def test_inference_request(self):
        self.safe_cprint("\n[*] 测试单次推理请求...", "cyan")
        # ... (内部逻辑不变)
        cost_time, qpos, actions, _, error = self.get_action()
        if error:
            self.safe_cprint(f"❌ 单次推理失败: {error}", "red")
            return

        if isinstance(actions[0], list):
            first_action = actions[0]
        else:
            first_action = actions

        self.safe_cprint("✅ 单次推理成功:", "green")
        print(f"   - 观测(qpos): {[f'{x:.2f}' for x in qpos]}")
        print(f"   - 动作(action): {[f'{x:.2f}' for x in first_action]}")
        print(f"   - 推理耗时: {cost_time*1000:.2f} ms")

        self.safe_cprint("[*] 正在进行10次连续推理测试...", "cyan")
        total_time, success_count = 0, 0
        for i in range(10):
            cost, _, _, _, err = self.get_action()
            if not err:
                total_time += cost
                success_count += 1
            time.sleep(1 / self.control_freq)

        if success_count > 0:
            self.safe_cprint(
                f"✅ {success_count}/10 次成功, 平均推理耗时: {total_time*1000/success_count:.2f} ms",
                "green",
            )
        else:
            self.safe_cprint("❌ 10次连续推理全部失败。", "red")

    def clear_cache(self):
        self.safe_cprint("\n🚮 准备清空模型缓存...", "cyan")
        self.go_home()
        
        # 1. 【新增功能】获取用户的可选输入
        try:
            # 提示用户输入，并移除首尾可能存在的空白字符
            task_description = input("    ➡️  请输入新的任务描述 (直接回车跳过): ").strip()
        except EOFError:
            # 处理 (e.g., Ctrl+D) 终止输入的情况
            task_description = ""
            self.safe_cprint("\n    ℹ️  输入已取消。", "yellow")

        # 2. 准备要发送的数据 (payload)
        payload = {}
        if task_description:
            # 如果用户输入了内容，将其添加到 payload
            payload['task_description'] = task_description
            self.safe_cprint(f"    🆕 将同时设置新任务: {task_description}", "blue")
        else:
            # 否则，打印提示信息
            self.safe_cprint("    ℹ️  未输入新任务, 仅清空缓存。", "yellow")

        # 3. 发送 HTTP 请求
        try:
            self.safe_cprint("    ...正在发送请求...", "cyan")
            resp = requests.post(
                self.server_url + "/clear_cache",
                proxies={"http": None, "https": None},
                json=payload  # <-- 核心改动：将 payload 作为 json 数据发送
            )
            
            # 4. 处理响应
            if resp.status_code == 200:
                self.safe_cprint("✅ 请求成功 (缓存已清理)", "green")
            else:
                self.safe_cprint(f"❌ 请求失败: {resp.status_code}, {resp.text}", "red")
        except requests.RequestException as e:
            self.safe_cprint(f"❌ 请求异常: {e}", "red")

    # ----------------------------------------------------------------
    # --- 5. 程序生命周期管理 ---
    # ----------------------------------------------------------------
    def shutdown(self):
        self.safe_cprint("\n[*] 开始关闭 AIRBOT Controller...", "yellow")
        self.stop_capture_thread()
        if hasattr(self, "robot"):
            self.robot.disconnect()
        self.safe_cprint("[*] Controller 已成功关闭。", "green")

    def __del__(self):
        # 作为最后的保障，但主要依赖 main 中的显式调用
        if self.capture_thread and self.capture_thread.is_alive():
            self.shutdown()


def print_menu():
    """打印功能菜单"""
    menu = f"""
================= 功能菜单 =================
  1️⃣  测试服务器连接
  2️⃣  测试发送数据用时
  3️⃣  机械臂复位
  4️⃣  测试推理请求
  5️⃣  开始推理控制 🦾（按 q + 回车 退出）
  6️⃣  清空模型缓存 🔄
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
        controller = AIRBOT_Controller(
            server_url=f"http://{opt.target_ip}:{opt.target_port}",
            max_steps=opt.max_steps,
            control_freq=opt.control_freq,
            camera_names=opt.camera_names,
            display_order=opt.display_order,
            jpg_quality=opt.jpg_quality,
        )

        cprint("\n[*] 等待后台线程捕获第一帧数据...", "yellow")
        cprint("[*] Ready...", "yellow")
        while controller.latest_observation_cache is None:
            if not controller.capture_thread.is_alive():
                cprint("[!] 采集线程意外退出，请检查摄像头或机器人连接。", "red")
                return
            time.sleep(0.5)
        cprint("[✔] GO!", "green")

        while True:
            print_menu()
            flush_input()
            choice = input()

            if choice == "0":
                cprint("\n[*] 我要退出程序咯~", "yellow")
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
                confirm = input("确认开始吗? (y/N): ").strip().lower()
                if confirm == "y":
                    controller.run_control_loop()
                else:
                    cprint("笑:) 还是练练再来吧~", "yellow")
            elif choice == "6":
                controller.clear_cache()
            else:
                cprint("诶~你刚刚输入了什么？是不是手抖了？再试一次吧❤️", "yellow")

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
