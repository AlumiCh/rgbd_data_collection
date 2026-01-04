"""
让一个机械臂跟随另一个机械臂的运动。
- 程序启动时，所有臂会先移动到预设的初始位置。
- 运行时，按下 'q' 或 's' 键，所有臂会平滑地移动到对应的预设位置，然后自动恢复跟随状态。
- 按 Ctrl+C 时，所有臂会用5秒时间平滑移动到预设的退出位置，然后程序关闭。
"""

import logging
import time
from pynput import keyboard
from airbot_py.arm import AIRBOTPlay, RobotMode
from typing import List, Tuple

LOG_FORMAT = (
    "[%(asctime)s] [%(levelname)-8s] [%(name)s.%(funcName)s:%(lineno)d] - %(message)s"
)

logging.basicConfig(
    level=logging.INFO,
    format=LOG_FORMAT,
    datefmt="%Y-%m-%d %H:%M:%S",
    handlers=[logging.StreamHandler()],
)

# ==============================================================================
# 宏定义（常量）：预设位置 [关节1, ..., 关节6, 夹爪]
# ==============================================================================
ZERO_POS_7D = [0, 0, 0, 0, 0, 0, 0.06]
# PRESET_POSITIONS = {
#     "initial": {
#         "pair_0": ZERO_POS_7D,
#         "pair_1": ZERO_POS_7D,
#     },
#     "q": {
#         "pair_0": ZERO_POS_7D,
#         "pair_1": ZERO_POS_7D,
#     },
#     "s": {
#         "pair_0": ZERO_POS_7D,
#         "pair_1": ZERO_POS_7D,
#     },
#     "w": {
#         "pair_0": ZERO_POS_7D,
#         "pair_1": ZERO_POS_7D,
#     },
# }
PRESET_POSITIONS = {
    "initial": {
        "pair_0": [0.4, -0.8, 1.0, 1.5, -1.0, -1.5, 0.06],
        "pair_1": [-0.4, -0.8, 1.0, 1.5, -1.0, -1.5, 0.06],
    },
    "q": {
        "pair_0": [0.4, -0.8, 1.0, 1.5, -1.0, -1.5, 0.06],
        "pair_1": [-0.4, -0.8, 1.0, 1.5, -1.0, -1.5, 0.06],
    },
    "s": {
        "pair_0": [0.4, -0.8, 1.0, 1.5, -1.0, -1.5, 0.00],
        "pair_1": [-0.4, -0.8, 1.0, 1.5, -1.0, -1.5, 0.00],
    },
    "w": {
        "pair_0": [0.4, -0.8, 1.0, 1.5, -1.0, -1.5, 0.06],
        "pair_1": [-0.4, -0.8, 1.0, 1.5, -1.0, -1.5, 0.06],
    },
}
# <<< 新增：定义Ctrl+C退出时的最终归位姿态
EXIT_HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06]


def move_all_arms_to_position(
    robots: List[Tuple[AIRBOTPlay, AIRBOTPlay]],
    target_pos_list: list,
    logger,
    duration: float,
):
    """
    一个通用的函数，将所有连接的机械臂（引导和跟随）平滑移动到同一个目标位置。
    """
    logger.info(f"所有机械臂将在 {duration} 秒内平滑移动到: {target_pos_list}...")

    # 1. 获取所有臂的起始位置并切换到伺服模式
    start_positions = []
    all_arms = [arm for pair in robots for arm in pair]  # 将所有臂放在一个列表里
    for arm in all_arms:
        start_arm_pos = arm.get_joint_pos()
        start_eef_pos = arm.get_eef_pos() or [0.0]
        if not start_arm_pos:
            logger.error(f"无法获取机械臂 {arm.port} 的当前位置，中断移动。")
            return
        start_positions.append({"arm": start_arm_pos, "eef": start_eef_pos})
        arm.switch_mode(RobotMode.SERVO_JOINT_POS)

    # 2. 插值计算与执行
    frequency = 50
    num_steps = int(duration * frequency)
    sleep_time = 1.0 / frequency

    target_arm_pos = target_pos_list[:6]
    target_eef_pos = target_pos_list[6:]

    for step in range(num_steps + 1):
        progress = step / num_steps
        for i, arm in enumerate(all_arms):
            start_pos_dict = start_positions[i]

            # 计算关节和夹爪的插值
            interp_arm = [
                s + progress * (e - s)
                for s, e in zip(start_pos_dict["arm"], target_arm_pos)
            ]
            interp_eef = [
                s + progress * (e - s)
                for s, e in zip(start_pos_dict["eef"], target_eef_pos)
            ]

            # 发送伺服指令
            arm.servo_joint_pos(interp_arm)
            arm.servo_eef_pos(interp_eef)

        time.sleep(sleep_time)
    logger.info("平滑移动完成。")

    # 3. 可选：打印每个臂的最终末端姿态以验证
    # for i, arm in enumerate(all_arms):
    #     end_pos = arm.get_end_pose()
    #     logger.info(f"{i}_th arm's end pose after move: {end_pos}")


def move_to_preset(
    robots: List[Tuple[AIRBOTPlay, AIRBOTPlay]], key: str, logger, duration: float = 5.0
):
    """根据按键，将每对机械臂移动到其对应的预设位置。"""
    if key not in PRESET_POSITIONS:
        logger.warning(f"未找到按键 '{key}' 对应的预设位置。")
        return

    logger.info(f"所有机械臂将根据 '{key}' 的设定，在 {duration} 秒内平滑移动...")
    target_positions_map = PRESET_POSITIONS[key]

    for i, pair in enumerate(robots):
        pair_key = f"pair_{i}"
        target_pos = target_positions_map.get(pair_key)
        if target_pos:
            # 复用通用移动函数
            move_all_arms_to_position([pair], target_pos, logger, duration)
        else:
            logger.warning(f"未在预设中找到第 {i+1} 对臂 ('{pair_key}') 的目标位置。")


def set_follow_mode(robots: List[Tuple[AIRBOTPlay, AIRBOTPlay]], logger):
    """设置引导臂为重力补偿模式，跟随臂为伺服模式"""
    logger.info("正在设置机械臂为跟随模式...")
    for lead, follow in robots:
        lead.switch_mode(RobotMode.GRAVITY_COMP)
        follow.switch_mode(RobotMode.SERVO_JOINT_POS)
        # ... (set_params与之前相同)
        follow.set_params(
            {
                "servo_node.moveit_servo.scale.linear": 10.0,
                "servo_node.moveit_servo.scale.rotational": 10.0,
                "servo_node.moveit_servo.scale.joint": 1.0,
                "sdk_server.max_velocity_scaling_factor": 1.0,
                "sdk_server.max_acceleration_scaling_factor": 0.5,
            }
        )
    logger.info("跟随模式已启动。")


def follow():
    # ... (命令行参数和连接部分与之前完全相同，此处省略以保持简洁)
    import argparse

    logger = logging.getLogger(__name__)
    parser = argparse.ArgumentParser(description="多对机械臂引导/跟随控制示例")
    parser.add_argument("--lead-url", type=str, nargs="+", help="引导臂服务器的URL。")
    parser.add_argument("--follow-url", type=str, nargs="+", help="跟随臂服务器的URL。")
    parser.add_argument(
        "-lp",
        "--lead-port",
        type=int,
        nargs="+",
        required=True,
        help="引导臂的服务器端口。",
    )
    parser.add_argument(
        "-fp",
        "--follow-port",
        type=int,
        nargs="+",
        required=True,
        help="跟随臂的服务器端口。",
    )

    args = parser.parse_args()
    n = len(args.lead_port)
    if not args.lead_url:
        args.lead_url = ["localhost"] * n
    if not args.follow_url:
        args.follow_url = ["localhost"] * n
    if not (
        len(args.lead_url)
        == len(args.lead_port)
        == len(args.follow_url)
        == len(args.follow_port)
    ):
        raise ValueError("引导/跟随的URL和端口数量必须一致。")
    for i in range(n):
        if (
            args.lead_port[i] == args.follow_port[i]
            and args.lead_url[i] == args.follow_url[i]
        ):
            raise ValueError(f"第 {i} 对：引导臂和跟随臂的URL和端口不能完全相同。")

    robots: List[Tuple[AIRBOTPlay, AIRBOTPlay]] = []
    for i in range(n):
        lead = AIRBOTPlay(url=args.lead_url[i], port=args.lead_port[i])
        follow = AIRBOTPlay(url=args.follow_url[i], port=args.follow_port[i])
        print(
            f"正在连接引导臂 {args.lead_url[i]}:{args.lead_port[i]} 和跟随臂 {args.follow_url[i]}:{args.follow_port[i]}"
        )
        assert lead.connect()
        assert follow.connect()
        robots.append((lead, follow))

    command_key = None
    listener = keyboard.Listener(on_press=lambda key: on_press(key))

    def on_press(key):
        nonlocal command_key
        try:
            if key.char in ["q", "s", "w"]:
                logger.info(f"接收到 '{key.char}' 键命令，即将执行归位动作。")
                command_key = key.char
        except AttributeError:
            pass

    listener.start()

    factor = 0.072 / 0.0471
    move_to_preset(robots, "initial", logger, duration=3.0)
    set_follow_mode(robots, logger)
    logger.info(
        "初始化完成。可以开始拖动引导臂。按 'q' 's' 或 'w' 重新归位，按 Ctrl+C 退出。"
    )

    try:
        while True:
            if command_key:
                move_to_preset(robots, command_key, logger, duration=3.0)
                command_key = None
                set_follow_mode(robots, logger)

            for lead, follow in robots:
                lead_joint_pos = lead.get_joint_pos()
                if lead_joint_pos:
                    follow.servo_joint_pos(lead_joint_pos)

                eef_pos = lead.get_eef_pos()
                if eef_pos:
                    eef_pos[0] *= factor
                    follow.servo_eef_pos(eef_pos)

            time.sleep(0.01)

    except KeyboardInterrupt:
        logger.info("\n接收到 Ctrl+C 信号。")
    finally:
        logger.info("正在执行退出前的清理工作...")

        # <<< 修改：在断开连接前，执行最终的自动归位
        move_all_arms_to_position(robots, EXIT_HOME_POSITION, logger, duration=5.0)

        for lead, follow in robots:
            lead.switch_mode(RobotMode.PLANNING_POS)
            follow.switch_mode(RobotMode.PLANNING_POS)
            # logger.info(f"正在断开与机械臂 {lead.port} 和 {follow.port} 的连接...")
            assert lead.disconnect()
            assert follow.disconnect()
        listener.join()
        logger.info("程序已结束。")


if __name__ == "__main__":
    follow()
