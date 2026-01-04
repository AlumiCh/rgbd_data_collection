from enum import Enum
from pprint import pformat
from bidict import bidict
from pydantic import BaseModel
from pynput import keyboard
from airbot_data_collection.basis import SystemMode
from airbot_data_collection.managers.basis import DemonstrateManagerBasis
from airbot_data_collection.state_machine.fsm import DemonstrateAction as Action
from airbot_data_collection.utils import bcolors


class KeyboardCallbackConfig(BaseModel):
    action_key: dict[Action, str] = {
        Action.sample: keyboard.Key.space.name,
        Action.save: "s",
        Action.abandon: "q",
        Action.remove: "r",
        Action.capture: "p",
        Action.finish: "z",
    }
    instruction: dict[str, str] = {
        "b": "Back to sample the last round (override the last saved file)",
        "i": "Show this instruction again",
        "g": "Switch passive (gravity composation) / resetting mode of the leaders",
        "f": "Start / stop following",
        "f2": "Lock / unlock the keyboard control",
    }
    # TODO: auto add mapped keys to the instruction
    key_mapping: dict[str, str] = {
        keyboard.Key.esc.name: "z",
        keyboard.Key.enter.name: "s",
        keyboard.Key.shift.name: "q",
    }

    def model_post_init(self, context):
        action_info = {
            Action.sample: "Start sampling",
            Action.save: "Save sampled data in the current round",
            Action.abandon: "Abandon current sampling without saving",
            Action.finish: "Finish the current round and save all data",
            Action.remove: "Remove the last saved episode",
            Action.capture: "Capture current component observations",
        }
        for action, key in self.action_key.items():
            self.instruction[key] = action_info[action]


class KeyboardCallbackManager(DemonstrateManagerBasis):
    """Handles key press events for controlling data collection.

    This class listens for specific key presses and triggers actions of the demontrate fsm.
    These actions include starting or stopping data collection, printing
    current component states, removing the last saved episode, etc.
    """

    config: KeyboardCallbackConfig

    def on_configure(self):
        self.show_instruction()
        self.listener = keyboard.Listener(on_press=self.keypress_callback)
        self.listener.start()
        self.key_to_action = bidict(self.config.action_key).inverse
        self._locked = False
        return True

    def update(self) -> bool:
        return True

    def show_instruction(self) -> None:
        """Displays the instructions for the key press actions.

        This function provides a user-friendly guide to inform the user about the available
        key press actions for controlling the system.
        """
        self.get_logger().info(
            bcolors.OKCYAN + f" \n{pformat(self.config.instruction)}"
        )

    def keypress_callback(self, key: keyboard.Key) -> None:
        """处理按键事件并触发相应的操作。

        此函数监听按键事件，并在 demonstrate 状态机中启动相应的操作
        这些操作包括启动或停止数据收集、打印状态、移除片段或更改机器人状态。

        参数：
            key: 用户触发的按键事件。

        返回值：
            None: 此函数不返回任何值。
        """
        key = self._key_to_str(key).lower()
        if key == "f2":
            self._locked = not self._locked
            self.get_logger().info(
                bcolors.OKGREEN
                + f"Keyboard control is now {'locked' if self._locked else 'unlocked'}."
            )
            return
        elif self._locked:
            return

        # 读取按键并执行
        action = self.key_to_action.get(self.config.key_mapping.get(key, key), None)
        if action is Action.capture:
            self.fsm.act(action)
            data = {}
            # only print low dim data
            for key, value in self.fsm.last_capture.items():
                if "image" not in key and "depth" not in key:
                    data[key] = value
            self.get_logger().info(bcolors.OKBLUE + f"\n{pformat(data)}")
        elif key == "i":
            self.show_instruction()
        elif key == "b":
            self.get_logger().warning("Not implemented yet")
        elif key == "g":
            cur_mode = (
                SystemMode.PASSIVE
                if self.fsm.demonstrator.current_mode is not SystemMode.PASSIVE
                else SystemMode.RESETTING
            )
            self.fsm.demonstrator.switch_mode(cur_mode)
        elif key == "f":
            if self.fsm.demonstrator.handler.is_stopped():
                self.fsm.demonstrator.handler.start()
            else:
                self.fsm.demonstrator.handler.stop()
        elif key in {"ctrl", "c"}:
            pass
        else:
            if action is not None:
                self.get_logger().info(f"Executing action: {action.name}")
                self.fsm.act(action)
            else:
                self.get_logger().warning(f"Invalid key pressed: {key}")

    def on_shutdown(self) -> bool:
        self.listener.stop()
        # TODO: why can not be stopped?
        # self.listener.join(2)
        # return not self.listener.is_alive()
        return True

    def _key_to_str(self, key):
        if isinstance(key, str):
            return key
        elif isinstance(key, Enum):
            return key.name
        else:
            try:
                key_char = key.char
                if key_char is None:
                    self.get_logger().warning(
                        "Unknown key pressed. There may be a situation where the number keys on the numeric keypad cannot be recognized properly."
                    )
                    key_char = str(key)
            except AttributeError:
                key_char = str(key)
            return key_char
