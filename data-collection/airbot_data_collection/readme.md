<!--
 * @Author: Zhaoq msnakes@qq.com
 * @Date: 2025-09-22 17:12:53
 * @LastEditors: Zhaoq msnakes@qq.com
 * @LastEditTime: 2025-09-22 18:59:33
 * @FilePath: /airbot_data_collection/readme.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
# 查询相机
```bash
v4l2-ctl --all -d /dev/video0 | grep -i "Bus info"
udevadm info --query=all --name=/dev/video0 | grep ID_PATH
```

# 启动机械臂示教 & 数采
```bash
python3 tests/test_task_follow.py -lp 50050 50052 -fp 50051 50053
# or
python3 tests/test_task_follow_reset.py -lp 50050 50052 -fp 50051 50053
# 数采
python3 main.py --path defaults/config_setup_ptk.yaml --dataset.directory test
```

# 单独起机械臂
```bash
airbot_fsm -i can_left_lead -p 50050
airbot_fsm -i can_right -p 50053
airbot_fsm -i can_left -p 50051
airbot_fsm -i can_right_lead -p 50052
```
