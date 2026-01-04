# 数采流程

```bash
cd /home/jojo/airbot/airbot-data-5.1.6.8a3 && conda activate airbot_data
# 启动 docker
./start_all_arm.sh
# tok4 用这个
python3 data-collection/airbot_data_collection/tests/test_task_follow_reset.py -lp 50050 50052 -fp 50051 50053
# 如果是 ptk 用这个
python3 data-collection/airbot_data_collection/tests/test_task_follow.py -lp 50050 50052 -fp 50051 50053
# 启动数采，修改 --dataset.directory
python3 data-collection/airbot_data_collection/main.py --path defaults/config_setup_ptk.yaml --dataset.directory pick_box_fixed
```

# 推理流程
```bash
cd /home/jojo/airbot/airbot-data-5.1.6.8a3 && conda activate airbot_data
# 启动 docker
./start_follow_arm.sh
cd /home/jojo/airbot/airbot-data-5.1.6.8a3 && conda activate airbot_data
# 启动客户端
python3 airbot_remote/airbot_remote.py
```

# 安装流程
[参考这里](https://docs.airbots.online/5.1.6/)
```bash
cd /home/jojo/airbot/software
sudo apt install ./airbot-configure_5.1.6-1_all.deb -y
conda create -n airbot_data python=3.10 && conda activate airbot_data
pip install airbot_py-5.1.2-py3-none-any.whl -i https://mirrors.huaweicloud.com/repository/pypi/simple
bash install.sh
# 亮机
airbot_server -i can0 -p 50000
# 双臂示教 can 口绑定，按照下面的这个顺序插
sudo ./scripts/bind_can_udev.sh --target can_left_lead can_left can_right_lead can_right
# 一个一个相机，修改 yaml 文件，需要注意带宽限制
v4l2-ctl --all -d /dev/video0 | grep -i "Bus info"
udevadm info --query=all --name=/dev/video0 | grep ID_PATH
```
