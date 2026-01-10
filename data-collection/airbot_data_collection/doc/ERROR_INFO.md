(airbot_data) jojo@jojo-System-Product-Name:~/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection$ python scripts/data_convert/mcap_to_dataset.py \
>   --mcap /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_03/0.mcap \
>   --calibration /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/scripts/dual_camera_calibration.yaml \
>   --pointcloud-output /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_03_pcd \
>   --joints-output /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_03_joints
============================================================
MCAP 数据处理工具
============================================================
深度缩放因子: 0.001
深度有效范围: 0.10m - 5.00m
============================================================

[1/4] 读取 MCAP 文件: /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_03/0.mcap
  读取完成
  - RGB 图像: 0 帧
  - 深度图: 512 帧
  - 相机: 

[2/4] 转换为点云（双相机融合模式）...

错误: 需要双相机，但只找到 0 个相机: []
Traceback (most recent call last):
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/scripts/data_convert/mcap_to_dataset.py", line 714, in main
    converter.process_single_mcap(
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/scripts/data_convert/mcap_to_dataset.py", line 570, in process_single_mcap
    raise ValueError(f"需要双相机，但只找到 {len(cameras)} 个相机: {cameras}")
ValueError: 需要双相机，但只找到 0 个相机: []