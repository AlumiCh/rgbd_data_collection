(airbot_data) jojo@jojo-System-Product-Name:~/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection$ python scripts/data_convert/mcap_to_dataset.py   --mcap /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_01/2.mcap   --calibration /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/scripts/dual_camera_calibration.yaml   --pointcloud-output /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_01_pcd   --joints-output /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_01_joints
============================================================
MCAP 数据处理工具
============================================================
深度缩放因子: 0.001
深度有效范围: 0.10m - 5.00m
============================================================

[1/4] 读取 MCAP 文件: /home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/data/data/test_20260110_01/2.mcap

错误: 'SeekingReader' object has no attribute 'metadata'
Traceback (most recent call last):
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/scripts/data_convert/mcap_to_dataset.py", line 711, in main
    converter.process_single_mcap(
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/scripts/data_convert/mcap_to_dataset.py", line 554, in process_single_mcap
    data = self.read_mcap_file(mcap_path)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/scripts/data_convert/mcap_to_dataset.py", line 145, in read_mcap_file
    for metadata_name, metadata_dict in reader.metadata.items():
AttributeError: 'SeekingReader' object has no attribute 'metadata'