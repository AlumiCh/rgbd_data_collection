(airbot_data) jojo@jojo-System-Product-Name:~/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection$ python main.py --path defaults/config_single_arm_dual_rgbd.yaml \--dataset.directory="data/test_20260107" \--sample-limit.size=1000
[INFO] 2026-01-07 17:31:31,510 airbot-data-collection: Preparing cv2.imshow (opencv.py:28)
[INFO] 2026-01-07 17:31:31,510 airbot-data-collection: Showing Prepared image (opencv.py:32)
[INFO] 2026-01-07 17:31:31,548 airbot-data-collection: Prepared image is ready (opencv.py:35)
[INFO] 2026-01-07 17:31:31,548 airbot-data-collection: cv2.imshow is ready (opencv.py:40)
[INFO] 2026-01-07 17:31:31,579 airbot-data-collection: Version: 0.6.3 (main.py:38)
[WARNING] 2026-01-07 17:31:31,647 airbot_data_collection.airbot.samplers.mcap_sampler: It is detected that the `UPLOAD` package is not installed, and the cloud upload function will not be available. If you need to use the upload function, please contact us to install it. (mcap_sampler.py:30)
[INFO] 2026-01-07 17:31:31,719 KeyboardCallbackManager:  
{'b': 'Back to sample the last round (override the last saved file)',
 'f': 'Start / stop following',
 'f2': 'Lock / unlock the keyboard control',
 'g': 'Switch passive (gravity composation) / resetting mode of the leaders',
 'i': 'Show this instruction again',
 'p': 'Capture current component observations',
 'q': 'Abandon current sampling without saving',
 'r': 'Remove the last saved episode',
 's': 'Save sampled data in the current round',
 'space': 'Start sampling',
 'z': 'Finish the current round and save all data'} (keyboard.py:75)
[INFO] 2026-01-07 17:31:31,720 airbot-data-collection: Update rate: 30.0 Hz (main.py:56)
[INFO] 2026-01-07 17:31:31,720 SelfManager: Configuring the demonstrate interface. (basis.py:91)
[INFO] 2026-01-07 17:31:32,461 airbot_py.arm: Client ID: 3f711499-61ef-4cf4-a96f-3da59fa95408 (arm.py:169)
[INFO] 2026-01-07 17:31:32,461 AIRBOTPlay: Connecting AIRBOT at localhost:50052 (airbot_play.py:179)
[ERROR] 2026-01-07 17:31:33,488 root: gRPC error: Parameter client not available (arm.py:144)
[INFO] 2026-01-07 17:31:33,502 AIRBOTPlay: Robot info: {'product_type': 'replay', 'sn': 'PB11C02542000207', 'is_sim': False, 'interfaces': ['can_right_lead'], 'arm_types': ['replay'], 'eef_types': ['PE2'], 'fw_versions': ['0300', '0304', '0304', '0304', '0304', '32304', '0304', '0304', 'N/A']} (airbot_play.py:112)
[INFO] 2026-01-07 17:31:33,502 airbot_py.arm: Client ID: efea6a5d-03f3-4320-9816-f2d11021852d (arm.py:169)
[INFO] 2026-01-07 17:31:33,502 AIRBOTPlay: Connecting AIRBOT at localhost:50053 (airbot_play.py:179)
[INFO] 2026-01-07 17:31:33,674 AIRBOTPlay: Robot info: {'product_type': 'play', 'sn': 'PZ51C02532000771', 'is_sim': False, 'interfaces': ['can_right'], 'arm_types': ['play'], 'eef_types': ['G2'], 'fw_versions': ['0515', '04115', '04115', '04115', '5015', '5015', '5015', '5015', '0502']} (airbot_play.py:112)
[INFO] 2026-01-07 17:31:34,352 RealSense: Configuring the interface... (wrappers.py:83)
[INFO] 2026-01-07 17:31:34,934 ConcurrentWrappedClass: Receiving info and first observation... (wrappers.py:50)
[INFO] 2026-01-07 17:31:35,554 ConcurrentWrappedClass: Sending shm observation... (wrappers.py:67)
[INFO] 2026-01-07 17:31:36,192 RealSense: Configuring the interface... (wrappers.py:83)
[INFO] 2026-01-07 17:31:36,363 ConcurrentWrappedClass: Receiving info and first observation... (wrappers.py:50)
[INFO] 2026-01-07 17:31:37,851 ConcurrentWrappedClass: Sending shm observation... (wrappers.py:67)
[INFO] 2026-01-07 17:31:37,852 ComponentGroupManager: Setting post capture for group arm: keys=['eef/joint_state/position'] target_ranges=[{0: (0.0, 0.072)}] (grouped.py:309)
[INFO] 2026-01-07 17:31:37,902 SelfManager: Activating the demonstrate interface. (basis.py:93)
[INFO] 2026-01-07 17:31:37,910 DemonstrateInterface: Warming up... (interface.py:134)                                                                                                    
[INFO] 2026-01-07 17:31:38,246 DemonstrateInterface: Current sample round: 0 (interface.py:344)
[WARNING] 2026-01-07 17:31:38,246 airbot-data-collection: Update took too long: exceed 6.492978593646937 s. (main.py:92)
[INFO] 2026-01-07 17:31:38,588 AIRBOTOpenCVVisualizer: Update loop started (opencv.py:108)
[INFO] 2026-01-07 17:31:43,852 KeyboardCallbackManager: Executing action: finish (keyboard.py:132)
[INFO] 2026-01-07 17:31:43,856 RealSense: Shutting down the interface... (wrappers.py:97)
[INFO] 2026-01-07 17:31:44,671 RealSense: Shutting down the interface... (wrappers.py:97)
[INFO] 2026-01-07 17:31:45,006 DemonstrateInterface: Finished the demonstration: from 0 to 0 (interface.py:334)
[INFO] 2026-01-07 17:31:45,050 AIRBOTOpenCVVisualizer: Update loop stopped (opencv.py:118)
Round 0:   0%|                                                                                                                                                | 0/1000 [00:07<?, ?step/s]
[WARNING] 2026-01-07 17:31:45,210 transitions.core: Can't trigger event t_capture from state finalized! (core.py:454)
[WARNING] 2026-01-07 17:31:45,210 transitions.DemonstrateFSM: Action failed: capture (basis.py:244)
[ERROR] 2026-01-07 17:31:45,210 SelfManager: Failed to capture the current component observations. (basis.py:105)
[WARNING] 2026-01-07 17:31:45,210 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[INFO] 2026-01-07 17:31:45,210 airbot-data-collection: Data collection finished. (main.py:75)
[INFO] 2026-01-07 17:31:45,210 airbot-data-collection: Shutting down: self_manager. (main.py:97)
[INFO] 2026-01-07 17:31:45,210 airbot-data-collection: Shutting down: keyboard. (main.py:97)
[INFO] 2026-01-07 17:31:45,210 airbot-data-collection: Summary:
{'Average update freq': '56.9237 Hz',
 'Average update time': '0.0176 s',
 'Total time taken': '13.4904 s'} (main.py:110)
[INFO] 2026-01-07 17:31:45,210 airbot-data-collection: Done. (main.py:111)
