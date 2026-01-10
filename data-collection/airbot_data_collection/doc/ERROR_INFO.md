(airbot_data) jojo@jojo-System-Product-Name:~/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection$ python main.py --path defaults/config_single_arm_dual_rgbd.yaml --dataset.directory data/test_20260110_02
[INFO] 2026-01-10 17:55:10,888 airbot-data-collection: Preparing cv2.imshow (opencv.py:28)
[INFO] 2026-01-10 17:55:10,888 airbot-data-collection: Showing Prepared image (opencv.py:32)
[INFO] 2026-01-10 17:55:10,997 airbot-data-collection: Prepared image is ready (opencv.py:35)
[INFO] 2026-01-10 17:55:10,998 airbot-data-collection: cv2.imshow is ready (opencv.py:40)
[INFO] 2026-01-10 17:55:11,028 airbot-data-collection: Version: 0.6.3 (main.py:38)
[WARNING] 2026-01-10 17:55:11,095 airbot_data_collection.airbot.samplers.mcap_sampler: It is detected that the `UPLOAD` package is not installed, and the cloud upload function will not be available. If you need to use the upload function, please contact us to install it. (mcap_sampler.py:30)
[INFO] 2026-01-10 17:55:11,175 KeyboardCallbackManager:  
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
[INFO] 2026-01-10 17:55:11,176 airbot-data-collection: Update rate: 30.0 Hz (main.py:56)
[INFO] 2026-01-10 17:55:11,176 SelfManager: Configuring the demonstrate interface. (basis.py:91)
[INFO] 2026-01-10 17:55:12,687 RealSense: Configuring the interface... (wrappers.py:83)
[INFO] 2026-01-10 17:55:13,261 ConcurrentWrappedClass: Receiving info and first observation... (wrappers.py:50)
[INFO] 2026-01-10 17:55:13,878 ConcurrentWrappedClass: Sending shm observation... (wrappers.py:67)
[INFO] 2026-01-10 17:55:14,645 RealSense: Configuring the interface... (wrappers.py:83)
[INFO] 2026-01-10 17:55:14,793 ConcurrentWrappedClass: Receiving info and first observation... (wrappers.py:50)
[INFO] 2026-01-10 17:55:16,290 ConcurrentWrappedClass: Sending shm observation... (wrappers.py:67)
[INFO] 2026-01-10 17:55:16,290 airbot_py.arm: Client ID: fef12863-a57c-48dc-9146-b2467762521a (arm.py:169)
[INFO] 2026-01-10 17:55:16,291 AIRBOTPlay: Connecting AIRBOT at localhost:50050 (airbot_play.py:179)
[ERROR] 2026-01-10 17:55:17,321 root: gRPC error: Parameter client not available (arm.py:144)
[INFO] 2026-01-10 17:55:17,337 AIRBOTPlay: Robot info: {'product_type': 'replay', 'sn': 'PB11C02542000205', 'is_sim': False, 'interfaces': ['can_left_lead'], 'arm_types': ['replay'], 'eef_types': ['PE2'], 'fw_versions': ['0300', '0304', '0304', '0304', '180304', '0304', '0304', '180304', 'N/A']} (airbot_play.py:112)
[INFO] 2026-01-10 17:55:17,337 airbot_py.arm: Client ID: 2dcd7aa3-87b6-4804-b129-a121b3a2fb96 (arm.py:169)
[INFO] 2026-01-10 17:55:17,338 AIRBOTPlay: Connecting AIRBOT at localhost:50051 (airbot_play.py:179)
[INFO] 2026-01-10 17:55:17,714 AIRBOTPlay: Robot info: {'product_type': 'play', 'sn': 'PZ51C02532000770', 'is_sim': False, 'interfaces': ['can_left'], 'arm_types': ['play'], 'eef_types': ['G2'], 'fw_versions': ['0515', '04114', '04114', '04114', '5015', '5015', '5015', '5015', '0502']} (airbot_play.py:112)
[INFO] 2026-01-10 17:55:17,715 ComponentGroupManager: Setting post capture for group arm: keys=['eef/joint_state/position'] target_ranges=[{0: (0.0, 0.072)}] (grouped.py:309)
[INFO] 2026-01-10 17:55:17,781 SelfManager: Activating the demonstrate interface. (basis.py:93)
[INFO] 2026-01-10 17:55:17,790 DemonstrateInterface: Warming up... (interface.py:134)                                                                                                    
[INFO] 2026-01-10 17:55:18,125 DemonstrateInterface: Current sample round: 1 (interface.py:344)
[WARNING] 2026-01-10 17:55:18,125 airbot-data-collection: Update took too long: exceed 6.9157884456649 s. (main.py:92)
[INFO] 2026-01-10 17:55:18,512 AIRBOTOpenCVVisualizer: Update loop started (opencv.py:108)
[INFO] 2026-01-10 17:55:27,919 KeyboardCallbackManager: Executing action: sample (keyboard.py:132)
[INFO] 2026-01-10 17:55:27,919 GroupedDemonstrator: Switching all leaders to SystemMode.PASSIVE mode (grouped.py:585)
[INFO] 2026-01-10 17:55:27,919 GroupedDemonstrator: Setting l mode to SystemMode.PASSIVE (grouped.py:462)
[INFO] 2026-01-10 17:55:27,935 airbot_py.arm: Switched to GRAVITY_COMP successfully. (arm.py:606)
[INFO] 2026-01-10 17:55:27,936 DemonstrateInterface: Start sampling round: 1 (interface.py:152)
Round 1:  12%|██████████████▍                                                                                                     | 124/1000 [00:04<00:29, 29.64step/s, Percentage=12.4%][WARNING] 2026-01-10 17:55:32,120 airbot-data-collection: Update took too long: exceed 0.01994323866844449 s. (main.py:92)
Round 1:  25%|█████████████████████████████                                                                                       | 251/1000 [00:08<00:25, 29.85step/s, Percentage=25.1%][INFO] 2026-01-10 17:55:36,373 KeyboardCallbackManager: Executing action: save (keyboard.py:132)
Round 1:  25%|█████████████████████████████▏                                                                                      | 252/1000 [00:08<00:25, 29.85step/s, Percentage=25.2%][INFO] 2026-01-10 17:55:36,380 DemonstrateInterface: Waiting for the update queue to finish...(size:0) (interface.py:235)
[INFO] 2026-01-10 17:55:36,381 DemonstrateInterface: Update queue finished in 0.00 seconds (interface.py:241)
[WARNING] 2026-01-10 17:55:36,382 AIRBOTMcapDataSampler: 没有采集到任何帧数据! (mcap_sampler.py:160)
[INFO] 2026-01-10 17:55:36,382 AIRBOTMcapDataSampler: 已保留第 1 帧 RGBD + 最后 1 帧关节角 (mcap_sampler.py:218)
[ERROR] 2026-01-10 17:55:36,385 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_save')@139961225599856>, <State('sampling')@139961225595584>, None)@139959703249600> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 256, in save
    save_path, self._sampler.save(save_path, self._round_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/airbot/samplers/mcap_sampler.py", line 253, in save
    log_stamps = data.pop("log_stamps")
KeyError: 'log_stamps'
[WARNING] 2026-01-10 17:55:36,388 transitions.DemonstrateFSM: Action failed: save (basis.py:244)
Round 1:  43%|██████████████████████████████████████████████████▎                                                                 | 434/1000 [00:14<00:18, 29.88step/s, Percentage=43.4%]^CProcess AIRBOTOpenCVVisualizer:
Process ConcurrentWrappedClassConcurrent:
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/process.py", line 314, in _bootstrap
    self.run()
[ERROR] 2026-01-10 17:55:42,494 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959699939840> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 538, in _fully_process
    func(group, component, comp_name, "result", True)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 498, in add_data
    for key, value in func(5.0).items():
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 115, in result
    if self._rpc.client.wait(timeout=timeout):
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/utils/event_rpc.py", line 72, in wait
    return self._rsp_event.wait(timeout=timeout)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/synchronize.py", line 349, in wait
    self._cond.wait(timeout)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/synchronize.py", line 261, in wait
    return self._wait_semaphore.acquire(True, timeout)
KeyboardInterrupt
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/process.py", line 108, in run
    self._target(*self._args, **self._kwargs)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 92, in _concurrent_loop
    while rpc_server.wait():
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/utils/event_rpc.py", line 33, in wait
    ret = self._req_event.wait(timeout=timeout)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/synchronize.py", line 349, in wait
    self._cond.wait(timeout)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/synchronize.py", line 261, in wait
    return self._wait_semaphore.acquire(True, timeout)
KeyboardInterrupt
[WARNING] 2026-01-10 17:55:42,495 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,496 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/process.py", line 314, in _bootstrap
    self.run()
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/process.py", line 108, in run
    self._target(*self._args, **self._kwargs)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/visualizers/opencv.py", line 116, in update_loop_shm
    time.sleep(sleep_time)
KeyboardInterrupt
Process ConcurrentWrappedClassConcurrent:
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/process.py", line 314, in _bootstrap
    self.run()
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/process.py", line 108, in run
    self._target(*self._args, **self._kwargs)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 93, in _concurrent_loop
    for key, value in interface.capture_observation(5.0).items():
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/airbot/sensors/cameras/realsense.py", line 16, in capture_observation
    output = super().capture_observation(timeout)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/devices/cameras/intelrealsense.py", line 187, in capture_observation
    frame = self.align.process(frame)
KeyboardInterrupt
[WARNING] 2026-01-10 17:55:42,515 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,516 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959699940080> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,516 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,517 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,549 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,549 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959699947040> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,549 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,549 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,583 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,583 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959700112864> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,584 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,584 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,616 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,616 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959701328256> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,616 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,616 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,649 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,649 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959700113296> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,650 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,650 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,682 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,683 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959700113536> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,683 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,683 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,716 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,716 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959700113824> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,716 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,717 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,749 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,750 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959700113632> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,750 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,751 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,783 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,783 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959700114064> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,784 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,784 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,816 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,817 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959700114496> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,817 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,818 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,850 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,850 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959699940080> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,850 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,850 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,883 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,884 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959699939840> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,885 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,885 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,916 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,917 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959699939696> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,918 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,918 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,950 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,950 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959700113488> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,951 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,951 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-10 17:55:42,983 EventRpcClient: previous response not received yet. (event_rpc.py:57)
[ERROR] 2026-01-10 17:55:42,984 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_update')@139961225599280>, <State('sampling')@139961225595584>, None)@139959700113344> (basis.py:284)
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 420, in _trigger
    self._process(event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 439, in _process
    self.machine.callbacks(self.machine.prepare_event, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1175, in callbacks
    self.callback(func, event_data)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/extensions/locking.py", line 201, in _locked_method
    return func(*args, **kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/transitions/core.py", line 1194, in callback
    func(event_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 254, in prepare_event
    self._action_result[action] = self._call_action(action)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/state_machine/basis.py", line 267, in _call_action
    return self._action_calls[action]()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 199, in update
    data = self.capture()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 166, in capture
    data = self._demonstrator.capture_observation()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 508, in capture_observation
    self._fully_process(add_data)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 536, in _fully_process
    func(group, component, comp_name, "capture", wait)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 495, in add_data
    return func(0.0)
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/common/systems/wrappers.py", line 111, in capture_observation
    raise TimeoutError("Timeout waiting for observation.")
TimeoutError: Timeout waiting for observation.
[WARNING] 2026-01-10 17:55:42,985 transitions.DemonstrateFSM: Action failed: update (basis.py:244)
[WARNING] 2026-01-10 17:55:42,985 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
^C[INFO] 2026-01-10 17:55:43,015 airbot-data-collection: Keyboard interrupt received. Exiting... (main.py:94)
[INFO] 2026-01-10 17:55:43,015 airbot-data-collection: Shutting down: self_manager. (main.py:97)
[INFO] 2026-01-10 17:55:43,016 airbot-data-collection: Shutting down: keyboard. (main.py:97)
[INFO] 2026-01-10 17:55:43,016 airbot-data-collection: Summary:
{'Average update freq': '206.1050 Hz',
 'Average update time': '0.0049 s',
 'Total time taken': '31.8397 s'} (main.py:110)
[INFO] 2026-01-10 17:55:43,016 airbot-data-collection: Done. (main.py:111)
^CException ignored in: <module 'threading' from '/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/threading.py'>
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/threading.py", line 1567, in _shutdown
    lock.acquire()
KeyboardInterrupt: 
^CException ignored in atexit callback: <function _exit_function at 0x7f4b4eafc940>
Traceback (most recent call last):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/util.py", line 334, in _exit_function
    _run_finalizers(0)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/util.py", line 300, in _run_finalizers
    finalizer()
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/util.py", line 224, in __call__
    res = self._callback(*self._args, **self._kwargs)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/managers.py", line 674, in _finalize_manager
    process.join(timeout=1.0)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/process.py", line 149, in join
    res = self._popen.wait(timeout)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/popen_fork.py", line 40, in wait
    if not wait([self.sentinel], timeout):
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/multiprocessing/connection.py", line 931, in wait
    ready = selector.select(timeout)
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/selectors.py", line 416, in select
    fd_event_list = self._selector.poll(timeout)
KeyboardInterrupt: 
Round 1:  43%|██████████████████████████████████████████████████▎                                                                 | 434/1000 [00:15<00:20, 28.03step/s, Percentage=43.4%]