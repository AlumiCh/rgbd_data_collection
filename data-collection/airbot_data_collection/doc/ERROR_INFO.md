(airbot_data) jojo@jojo-System-Product-Name:~/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection$ python main.py --path defaults/config_single_arm_dual_rgbd.yaml \--dataset.directory="data/test_20260107" \--sample-limit.size=1
[INFO] 2026-01-07 17:07:43,563 airbot-data-collection: Preparing cv2.imshow (opencv.py:28)
[INFO] 2026-01-07 17:07:43,563 airbot-data-collection: Showing Prepared image (opencv.py:32)
[INFO] 2026-01-07 17:07:43,625 airbot-data-collection: Prepared image is ready (opencv.py:35)
[INFO] 2026-01-07 17:07:43,625 airbot-data-collection: cv2.imshow is ready (opencv.py:40)
[INFO] 2026-01-07 17:07:43,659 airbot-data-collection: Version: 0.6.3 (main.py:38)
[WARNING] 2026-01-07 17:07:43,716 airbot_data_collection.airbot.samplers.mcap_sampler: It is detected that the `UPLOAD` package is not installed, and the cloud upload function will not be available. If you need to use the upload function, please contact us to install it. (mcap_sampler.py:30)
[INFO] 2026-01-07 17:07:43,787 KeyboardCallbackManager:  
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
[INFO] 2026-01-07 17:07:43,787 airbot-data-collection: Update rate: 30.0 Hz (main.py:56)
[INFO] 2026-01-07 17:07:43,788 SelfManager: Configuring the demonstrate interface. (basis.py:91)
[ERROR] 2026-01-07 17:07:44,363 transitions.DemonstrateFSM: <EventData(<LockedEvent('t_configure')@140038654974976>, <State('unconfigured')@140038654971904>, None)@140037787632736> (basis.py:284)
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
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/interface.py", line 82, in configure
    if self._demonstrator.configure():
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/basis.py", line 113, in configure
    self._configured = self.on_configure()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 406, in on_configure
    return self._init_all_components()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 410, in _init_all_components
    self._cg_manager.instance_groups()
  File "/home/jojo/airbot/airbot-data-5.1.6.8a3/data-collection/airbot_data_collection/demonstrate/demonstrators/grouped.py", line 272, in instance_groups
    DemonstrateGroup(
  File "/home/jojo/anaconda3/envs/airbot_data/lib/python3.10/site-packages/pydantic/main.py", line 250, in __init__
    validated_self = self.__pydantic_validator__.validate_python(data, self_instance=self)
pydantic_core._pydantic_core.ValidationError: 4 validation errors for DemonstrateGroup
others.0.is-instance[System]
  Input should be an instance of System [type=is_instance_of, input_value={'camera_index': 25012207...ue, 'align_depth': True}, input_type=DictConfig]
    For further information visit https://errors.pydantic.dev/2.12/v/is_instance_of
others.0.is-instance[Sensor]
  Input should be an instance of Sensor [type=is_instance_of, input_value={'camera_index': 25012207...ue, 'align_depth': True}, input_type=DictConfig]
    For further information visit https://errors.pydantic.dev/2.12/v/is_instance_of
others.1.is-instance[System]
  Input should be an instance of System [type=is_instance_of, input_value={'camera_index': 24352207...ue, 'align_depth': True}, input_type=DictConfig]
    For further information visit https://errors.pydantic.dev/2.12/v/is_instance_of
others.1.is-instance[Sensor]
  Input should be an instance of Sensor [type=is_instance_of, input_value={'camera_index': 24352207...ue, 'align_depth': True}, input_type=DictConfig]
    For further information visit https://errors.pydantic.dev/2.12/v/is_instance_of
[WARNING] 2026-01-07 17:07:44,364 transitions.DemonstrateFSM: Action failed: configure (basis.py:244)
[INFO] 2026-01-07 17:07:44,364 SelfManager: Failed to configure the demonstrate interface. (basis.py:96)
[WARNING] 2026-01-07 17:07:44,364 airbot-data-collection: Failed to update manager: self_manager. (main.py:69)
[WARNING] 2026-01-07 17:07:44,364 airbot-data-collection: Update took too long: exceed 0.5433250046548589 s. (main.py:92)
[INFO] 2026-01-07 17:07:55,378 KeyboardCallbackManager: Executing action: sample (keyboard.py:132)
[WARNING] 2026-01-07 17:07:55,378 transitions.core: Can't trigger event t_sample from state unconfigured! (core.py:454)
[WARNING] 2026-01-07 17:07:55,379 transitions.DemonstrateFSM: Action failed: sample (basis.py:244)
 ^C[INFO] 2026-01-07 17:07:58,947 airbot-data-collection: Keyboard interrupt received. Exiting... (main.py:94)
[INFO] 2026-01-07 17:07:58,948 airbot-data-collection: Shutting down: self_manager. (main.py:97)
[INFO] 2026-01-07 17:07:58,948 airbot-data-collection: Shutting down: keyboard. (main.py:97)
[INFO] 2026-01-07 17:07:58,948 airbot-data-collection: Summary:
{'Average update freq': '19127.8462 Hz',
 'Average update time': '0.0001 s',
 'Total time taken': '15.1604 s'} (main.py:110)
[INFO] 2026-01-07 17:07:58,948 airbot-data-collection: Done. (main.py:111)
