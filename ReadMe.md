## Object Detector

### For running on the robot
Run the following two commands:
```shell
rosrun object_detector custom_msg_util
rosrun object_detector run_object_detector
```

Topics
```
/object_detector/detections

```

### Run yolo only

commands to run in separate terminals on the robot
```shell
rostopic hz -w 60 /detection_yolo/image /camera/color/image_raw

rosrun object_detector tf_lite_ros.py
```
To visualize the results
```shell
roslaunch manage_rosbag test_yolo.launch
```

transfer to laptop and see results using rviz
