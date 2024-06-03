# Object Detector

Detect faeces in images and provides detections as a list of bounding box images.

Subscribes to RGB and Depth image topics published in the Mirte and publishes Detections (custom_msgs package) which contain the source images and list of bounding box detections


## Run instructions

```bash
rosrun object_detector run_object_detector
rosrun object_detector custom_msg_util
```

run_object_detector: Runs the model on subscribed RGB image
custom_msg_util: Publishes annotated RGB image and Depth image to view detections on Rviz.