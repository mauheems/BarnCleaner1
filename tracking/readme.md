# Tracking

This package receives object detection and self localization message, and gives the tracks of detected objects in BEV.

## Run

```bash
rosrun tracking object_localization.py
```

published topic

```
/tracker/feces_locations
/tracker/feces_markers
```

with type `ObjectLocationArray` from `custom_msgs`, and `MarkerArray` from `visualization_msgs`

## Test with rosbag

Run the following lines in separate terminals

```bash
roscore
rosbag play <path_to_rosbag>
rosrun tracking dummy_publisher.py
```

