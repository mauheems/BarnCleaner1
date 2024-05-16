std_msgs/Header header

vision_msgs/BoundingBox2D[] bboxes
uint16[] classes
float32[] detection_score
sensor_msgs/Image source_img
sensor_msgs/Image depth_img

float32 min_range       # minimum range value [m]
float32 max_range       # maximum range value [m]
float32 range_lf
float32 range_lr
float32 range_rf
float32 range_rr
