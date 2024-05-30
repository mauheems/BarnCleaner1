std_msgs/Header header

vision_msgs/BoundingBox2D[] bboxes
uint16[] classes        # 0:feces, 1:cow
float32[] detection_score
sensor_msgs/Image source_img
sensor_msgs/Image depth_img
