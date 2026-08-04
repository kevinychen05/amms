# All objects detected in a single perception cycle (one message per processed frame).

std_msgs/Header header
# header.stamp matches the source image's stamp; header.frame_id is "map".

DetectedObject[] objects
