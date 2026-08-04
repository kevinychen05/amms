# A single detected object with its estimated 3D pose.

string class_label
# Object category label, e.g. "red_mug". Matched against MissionCommand.object_class.

geometry_msgs/PoseStamped pose
# Estimated object pose. pose.header.frame_id is "map"; pose.header.stamp is the
# timestamp of the source RGB-D image the detection came from (not detection wall-clock
# time), so downstream TF lookups use the correct historical transform.

float32 confidence
# Detector confidence score, range [0.0, 1.0].

float32[3] bounding_box_size
# Approximate object extent (x, y, z) in meters, in the object's own local frame.
# Used by grasp_planner_node to generate candidate grasp poses.
