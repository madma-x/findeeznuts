# Individual ArUco tag with tracked pose and alignment information
uint32 tag_id
geometry_msgs/Pose tag_pose
int32 slot_id           # theoretical slot index (0-3), -1 if unmatched
geometry_msgs/Pose alignment_pose
float32 confidence      # Tracking quality [0.0, 1.0]
