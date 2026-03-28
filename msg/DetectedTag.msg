# Individual ArUco tag with tracked pose and cluster information
uint32 tag_id
geometry_msgs/Pose tag_pose
int32 cluster_id        # -1 if not part of any cluster
geometry_msgs/Pose cluster_pose
float32 confidence      # Tracking quality [0.0, 1.0]
