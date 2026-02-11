# LitterItem.msg — Tracked litter item with world-frame pose
# Combines detection + depth + SLAM localization into a graspable target

std_msgs/Header header

# Unique tracking ID (persistent across frames)
uint32 track_id

# Classification
string class_name    # e.g. "plastic_bottle", "can", "paper"
int32 class_id       # Numeric class ID (0-9)
float32 confidence   # Latest detection confidence [0.0, 1.0]

# World-frame pose (from base_link or map frame)
geometry_msgs/PoseStamped pose

# Estimated physical size (meters)
float32 estimated_width
float32 estimated_height

# Grasp recommendation
string recommended_grasp  # "top_down", "pinch", "scoop", "edge"

# Tracking metadata
uint32 detection_count    # Number of frames this item was detected
float32 first_seen        # Timestamp of first detection (seconds)
bool is_reachable         # True if within arm workspace
