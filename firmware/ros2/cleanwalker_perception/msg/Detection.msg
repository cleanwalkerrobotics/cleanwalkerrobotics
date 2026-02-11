# Detection.msg — Single litter detection from YOLO26 pipeline
# Published as part of DetectionArray or individually

std_msgs/Header header

# Bounding box in image coordinates (pixels)
float32 bbox_x       # Top-left x
float32 bbox_y       # Top-left y
float32 bbox_width   # Width in pixels
float32 bbox_height  # Height in pixels

# Detection class
string class_name    # e.g. "plastic_bottle", "cigarette_butt", "paper"
int32 class_id       # Numeric class ID (0-9)
float32 confidence   # Detection confidence [0.0, 1.0]

# 3D position in camera frame (populated when depth is available)
geometry_msgs/Point position  # (x, y, z) in meters, camera optical frame
bool has_depth                # True if depth was successfully estimated
