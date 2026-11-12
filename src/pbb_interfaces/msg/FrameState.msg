# CARLA simulation time. Vehicle coordinates: x forward, y right, z up.
std_msgs/Header header
string session_id
uint64 frame
# World pose in CARLA coordinates (x/y metres, yaw radians; positive turns right).
float64 pose_x_m
float64 pose_y_m
float64 yaw_rad
float64 speed_mps
float64 dt
float64 max_steer_rad
