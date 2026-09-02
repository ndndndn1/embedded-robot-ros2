# The calibration-target detector publishes one independent target-to-camera observation.
# header.frame_id and target_from_camera.header.frame_id are the target frame.
std_msgs/Header header
string camera_name
string target_id
string camera_serial
string stream_profile
geometry_msgs/TransformStamped target_from_camera
float32 reprojection_error_px
