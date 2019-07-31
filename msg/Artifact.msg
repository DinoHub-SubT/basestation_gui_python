# Message for transporting artifact info throughout the gui
string category
geometry_msgs/Pose curr_pose
geometry_msgs/Pose orig_pose
uint8 source_robot_id
uint8 artifact_report_id
float32 time_from_robot
float32 time_to_darpa
bool unread
string priority
string darpa_response
sensor_msgs/Image[] imgs
time[] img_stamps
float32 original_timestamp
string unique_id
string robot_uuid
