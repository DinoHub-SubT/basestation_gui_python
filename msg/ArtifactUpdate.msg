#Message for updating artifact info

string unique_id
uint8 update_type

uint8 PROPERTY_CATEGORY = 0
uint8 PROPERTY_PRIORITY = 1
uint8 PROPERTY_POSE_X = 2
uint8 PROPERTY_POSE_Y = 3
uint8 PROPERTY_POSE_Z = 4

string category
geometry_msgs/Pose curr_pose
string priority