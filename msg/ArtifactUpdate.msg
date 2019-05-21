#Message for updating artifact info

string unique_id
uint8 update_type

uint8 PROPERTY_CATEGORY = 0
uint8 PROPERTY_POSE = 1
uint8 PROPERTY_PRIORITY = 2

string category
geometry_msgs/Pose curr_pose
string priority