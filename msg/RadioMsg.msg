# message type
uint8 MESSAGE_TYPE_ESTOP=1
uint8 MESSAGE_TYPE_DEFINE_WAYPOINT=2
uint8 MESSAGE_TYPE_ARTIFACT_REPORT=3
uint8 MESSAGE_TYPE_RETURN_HOME=4
uint8 ROBOT_ODOM=5
uint8 message_type

# data
uint8 recipient_robot_id
string data

# fields relevant to MESSAGE_TYPE_ARTIFACT_REPORT 
float32 artifact_x
float32 artifact_y
float32 artifact_z
string artifact_type
uint8 artifact_robot_id
uint16 artifact_report_id

# constants relevant to MESSAGE_TYPE_ESTOP
string ESTOP_RESUME=0
string ESTOP_PAUSE=1
string ESTOP_SOFT=2
string ESTOP_HARD=3