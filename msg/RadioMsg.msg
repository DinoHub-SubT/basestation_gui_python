# message type
uint8 MESSAGE_TYPE_ESTOP=1
uint8 MESSAGE_TYPE_DEFINE_WAYPOINT=2
uint8 message_type

# data
uint8 recipient_robot_id
string data

# constants relevant to ESTOP messages
string ESTOP_RESUME=0
string ESTOP_PAUSE=1
string ESTOP_SOFT=2
string ESTOP_HARD=3

