# message type
uint8 MESSAGE_TYPE_ESTOP=1
uint8 MESSAGE_TYPE_DEFINE_WAYPOINT=2
uint8 MESSAGE_TYPE_ARTIFACT_REPORT=3
uint8 MESSAGE_TYPE_RETURN_HOME=4
uint8 MESSAGE_TYPE_RESEND_ALL_ARTIFACTS=5
uint8 MESSAGE_TYPE_DROP_COMMS=6
uint8 MESSAGE_TYPE_LANDING_BEHAVIOR=7
uint8 MESSAGE_TYPE_MAX_FLIGHT_TIME=8
uint8 message_type

# data
uint8 recipient_robot_id
string data

# fields relevant to MESSAGE_TYPE_ARTIFACT_REPORT 
# Artifact identifiers
time artifact_stamp
uint8 artifact_robot_id
uint16 artifact_report_id

# Artifact data
int8 ARTIFACT_UNKNOWN=0 # Know that something exists with unknown type
int8 ARTIFACT_REMOVE=-1 # Positive IDs will be used for actual categories
int8 artifact_type
float32 artifact_x
float32 artifact_y
float32 artifact_z

float32 bluetooth_strength
float32 audio_strength

#field relevant to odom message
uint32 timestamp
float32 robot_x
float32 robot_y
float32 robot_z
float32 robot_roll
float32 robot_pitch
float32 robot_yaw

# constants relevant to MESSAGE_TYPE_ESTOP
string ESTOP_RESUME=0
string ESTOP_PAUSE=1
string ESTOP_SOFT=2
string ESTOP_HARD=3

#constants relevant to land in comms
string LAND_IN_COMMS = 4
string LAND_AT_HOME = 5
