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

# Additional data to include
sensor_msgs/Image[] imgs # Entire set of images corresponding to detection
