# 0 is resume, 1 is hard e-stop, 2 is position cmd
uint8 message_type

uint8 recipient_robot_id

#int16 x
#int16 y
#int16 z

uint8 ESTOP_RESUME=0
uint8 ESTOP_HARD=1
uint8 ESTOP_POSITION=2
