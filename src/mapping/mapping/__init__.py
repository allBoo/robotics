INITIAL_POINT = (0.0, -3.5)

CONTROLLER_TOPIC = '/mapping/move'
CMD_VEL_TOPIC = '/cmd_vel'
SCAN_TOPIC = '/scan'

DISTANCE_TOPIC = '/walker/distance'
ANGLE_TOPIC = '/walker/angle'

DISTANCE_ERROR_TOPIC = '/walker/distance/error'
DISTANCE_CORRECTION_TOPIC = '/walker/distance/correction'
DISTANCE_CORRECTION_P_TOPIC = '/walker/distance/correction_p'
DISTANCE_CORRECTION_D_TOPIC = '/walker/distance/correction_d'
DISTANCE_CORRECTION_I_TOPIC = '/walker/distance/correction_i'

ANGLE_ERROR_TOPIC = '/walker/angle/error'
ANGLE_CORRECTION_TOPIC = '/walker/angle/correction'
ANGLE_CORRECTION_P_TOPIC = '/walker/angle/correction_p'
ANGLE_CORRECTION_D_TOPIC = '/walker/angle/correction_d'
ANGLE_CORRECTION_I_TOPIC = '/walker/angle/correction_i'

STATE_IDLE = 'idle'
STATE_SPIN = 'spin'
STATE_FOLLOW = 'follow'
STATE_NEARBY = 'nearby'
STATE_RESET = 'reset'
