std_msgs/Header header

int32 movement_id
string movement_type   # 예: 'left_turn', 'straight', ...
string movement_status # 예: 'allowed', 'not_allowed'
uint8 signalGroup
string movementName
v2x_msgs/MovementEvent[] state_time_speed
