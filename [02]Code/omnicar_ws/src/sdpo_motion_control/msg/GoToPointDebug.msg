std_msgs/Header header

bool active
bool goal_reached

string pose_source
string control_frame_id
string current_frame_id
string goal_frame_id
string stop_reason

float64 current_x
float64 current_y
float64 current_yaw

float64 target_x
float64 target_y
float64 target_yaw

float64 error_x_world
float64 error_y_world
float64 error_x_body
float64 error_y_body
float64 error_yaw
float64 distance_error

float64 integral_x
float64 integral_y
float64 integral_yaw

float64 derivative_x
float64 derivative_y
float64 derivative_yaw

float64 cmd_vx_raw
float64 cmd_vy_raw
float64 cmd_w_raw

float64 cmd_vx
float64 cmd_vy
float64 cmd_w
float64 linear_cmd_norm

bool saturated_vx
bool saturated_vy
bool saturated_w
