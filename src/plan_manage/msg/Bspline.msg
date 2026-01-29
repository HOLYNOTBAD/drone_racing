int32 order
int64 traj_id
time start_time

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt

# Corridor radius from tube RRT* for ILC planner
geometry_msgs/Point[] corridor_pts    # tube RRT* 路径点
float64[] corridor_radius             # 对应每个路径点的安全半径

