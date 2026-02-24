#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "plan_manage/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64.h" // Add this include
#include "std_msgs/Int32.h"
#include "visualization_msgs/Marker.h"
#include "plan_manage/ilc_planner.h"
#include <ros/ros.h>
#include <chrono>
#include <limits>
#include <algorithm>

ros::Publisher pos_pub, traj_pub, vel_pub, vel_field_pub, drone_arrow_pub;
ros::Publisher speed_pub, thr_pub, real_speed_pub, real_thr_pub;

nav_msgs::Odometry odom;
Eigen::Vector3d last_real_vel_ = Eigen::Vector3d::Zero();
ros::Time last_odom_time_ = ros::Time(0);

// ILC planner 实例 (仅在 traj_server 中使用)
ilc_planner* ilcplan = nullptr;
bool _has_traj = false;
Eigen::Vector3d _start_pt;
Eigen::Vector3d _init_pos; // Add _init_pos
Eigen::Vector3d last_vel_cmd_;

// ILC 参数
double _vmin, _vmax, _goalth, _xth, _kp_vl, _kp_law, _kd_law, _kp_path, _kd_path, _tau, _amax;
int _iteration;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {5.7, 5.7, 6.2};
// double vel_gain[3] = {3.4, 3.4, 4.0};
double pos_gain[3] = { 5.7, 5.7, 6.2 };
double vel_gain[3] = { 3.4, 3.4, 4.0 };

// using NonUniformBspline = NonUniformBspline<Eigen::Vector3d>;

bool receive_traj_ = false;
vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
int fsm_state = -1;  // INIT:0 GO_START:1 WAIT_TRIGGER:2 EXEC_TRAJ:3 HOVER:4

// yaw control
double last_yaw_;
double time_forward_;

vector<Eigen::Vector3d> traj_cmd_, traj_real_;

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (size_t i = 0; i < path.size(); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

// 可视化速度向量场
void visualizeVelocityField(const std::vector<Eigen::Vector3d>& traj_points) {
  if (ilcplan == nullptr) return;
  
  visualization_msgs::Marker arrows;
  arrows.header.frame_id = "world";
  arrows.header.stamp = ros::Time::now();
  arrows.ns = "velocity_field";
  arrows.id = 0;
  arrows.type = visualization_msgs::Marker::LINE_LIST;
  arrows.action = visualization_msgs::Marker::ADD;
  arrows.pose.orientation.w = 1.0;
  
  // 箭头线宽
  arrows.scale.x = 0.02;
  
  // 颜色: 青色 (cyan)
  arrows.color.r = 0.0;
  arrows.color.g = 1.0;
  arrows.color.b = 1.0;
  arrows.color.a = 0.8;
  
  // 在轨迹附近采样点,构建速度向量场
  const double lateral_dist = 0.5;  // 轨迹两侧的距离
  const double vertical_dist = 0.3; // 上下的距离
  const int samples_per_segment = 3; // 每段轨迹采样的密度
  
  for (size_t i = 0; i < traj_points.size() - 1; i += 20) { // 每20个点采样一次,避免过密
    Eigen::Vector3d traj_pt = traj_points[i];
    
    // 计算轨迹切向
    Eigen::Vector3d tangent = (traj_points[std::min(i + 1, traj_points.size() - 1)] - traj_pt).normalized();
    
    // 计算轨迹的法向量和副法向量
    Eigen::Vector3d up(0, 0, 1);
    Eigen::Vector3d binormal = tangent.cross(up).normalized();
    if (binormal.norm() < 0.1) { // 如果轨迹接近垂直
      binormal = Eigen::Vector3d(1, 0, 0);
    }
    Eigen::Vector3d normal = binormal.cross(tangent).normalized();
    
    // 在轨迹周围采样
    for (int lat = -samples_per_segment; lat <= samples_per_segment; ++lat) {
      for (int vert = -samples_per_segment; vert <= samples_per_segment; ++vert) {
        // 跳过轨迹上的点
        if (lat == 0 && vert == 0) continue;
        
        // 计算采样点位置
        Eigen::Vector3d sample_pt = traj_pt + 
                                     binormal * (lat * lateral_dist / samples_per_segment) +
                                     normal * (vert * vertical_dist / samples_per_segment);
        
        // 调用 ILC 计算该点的速度
        Eigen::Vector3d cmd_vel;
        Eigen::Vector3d refer_pose;
        double yaw_des;
        ilcplan->computeVelocityCommands(cmd_vel, refer_pose, yaw_des, sample_pt);
        
        // 缩放速度向量以便可视化
        double scale = 0.15;  // 箭头长度缩放
        Eigen::Vector3d vel_scaled = cmd_vel * scale;
        
        // 添加箭头起点
        geometry_msgs::Point p_start, p_end;
        p_start.x = sample_pt.x();
        p_start.y = sample_pt.y();
        p_start.z = sample_pt.z();
        
        // 添加箭头终点
        p_end.x = sample_pt.x() + vel_scaled.x();
        p_end.y = sample_pt.y() + vel_scaled.y();
        p_end.z = sample_pt.z() + vel_scaled.z();
        
        arrows.points.push_back(p_start);
        arrows.points.push_back(p_end);
      }
    }
  }
  
  vel_field_pub.publish(arrows);
  ROS_INFO("[traj_server] Velocity field visualized with %zu arrows", arrows.points.size() / 2);
}

void bsplineCallback(plan_manage::BsplineConstPtr msg) {
  // parse pos traj
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (size_t i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(i, 0) = msg->yaw_pts[i];
  }

  NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
  _has_traj = true;
  _start_pt = pos_traj.evaluateDeBoorT(0.0); // 获取轨迹起点

  // ====== 设置 ILC Planner ======
  // 1. 从 B-spline 采样 1000 个点
    const int sample_num = 1000;
    std::vector<Eigen::Vector3d> sampled_points;
    sampled_points.reserve(sample_num);
    
    double total_duration = traj_duration_;
    for (int i = 0; i < sample_num; ++i) {
      double t = total_duration * i / (sample_num - 1.0);
      Eigen::Vector3d point = pos_traj.evaluateDeBoorT(t);
      sampled_points.push_back(point);
    }
    
    // 2. 设置 xth_vec based on bound 参数
    double bound = _xth; // 使用全局参数 ILParam/xth
    std::vector<double> xth_vec(sample_num, bound / 3.0); //取等宽跑道的1/3
    

    // 3. 设置 ILC 参数
    ilcplan->set_param(_vmin, _vmax, _goalth, _kp_vl, _kp_law, _kd_law, 
                       xth_vec, _kp_path, _kd_path, _tau, _iteration);
    

  // 3. 调用 setPlan 设置轨迹 (需要3个参数: path, robot_pose, robot_vel)
  Eigen::Vector3d robot_pose = _start_pt;  // 使用轨迹起点
  Eigen::Vector3d robot_vel = Eigen::Vector3d::Zero();  // 初始速度为0
  
  // 调用ILC核心算法
  ilcplan->setPlan(sampled_points, robot_pose, robot_vel);
  
  // 4. 可视化速度向量场
  //visualizeVelocityField(sampled_points);


}

void stateCallback(const std_msgs::Int32ConstPtr& msg) {
  // INIT:0 GO_START:1 WAIT_TRIGGER:2 EXEC_TRAJ:3 HOVER:4
  fsm_state = msg->data;

}

void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  odom = msg;

  /* Visualize real speed and thrust */
  Eigen::Vector3d current_vel(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
  ros::Time current_time = odom.header.stamp;

  if (last_odom_time_.toSec() == 0) {
    last_real_vel_ = current_vel;
    last_odom_time_ = current_time;
  } else {
    double dt = (current_time - last_odom_time_).toSec();
    if (dt > 1e-3) {
      Eigen::Vector3d accel = (current_vel - last_real_vel_) / dt;
      
      // Only publish visualization data when in EXEC_TRAJ state
      if (fsm_state == 3) {
        std_msgs::Float64 real_speed_msg;
        real_speed_msg.data = current_vel.norm();
        real_speed_pub.publish(real_speed_msg);

        // Total Thrust = Norm(Acc + g) assuming unit mass or just representing specific force
        std_msgs::Float64 real_thr_msg;
        real_thr_msg.data = (accel + Eigen::Vector3d(0, 0, 9.8)).norm();
        real_thr_pub.publish(real_thr_msg);
      }

      last_real_vel_ = current_vel;
      last_odom_time_ = current_time;
    }
  }

  traj_real_.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);

  // Visualize drone orientation arrow
  if (true) { // Always visualize if odom is available
    Eigen::Vector3d pos(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    
    Eigen::Quaterniond q(odom.pose.pose.orientation.w,
                          odom.pose.pose.orientation.x, 
                          odom.pose.pose.orientation.y, 
                          odom.pose.pose.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();
    
    // Body x-axis in world frame (forward direction)
    Eigen::Vector3d forward_dir = R.col(0); 

    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "drone_arrow";
    arrow.id = 0;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    
    arrow.points.resize(2);
    arrow.points[0].x = pos(0);
    arrow.points[0].y = pos(1);
    arrow.points[0].z = pos(2);
    
    // Arrow length 1.0m
    arrow.points[1].x = pos(0) + forward_dir(0) * 1.0;
    arrow.points[1].y = pos(1) + forward_dir(1) * 1.0;
    arrow.points[1].z = pos(2) + forward_dir(2) * 1.0;
    
    arrow.scale.x = 0.1; // Shaft diameter
    arrow.scale.y = 0.2; // Head diameter
    arrow.scale.z = 0.2; // Head length
    
    arrow.color.r = 1.0;
    arrow.color.g = 0.0;
    arrow.color.b = 0.0; // Red arrow
    arrow.color.a = 1.0;
    
    drone_arrow_pub.publish(arrow);
  }
}

void visCallback(const ros::TimerEvent& e) {

  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}

/*ILC专用*/
void velCallback(const ros::TimerEvent& e)
{   
    if (!receive_traj_) return; // 如果还没有接收到轨迹数据，则直接返回

    // 检查 ilcplan 是否已初始化
    if (ilcplan == nullptr) {
        ROS_WARN_THROTTLE(5.0, "[traj_server] ilcplan not initialized yet");
        return;
    }

    if (_has_traj)
    {
        // 使用当前里程计位置作为机器人位置
        Eigen::Vector3d robot(odom.pose.pose.position.x, 
                             odom.pose.pose.position.y, 
                             odom.pose.pose.position.z);
        
        Eigen::Vector3d cmd_vel;
        Eigen::Vector3d refer_pose;
        double yaw_des;
        ilcplan->computeVelocityCommands(cmd_vel, refer_pose, yaw_des, robot);


        /**********打包指令信息*************/
        // 位置设为0
        cmd.position.x = refer_pose[0];
        cmd.position.y = refer_pose[1];
        cmd.position.z = refer_pose[2];
        // 填充速度信息
        cmd.velocity.x = cmd_vel[0];
        cmd.velocity.y = cmd_vel[1];
        cmd.velocity.z = cmd_vel[2];
        // 加速度设为0
        double dt = 0.03;
        Eigen::Vector3d acc = (cmd_vel - last_vel_cmd_) / dt;
        
        acc = acc.normalized() * std::min(acc.norm(), _amax);

        cmd.acceleration.x = acc[0];
        cmd.acceleration.y = acc[1];
        cmd.acceleration.z = acc[2];
        // 填充偏航角信息
        cmd.yaw        = yaw_des;
        cmd.yaw_dot    = (yaw_des - last_yaw_) / dt;
        // 增益设为1
        cmd.kx[0] = 2.5; cmd.kx[1] = 2.5; cmd.kx[2] = 1.5;
        cmd.kv[0] = 3.0; cmd.kv[1] = 3.0; cmd.kv[2] = 2.0;

        last_vel_cmd_ = cmd_vel;
        last_yaw_  = yaw_des;

        vel_pub.publish(cmd);

        /* visualize aircraft speed and total thrust */
        // Only publish visualization data when in EXEC_TRAJ state
        if (fsm_state == 3) {
            std_msgs::Float64 speed_msg, thr_msg;
            speed_msg.data = cmd_vel.norm();
            thr_msg.data = (acc + Eigen::Vector3d(0.0, 0.0, 9.8)).norm();
            speed_pub.publish(speed_msg);
            thr_pub.publish(thr_msg);
        }
    }
    else
    {
        // 所有字段设为0
        cmd.position.x = 0.0;
        cmd.position.y = 0.0;
        cmd.position.z = 0.0;
        cmd.velocity.x = 0.0;
        cmd.velocity.y = 0.0;
        cmd.velocity.z = 0.0;
        cmd.acceleration.x = 0.0;
        cmd.acceleration.y = 0.0;
        cmd.acceleration.z = 0.0;
        cmd.yaw        = 0.0;
        cmd.yaw_dot    = 0.0;
        cmd.kx[0] = 0.0; cmd.kx[1] = 0.0; cmd.kx[2] = 0.0;
        cmd.kv[0] = 0.0; cmd.kv[1] = 0.0; cmd.kv[2] = 0.0;

        vel_pub.publish(cmd);
    }
}

void posCallback(const ros::TimerEvent& e)
{

  /********EXEC_TRAJ*******/
    if(fsm_state == 3)  return; // EXEC_TRAJ 不发布位置指令,而是发布速度指令
    static int last_state = -1;
    static Eigen::Vector3d last_setpoint;

  /********GO_START*******/
    if(fsm_state == 1){         // GO_START 控制飞行器飞行到指定起点init_pos
        // Simple smoothing controller
        Eigen::Vector3d cur_pos(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
        Eigen::Vector3d dest = _init_pos;
        
        // Reset setpoint if we just entered this state or drifted too far (safety)
        if (last_state != 1 || (last_setpoint - cur_pos).norm() > 1.0) {
            last_setpoint = cur_pos;
            ROS_INFO("[traj_server] Entering GO_START. Current Pos: [%f, %f, %f], Goal: [%f, %f, %f]", 
                cur_pos(0), cur_pos(1), cur_pos(2), dest(0), dest(1), dest(2));
        }

        Eigen::Vector3d dir = dest - last_setpoint;
        
        // Decouple XY and Z control
        Eigen::Vector3d dir_xy = dir; dir_xy(2) = 0.0;
        double dist_xy = dir_xy.norm();
        double dist_z = std::abs(dir(2));
        
        double v_max_xy = 3.0; // m/s
        double v_max_z = 1.5;  // m/s
        double dt = 0.02;      // 50Hz

        Eigen::Vector3d des_vel = Eigen::Vector3d::Zero();

        if (dist_xy > v_max_xy * dt) {
            des_vel.head(2) = dir_xy.head(2).normalized() * v_max_xy;
        } else {
            des_vel.head(2) = dir_xy.head(2) / dt;
        }

        if (dist_z > v_max_z * dt) {
            des_vel(2) = (dir(2) > 0 ? 1.0 : -1.0) * v_max_z;
        } else {
            des_vel(2) = dir(2) / dt;
        }

        last_setpoint += des_vel * dt;
        
        if ((dest - last_setpoint).norm() < 0.01) {
            last_setpoint = dest;
            des_vel = Eigen::Vector3d::Zero();
        }

        cmd.position.x = last_setpoint(0);
        cmd.position.y = last_setpoint(1);
        cmd.position.z = last_setpoint(2);
        
        cmd.velocity.x = des_vel(0);
        cmd.velocity.y = des_vel(1);
        cmd.velocity.z = des_vel(2);
        
        cmd.acceleration.x = 0.0;
        cmd.acceleration.y = 0.0;
        cmd.acceleration.z = 0.0;

        // Yaw alignment with trajectory start tangent
        double desired_yaw = 0.0;
        
        // If we have a trajectory, align with its start tangent
        if (receive_traj_ && !traj_.empty() && traj_.size() > 0) {
             // Calculate tangent direction geometrically 
             double t_step = 0.05;              // Use a slightly larger step to ensure we get a valid direction
             if (traj_[0].getTimeSum() < t_step) t_step = traj_[0].getTimeSum() / 2.0;

             Eigen::Vector3d p0 = traj_[0].evaluateDeBoorT(0.0);
             Eigen::Vector3d p1 = traj_[0].evaluateDeBoorT(t_step); 
             Eigen::Vector3d tangent = p1 - p0;
             
             if (tangent.norm() > 1e-3) {
                 desired_yaw = atan2(tangent(1), tangent(0));
             }
        } 
        // If no trajectory received yet, align with the direction to the goal if moving
        else if (des_vel.norm() > 0.1) {
            desired_yaw = atan2(des_vel(1), des_vel(0));
        }
        
        cmd.yaw = desired_yaw;
        cmd.yaw_dot = 0.03;

        cmd.kx[0] = pos_gain[0]; cmd.kx[1] = pos_gain[1]; cmd.kx[2] = pos_gain[2];
        cmd.kv[0] = vel_gain[0]; cmd.kv[1] = vel_gain[1]; cmd.kv[2] = vel_gain[2];
    }
    /**************INIT | HOVER | WAIT_TRIGGER*************/
    else {  // INIT HOVER
        if (fsm_state == 0 || fsm_state == 2) {
            last_setpoint = _init_pos;
        } 
        else if (last_state != fsm_state) {  
                // For other transitions (e.g. entering HOVER from EXEC), latch current position
                last_setpoint = Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
                ROS_INFO("[traj_server] Hovering at [%f, %f, %f]", last_setpoint(0), last_setpoint(1), last_setpoint(2));
        }
        cmd.velocity.x = 0.0;
        cmd.velocity.y = 0.0;
        cmd.velocity.z = 0.0;
        cmd.acceleration.x = 0.0;
        cmd.acceleration.y = 0.0;
        cmd.acceleration.z = 0.0;
    }
    // Update last_state
    last_state = fsm_state;
    pos_pub.publish(cmd);
}

void cmdCallback(const ros::TimerEvent& e)
{
    if (fsm_state == 3)  velCallback(e);
    else posCallback(e);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  /* get start pos */
  std::vector<double> init_pos_vec;
  if(node.getParam("init_pos", init_pos_vec))
  {
      _init_pos(0) = init_pos_vec[0];
      _init_pos(1) = init_pos_vec[1];
      _init_pos(2) = init_pos_vec[2];
      ROS_INFO("[traj_server] init_pos loaded: [%f, %f, %f]", _init_pos(0), _init_pos(1), _init_pos(2));
  }
  else
  {
      ROS_WARN("[traj_server] init_pos not found in parameter server, using (0,0,0)");
      _init_pos = Eigen::Vector3d::Zero();
  }

  /*************************************** START IL-Planner相关变量 ***************************************/
  // 从参数服务器加载 ILC 参数
  nh.param("ILParam/vmax", _vmax, 2.0);
  nh.param("ILParam/vmin", _vmin, 1.0);
  nh.param("ILParam/goalth", _goalth, 1.0);
  nh.param("ILParam/xth", _xth, 0.5);
  nh.param("ILParam/kp_law", _kp_law, 1.0);
  nh.param("ILParam/kp_vl", _kp_vl, 0.4);
  nh.param("ILParam/kd_law", _kd_law, 0.6);
  nh.param("ILParam/kp_path", _kp_path, 2.9);
  nh.param("ILParam/kd_path", _kd_path, 2.5);
  nh.param("ILParam/iteration", _iteration, 10);
  nh.param("ILParam/dynamic", _tau, 2.5);
  nh.param("ILParam/amax", _amax, 6.0);


  // 初始化 ILC planner (仅在 traj_server 中创建和使用)
  ilcplan = new ilc_planner();
  ROS_INFO("[traj_server] ILC planner initialized");
  /*************************************** END IL-Planner相关变量 ***************************************/

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber state_sub = node.subscribe("planning/state", 10, stateCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);

  pos_pub = node.advertise<quadrotor_msgs::PositionCommand>("planning/position_cmd", 50);
  vel_pub = node.advertise<quadrotor_msgs::PositionCommand>("planning/velocity_cmd", 50);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);
  vel_field_pub = node.advertise<visualization_msgs::Marker>("planning/velocity_field", 10);
  drone_arrow_pub = node.advertise<visualization_msgs::Marker>("planning/drone_arrow", 10);

  /* data visualization */
  speed_pub = node.advertise<std_msgs::Float64>("visualizer/speed_cmd", 1000);
  thr_pub = node.advertise<std_msgs::Float64>("visualizer/total_thrust_cmd", 1000);
  real_speed_pub = node.advertise<std_msgs::Float64>("visualizer/real_speed", 1000);
  real_thr_pub = node.advertise<std_msgs::Float64>("visualizer/real_total_thrust", 1000);

  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);
  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.02), cmdCallback); // Changed vel_call to cmd_timer

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;

  ros::Duration(1.0).sleep();

  // Clear RViz display
  visualization_msgs::Marker delete_all;
  delete_all.action = 3; // DELETEALL
  delete_all.id = 0;
  traj_pub.publish(delete_all);
  vel_field_pub.publish(delete_all);

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}