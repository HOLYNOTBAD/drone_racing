/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "plan_manage/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include "plan_manage/ilc_planner.h"
#include <ros/ros.h>
#include <chrono>
#include <limits>
#include <algorithm>

ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub, vel_pub, vel_field_pub;

nav_msgs::Odometry odom;

// ILC planner 实例 (仅在 traj_server 中使用)
ilc_planner* ilcplan = nullptr;
bool _has_traj = false;
Eigen::Vector3d _start_pt;
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

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
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

  // parse yaw traj

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
  if (ilcplan != nullptr) {
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
    
    // 2. 解析 corridor 信息并为每个采样点计算对应的 xth
    std::vector<Eigen::Vector3d> corridor_pts;
    std::vector<double> corridor_radius;
    
    // 从消息中提取 corridor 点和半径
    for (size_t i = 0; i < msg->corridor_pts.size(); ++i) {
      Eigen::Vector3d pt(msg->corridor_pts[i].x, msg->corridor_pts[i].y, msg->corridor_pts[i].z);
      corridor_pts.push_back(pt);
    }
    for (size_t i = 0; i < msg->corridor_radius.size(); ++i) {
      corridor_radius.push_back(msg->corridor_radius[i]);
    }
    
    // 为每个采样点计算 xth (基于最近的 corridor 点的半径)
    std::vector<double> xth_vec(sample_num, _xth);  // 默认值
    
    if (!corridor_pts.empty() && !corridor_radius.empty()) {
      for (int i = 0; i < sample_num; ++i) {
        const Eigen::Vector3d& sample_pt = sampled_points[i];
        
        // 找到最近的 corridor 点
        double min_dist = std::numeric_limits<double>::max();
        int closest_idx = 0;
        for (size_t j = 0; j < corridor_pts.size(); ++j) {
          double dist = (sample_pt - corridor_pts[j]).norm();
          if (dist < min_dist) {
            min_dist = dist;
            closest_idx = j;
          }
        }
        
        // 使用对应 corridor 点的半径来计算 xth
        // xth 与半径成反比: 半径越大（越安全），xth 越小，允许更高速度
        // xth 与半径成正比的公式: xth = _xth * (max_radius / radius)
        // 这里使用简单的反比关系
        double radius = corridor_radius[closest_idx];
        double max_radius = *std::max_element(corridor_radius.begin(), corridor_radius.end());
        
        // 归一化的 xth: 当 radius 最大时 xth 最小，当 radius 最小时 xth 最大
        if (radius > 0.1) {  // 防止除零
          xth_vec[i] = radius / 1.5;
        } else {
          xth_vec[i] = _xth;  // 非常小的半径，使用较大的 xth（更保守）
        }
      }
      ROS_INFO("[traj_server] xth_vec computed from corridor radius, size: %zu", xth_vec.size());
    } else {
      ROS_WARN("[traj_server] No corridor info, using default xth for all points");
    }
    
    // 3. 设置 ILC 参数
    ilcplan->set_param(_vmin, _vmax, _goalth, _kp_vl, _kp_law, _kd_law, 
                       xth_vec, _kp_path, _kd_path, _tau, _iteration);
    


  // 3. 调用 setPlan 设置轨迹 (需要3个参数: path, robot_pose, robot_vel)
  Eigen::Vector3d robot_pose = _start_pt;  // 使用轨迹起点
  Eigen::Vector3d robot_vel = Eigen::Vector3d::Zero();  // 初始速度为0

  // 记录这几行执行时间
  auto __tp_start = std::chrono::high_resolution_clock::now();
  
  // 调用ILC核心算法
  ilcplan->setPlan(sampled_points, robot_pose, robot_vel);

  auto __tp_end = std::chrono::high_resolution_clock::now();
  double __tp_ms = std::chrono::duration_cast<std::chrono::microseconds>(__tp_end - __tp_start).count() / 1000.0;
  ROS_INFO("[traj_server] ilcplan->setPlan execution time: %.3f ms", __tp_ms);
    
  ROS_INFO("[traj_server] ILC planner configured with %d sampled points", sample_num);
  
  // 4. 可视化速度向量场
  //visualizeVelocityField(sampled_points);

  } else {
    ROS_WARN("[traj_server] ilcplan is nullptr, cannot configure ILC");
  }
}

void replanCallback(std_msgs::Empty msg) {
  /* reset duration */
  const double time_out = 0.01;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out;
  traj_duration_ = min(t_stop, traj_duration_);
}

void newCallback(std_msgs::Empty msg) {
  traj_cmd_.clear();
  traj_real_.clear();
}

void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  odom = msg;

  traj_real_.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
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
        quadrotor_msgs::PositionCommand cmd;
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

        // cout << cmd_vel << endl;
    }
    else
    {
        quadrotor_msgs::PositionCommand cmd;
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


int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  /*************************************** START IL-Planner相关变量 ***************************************/
  // 从参数服务器加载 ILC 参数
  nh.param("ILParam/vmax",       _vmax,        6.0);
  nh.param("ILParam/vmin",       _vmin,        2.0);
  nh.param("ILParam/goalth",     _goalth,      1.0);
  nh.param("ILParam/xth",        _xth,         0.5);
  nh.param("ILParam/kp_law",     _kp_law,      1.0);
  nh.param("ILParam/kp_vl",      _kp_vl,       0.4);
  nh.param("ILParam/kd_law",     _kd_law,      0.6);
  nh.param("ILParam/kp_path",    _kp_path,     2.9);
  nh.param("ILParam/kd_path",    _kd_path,     2.5);
  nh.param("ILParam/iteration",  _iteration,   10);
  nh.param("ILParam/dynamic",    _tau,         2.5);
  nh.param("ILParam/amax",       _amax,        6.0);

  // 初始化 ILC planner (仅在 traj_server 中创建和使用)
  ilcplan = new ilc_planner();
  ROS_INFO("[traj_server] ILC planner initialized");
  /*************************************** END IL-Planner相关变量 ***************************************/

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);

  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  vel_pub = node.advertise<quadrotor_msgs::PositionCommand>("planning/velocity_cmd", 50);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);
  vel_field_pub = node.advertise<visualization_msgs::Marker>("planning/velocity_field", 10);


  //  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);
  ros::Timer vel_call = nh.createTimer(ros::Duration(0.02), velCallback);

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

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}