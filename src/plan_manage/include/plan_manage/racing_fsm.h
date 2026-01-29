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



#ifndef _RACING_FSM_H_
#define _RACING_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_manage/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;



class RacingFSM {

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, HOVER, GEN_NEW_TRAJ, EXEC_TRAJ };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  PlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;
  double waypoints_[50][3];
  int waypoint_num_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_;                              // target state
  int current_wp_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, test_something_timer_;
  ros::Subscriber odom_sub_, waypoint_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, vis_traj_pub_, vis_waypoint_pub_;

  /* helper functions */
  bool callTubeRRTReplan(bool first);        // front-end and back-end method
  void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent& e);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg);

public:
  RacingFSM(/* args */) {
  }
  ~RacingFSM() {
  }

  void init(ros::NodeHandle& nh);

};


#endif