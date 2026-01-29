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

#include <iostream>
#include <plan_manage/racing_fsm.h>


void RacingFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);


  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules */
  planner_manager_.reset(new PlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &RacingFSM::execFSMCallback, this);

  odom_sub_     = nh.subscribe("/odom_world", 1, &RacingFSM::odometryCallback, this);
  waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &RacingFSM::waypointCallback, this);

  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  
  // 生成参考轨迹
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
  vis_traj_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/vis_traj", 100);
  vis_waypoint_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/vis_waypoints", 10);
}


void RacingFSM::waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

  if (msg->pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->pose.position.x, msg->pose.position.y, 0.3;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

  have_target_ = true;

  if (exec_state_ == HOVER)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(HOVER, "TRIG");
}



void RacingFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void RacingFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
  std::string state_str[4] = { "INIT", "HOVER", "GEN_NEW_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void RacingFSM::printFSMExecState() {
  std::string state_str[4] = { "INIT", "HOVER", "GEN_NEW_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void RacingFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(HOVER, "FSM");
      break;
    }

    case HOVER: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt_  = odom_pos_;

      // 在这里生成参考轨迹，并且以planning/bspline话题发出去
      
      // 1. Prepare control points (Use waypoints as B-spline control points directly)
      // To create a closed loop, we append the first p points to the end
      int p = 3; 
      int extra_pts = p; 
      int total_pts = waypoint_num_ + extra_pts;

      Eigen::MatrixXd ctrl_pts(total_pts, 3);
      for (int i = 0; i < waypoint_num_; ++i) {
        ctrl_pts.row(i) << waypoints_[i][0], waypoints_[i][1], waypoints_[i][2];
      }
      // Wrap around for closed loop
      for (int i = 0; i < extra_pts; ++i) {
        ctrl_pts.row(waypoint_num_ + i) = ctrl_pts.row(i);
      }

      // 2. Time allocation based on average speed
      double avg_vel = 3.0; // m/s
      double total_dist = 0.0;
      for (int i = 0; i < total_pts - 1; ++i) {
        total_dist += (ctrl_pts.row(i+1) - ctrl_pts.row(i)).norm();
      }
      
      // For uniform B-spline, valid time segments = num_pts - order
      int valid_segments = total_pts - p;
      if (valid_segments <= 0) valid_segments = 1;
      
      double total_time = total_dist / avg_vel;
      double dt = total_time / double(valid_segments);

      // 3. Create B-spline object (to generate knots automatically)
      NonUniformBspline bspline(ctrl_pts, p, dt);

      // 4. Construct ROS Message
      plan_manage::Bspline bspline_msg;
      bspline_msg.order = p;
      bspline_msg.start_time = ros::Time::now();
      static int traj_id_seq = 0;
      bspline_msg.traj_id = ++traj_id_seq;

      // Fill control points
      for (int i = 0; i < total_pts; ++i) {
        geometry_msgs::Point pt;
        pt.x = ctrl_pts(i, 0);
        pt.y = ctrl_pts(i, 1);
        pt.z = ctrl_pts(i, 2);
        bspline_msg.pos_pts.push_back(pt);
      }

      // Fill knots
      Eigen::VectorXd knots = bspline.getKnot();
      for (int i = 0; i < knots.rows(); ++i) {
        bspline_msg.knots.push_back(knots(i));
      }

      // Fill yaw points (set to 0 for simplicity)
      bspline_msg.yaw_dt = dt;
      for (int i = 0; i < total_pts; ++i) {
        bspline_msg.yaw_pts.push_back(0.0);
      }

      // Publish
      bspline_pub_.publish(bspline_msg);
      ROS_INFO("Generated Loop Trajectory with %d points", total_pts);

      // Visualize trajectory
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.type = visualization_msgs::Marker::LINE_STRIP;
      mk.action = visualization_msgs::Marker::ADD;
      mk.id = 0;
      mk.scale.x = 0.1; 
      mk.color.r = 1.0; mk.color.g = 0.0; mk.color.b = 0.0; mk.color.a = 1.0;
      
      double dt_eval = 0.05;
      double duration = bspline.getTimeSum();
      for (double t = 0; t <= duration; t += dt_eval) {
         Eigen::Vector3d pt = bspline.evaluateDeBoorT(t);
         geometry_msgs::Point p;
         p.x = pt(0); p.y = pt(1); p.z = pt(2);
         mk.points.push_back(p);
      }
      vis_traj_pub_.publish(mk);

      // Visualize waypoints
      visualization_msgs::Marker mk_wp;
      mk_wp.header.frame_id = "world";
      mk_wp.header.stamp = ros::Time::now();
      mk_wp.type = visualization_msgs::Marker::SPHERE_LIST;
      mk_wp.action = visualization_msgs::Marker::ADD;
      mk_wp.id = 1;
      mk_wp.scale.x = 0.3; mk_wp.scale.y = 0.3; mk_wp.scale.z = 0.3; 
      mk_wp.color.r = 0.0; mk_wp.color.g = 1.0; mk_wp.color.b = 0.0; mk_wp.color.a = 1.0;

      for (int i = 0; i < total_pts; ++i) {
        geometry_msgs::Point pt;
        pt.x = ctrl_pts(i, 0);
        pt.y = ctrl_pts(i, 1);
        pt.z = ctrl_pts(i, 2);
        mk_wp.points.push_back(pt);
      }
      vis_waypoint_pub_.publish(mk_wp);
      vis_traj_pub_.publish(mk);

      bool success = true;
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        have_target_ = false;
        changeFSMExecState(HOVER, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */


    }

    
  }
}
