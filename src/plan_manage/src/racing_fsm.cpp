#include <iostream>
#include <plan_manage/racing_fsm.h>
#include <std_msgs/Int32.h>


void RacingFSM::init(ros::NodeHandle& nh) {
  node_ = nh;
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;
  trigger_     = false;

  /*  fsm param  */


  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* get global init_pos */
  std::vector<double> init_p;
  if (nh.getParam("/init_pos", init_p) && init_p.size() == 3) {
    init_pos_ << init_p[0], init_p[1], init_p[2];
  } else {
    init_pos_ << 0.0, 0.0, 1.0;
    ROS_WARN("No init_pos parameter found, use default.");
  }

  /* initialize main modules */
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &RacingFSM::execFSMCallback, this);

  odom_sub_     = nh.subscribe("/odom_world", 1, &RacingFSM::odometryCallback, this);
  waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &RacingFSM::waypointCallback, this);
  
  // initialize timer
  near_start_time_ = ros::Time(0);
  flight_start_time_ = ros::Time(0);

  state_pub_     = nh.advertise<std_msgs::Int32>("/planning/state", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
  vis_traj_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/vis_traj", 100);
  lap_cnt_pub_ = nh.advertise<std_msgs::Int32>("/planning/lap_count", 10);

  // Clear RViz display
  ros::Duration(1.0).sleep();
  visualization_msgs::Marker delete_all;
  delete_all.action = 3; // visualization_msgs::Marker::DELETEALL may not be defined in older libs, but 3 is standard
  delete_all.id = 0;
  vis_traj_pub_.publish(delete_all);
}


void RacingFSM::waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

  if (msg->pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;
  trigger_ = true;

    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;


  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

  have_target_ = true;
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
  std::string state_str[5] = { "INIT", "GO_START", "WAIT_TRIGGER", "EXEC_TRAJ", "HOVER" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  
  std_msgs::Int32 state_msg;
  state_msg.data = int(new_state);
  state_pub_.publish(state_msg);
}

void RacingFSM::printFSMExecState() {
  std::string state_str[5] = { "INIT", "GO_START", "WAIT_TRIGGER", "EXEC_TRAJ", "HOVER" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void RacingFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      gen_ref_traj();
      vis_ref_traj();
      pub_ref_traj(); //发布参考轨迹后，飞机将会自动开始执行，进入EXEC_TRAJ状态

      vis_bound();
      changeFSMExecState(GO_START, "FSM");
      break;
    }

    case GO_START: {
      if (!have_odom_) return;

      // Calculate start tangent yaw from waypoints
      // p0 -> p1 direction
      Eigen::Vector3d p0(waypoints_[0][0], waypoints_[0][1], waypoints_[0][2]);
      Eigen::Vector3d p1(waypoints_[1][0], waypoints_[1][1], waypoints_[1][2]);
      Eigen::Vector3d start_dir = (p1 - p0).normalized();
      double start_yaw = atan2(start_dir(1), start_dir(0));

      // Calculate current yaw from odometry
      Eigen::Matrix3d R = odom_orient_.toRotationMatrix();
      Eigen::Vector3d body_dir = R.col(0); // Body X axis
      double current_yaw = atan2(body_dir(1), body_dir(0));
      
      // Compute yaw error
      double yaw_err = current_yaw - start_yaw;
      // Normalize to [-pi, pi]
      while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
      while (yaw_err < -M_PI) yaw_err += 2 * M_PI;
      yaw_err = fabs(yaw_err);

      // Check both position and yaw alignment
      if ((odom_pos_ - init_pos_).norm() < 0.5 && yaw_err < 0.3) {
        if (near_start_time_.isZero()) {
          near_start_time_ = ros::Time::now(); // Start timer if not already running
        } else if ((ros::Time::now() - near_start_time_).toSec() > 2.0) {
          changeFSMExecState(WAIT_TRIGGER, "FSM");
          near_start_time_ = ros::Time(0); // Reset timer
        }
      } else {
        near_start_time_ = ros::Time(0); // Reset timer if position drifts
      }

      break;
    }

    case WAIT_TRIGGER: {
      if (trigger_) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        trigger_ = false; // 重置trigger
        flight_start_time_ = ros::Time::now(); // Start flight timer
      }
      break;
    }

    case EXEC_TRAJ: {    
      // Case 1: Triggered again (e.g. by 2D Nav Goal)
      if (trigger_) {
        // Stop current trajectory
        changeFSMExecState(HOVER, "FSM");
        trigger_ = false;
        break;
      }

      // Case 2: Flight finished (To be implemented)
      bool flight_finished = false; 
      if (!traj_pts_.empty()) {
          Eigen::Vector3d end_pt = traj_pts_.back();
          double dist = (odom_pos_ - end_pt).norm();
          double bound = 1.0; 
          node_.getParam("ILParam/xth", bound);

          if (dist < bound && (ros::Time::now() - flight_start_time_).toSec() > 3.0) {
              flight_finished = true;
          }
      }

      if (flight_finished) {
        changeFSMExecState(HOVER, "FSM");
        trigger_ = false; // Reset trigger
        lap_cnt_++;
        std_msgs::Int32 lap_msg;
        lap_msg.data = lap_cnt_;
        lap_cnt_pub_.publish(lap_msg);
        break;
      }

    }

    case HOVER: {
      if (trigger_) {
        changeFSMExecState(GO_START, "FSM");
        trigger_ = false;
      }
      break;
    }

     default: {
      break;
    }
    
  }
}

void RacingFSM::gen_ref_traj() {
      // 在这里生成参考轨迹，并且以planning/bspline话题发出去
      
      // 1. Prepare control points (Use waypoints as B-spline control points directly)
      int p = 3; 
      // 为了避免B样条首尾导数为0导致的边界生成畸变，我们不仅重复首尾点，
      // 还需要向外延伸一点点，或者不要完全重复这么多。
      // 但为了保证穿过端点，通常需要重复p次。
      // 现在的做法是：首尾各重复p次作为padding，使得曲线严格穿过首尾。
      // 导致的问题：首尾速度(切线)可能出现奇异或者为0，从而导致vis_bound计算切线方向不稳定。
      // 改进：只重复p-1次，或者甚至不重复，而是人工添加延长的辅助点。
      // 既然这只是参考轨迹，我们可以简单地添加两个辅助点延伸出去，而不是原地重复点。
      
      int total_pts = waypoint_num_ + 2; 

      Eigen::MatrixXd ctrl_pts(total_pts, 3);
      
      // 添加起点前的辅助点：从起点沿(起点->第二个点)的反方向延伸
      Eigen::Vector3d p0(waypoints_[0][0], waypoints_[0][1], waypoints_[0][2]);
      Eigen::Vector3d p1(waypoints_[1][0], waypoints_[1][1], waypoints_[1][2]);
      Eigen::Vector3d start_ext = p0 - (p1 - p0).normalized() * 1.0; // 延伸1米
      
      ctrl_pts.row(0) = start_ext.transpose();
      
      // 填充原始点
      for (int i = 0; i < waypoint_num_; ++i) {
        ctrl_pts.row(i + 1) << waypoints_[i][0], waypoints_[i][1], waypoints_[i][2];
      }
      
      // 添加终点后的辅助点
      Eigen::Vector3d pn(waypoints_[waypoint_num_-1][0], waypoints_[waypoint_num_-1][1], waypoints_[waypoint_num_-1][2]);
      Eigen::Vector3d pn_1(waypoints_[waypoint_num_-2][0], waypoints_[waypoint_num_-2][1], waypoints_[waypoint_num_-2][2]);
      Eigen::Vector3d end_ext = pn + (pn - pn_1).normalized() * 1.0; // 延伸1米
      
      ctrl_pts.row(total_pts - 1) = end_ext.transpose();

      // 2. Time allocation based on average speed
      double avg_vel = 3.0; // m/s
      double total_dist = 0.0;
      for (int i = 0; i < waypoint_num_ - 1; ++i) {
        Eigen::Vector3d p1(waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]);
        Eigen::Vector3d p2(waypoints_[i+1][0], waypoints_[i+1][1], waypoints_[i+1][2]);
        total_dist += (p2 - p1).norm();
      }
      
      // For uniform B-spline, valid time segments = num_pts - order
      int valid_segments = total_pts - p;
      if (valid_segments <= 0) valid_segments = 1;
      
      double total_time = total_dist / avg_vel;
      double dt = total_time / double(valid_segments);

      // 3. Create B-spline object (to generate knots automatically)
      NonUniformBspline bspline(ctrl_pts, p, dt);

      // 4. Construct ROS Message
      bspline_msg_.order = p;
      bspline_msg_.start_time = ros::Time::now();
      static int traj_id_seq = 0;
      bspline_msg_.traj_id = ++traj_id_seq;
      bspline_msg_.pos_pts.clear();
      bspline_msg_.knots.clear();
      bspline_msg_.yaw_pts.clear();

      // Fill control points
      try {
        Eigen::VectorXd knots = bspline.getKnot();
        
        for (int i = 0; i < total_pts; ++i) {
          geometry_msgs::Point pt;
          pt.x = ctrl_pts(i, 0);
          pt.y = ctrl_pts(i, 1);
          pt.z = ctrl_pts(i, 2);
          bspline_msg_.pos_pts.push_back(pt);
        }

        // Fill knots
        for (int i = 0; i < knots.rows(); ++i) {
          bspline_msg_.knots.push_back(knots(i));
        }

        // Fill yaw points (set to 0 for simplicity)
        bspline_msg_.yaw_dt = dt;
        for (int i = 0; i < total_pts; ++i) {
          bspline_msg_.yaw_pts.push_back(0.0);
        }
        
        traj_pts_.clear();
        double dt_eval = 0.05;
        double duration = bspline.getTimeSum();
        for (double t = 0; t <= duration; t += dt_eval) {
          Eigen::Vector3d pt = bspline.evaluateDeBoorT(t);
          traj_pts_.push_back(pt);
        }
        traj_pts_num_ = traj_pts_.size();
      
      } catch (const std::exception& e) {
         ROS_ERROR("Bspline generation failed: %s", e.what());
      }
}

void RacingFSM::pub_ref_traj() {
  bspline_msg_.start_time = ros::Time::now();
  bspline_pub_.publish(bspline_msg_);
}


void RacingFSM::vis_ref_traj() {
        // Visualize trajectory
        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::LINE_STRIP;
        mk.action = visualization_msgs::Marker::ADD;
        mk.id = 0;
        mk.scale.x = 0.1; 
        mk.color.r = 1.0; mk.color.g = 0.0; mk.color.b = 0.0; mk.color.a = 1.0;
        
        for (int i = 0; i < traj_pts_num_; ++i) {
          geometry_msgs::Point p;
          p.x = traj_pts_[i](0); p.y = traj_pts_[i](1); p.z = traj_pts_[i](2);
          mk.points.push_back(p);
        }
        vis_traj_pub_.publish(mk);
}

void RacingFSM::vis_bound() {
  // read bound parameter (meters)
  double bound = 1.0;
  if (node_.getParam("ILParam/xth", bound)) {
      ROS_INFO("[racing_fsm] ILParam/xth loaded: %.3f", bound);
  } else if (node_.getParam("/il_planner_node/ILParam/xth", bound)) {
      ROS_INFO("[racing_fsm] /il_planner_node/ILParam/xth loaded: %.3f", bound);
  }

  if (traj_pts_.empty()) return;

  visualization_msgs::Marker inner_mk, outer_mk;
  inner_mk.header.frame_id = "world";
  inner_mk.header.stamp = ros::Time::now();
  inner_mk.type = visualization_msgs::Marker::LINE_STRIP;
  inner_mk.action = visualization_msgs::Marker::ADD;
  inner_mk.id = 2;
  inner_mk.scale.x = 0.08;
  inner_mk.color.r = 0.0; inner_mk.color.g = 0.3; inner_mk.color.b = 1.0; inner_mk.color.a = 1.0;

  outer_mk = inner_mk;
  outer_mk.id = 3;
  outer_mk.color.r = 1.0; outer_mk.color.g = 0.0; outer_mk.color.b = 0.8; outer_mk.color.a = 1.0;

  // Build offset points along trajectory normal in XY plane
  for (int i = 0; i < traj_pts_num_; ++i) {
    Eigen::Vector3d p = traj_pts_[i];

    // compute tangent
    Eigen::Vector3d t;
    if (i == 0) t = (traj_pts_[i+1] - traj_pts_[i]).normalized();
    else if (i == traj_pts_num_ - 1) t = (traj_pts_[i] - traj_pts_[i-1]).normalized();
    else t = (traj_pts_[i+1] - traj_pts_[i-1]).normalized();

    // project tangent to XY plane and normalize
    Eigen::Vector3d t_xy(t(0), t(1), 0.0);
    double txy_norm = t_xy.norm();
    if (txy_norm < 1e-6) {
      // if tangent is vertical, choose X axis as tangent
      t_xy = Eigen::Vector3d(1, 0, 0);
      txy_norm = 1.0;
    }
    t_xy /= txy_norm;

    // normal in XY plane (rotate tangent by +90deg)
    Eigen::Vector3d n(-t_xy(1), t_xy(0), 0.0);

    Eigen::Vector3d inner_p = p - n * bound;
    Eigen::Vector3d outer_p = p + n * bound;

    geometry_msgs::Point gp_inner, gp_outer;
    gp_inner.x = inner_p.x(); gp_inner.y = inner_p.y(); gp_inner.z = inner_p.z();
    gp_outer.x = outer_p.x(); gp_outer.y = outer_p.y(); gp_outer.z = outer_p.z();

    inner_mk.points.push_back(gp_inner);
    outer_mk.points.push_back(gp_outer);
  }

  vis_traj_pub_.publish(inner_mk);
  vis_traj_pub_.publish(outer_mk);
}
