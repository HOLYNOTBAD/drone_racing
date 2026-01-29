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



#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline_opt/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>


#include <ros/ros.h>

using namespace std;


struct PlanParameters {
  /* planning algorithm parameters */
  double max_vel_, max_acc_, max_jerk_;  // physical limits
  double local_traj_len_;                // local replanning trajectory length
  double ctrl_pt_dist;                   // distance between adjacient B-spline
                                         // control points
  double clearance_;
  int dynamic_;
  double max_radius_; // added for tube radius configuration
  /* processing time */
  double time_search_ = 0.0;
  double time_optimize_ = 0.0;
  double time_adjust_ = 0.0;
};


// Fast Planner Manager
// Key algorithms of mapping and planning are called

class PlannerManager {
  // SECTION stable
public:
  PlannerManager();
  ~PlannerManager();

  /* main planning interface */
  void planYaw(const Eigen::Vector3d& start_yaw);

  void initPlanModules(ros::NodeHandle& nh);

  bool checkTrajCollision(double& distance);

  PlanParameters pp_;



  
private:


  // heading planning
  void calcNextYaw(const double& last_yaw, double& yaw);

  // !SECTION stable

  // SECTION developing

public:
  typedef unique_ptr<PlannerManager> Ptr;

  // !SECTION
};

#endif