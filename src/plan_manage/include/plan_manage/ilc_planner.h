#ifndef _ILC_PLANNER_H_
#define _ILC_PLANNER_H_


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>

#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>


class ilc_planner
{
    private:
        double v_max;
        double v_min;
        double goal_th;
        double kp_vl;
        double kp_law;
        double kd_law;
        std::vector<double> x_th;
        double kp_path;
        double kd_path;
        double _tau;
        int    _iteration;

        double _last_error = 0.0;

    public:
        ilc_planner(){};
        ~ilc_planner(){};

        std::vector<Eigen::Vector3d> _global_plan;
        Eigen::Vector3d _robot_pose;
        Eigen::Vector3d _robot_vel;
        std::vector<double> vel_list;

        // void set_param(double vmin, double vmax, double goalth);
        void set_param(double vmin, double vmax, double goalth, double kpvl, double kplaw, double kdlaw, std::vector<double> xth, double kppath, double kdpath, double tau, int iteration);

        bool judge_reach();

        bool setPlan(std::vector<Eigen::Vector3d> orig_global_plan, Eigen::Vector3d robot_pose, Eigen::Vector3d robot_vel);
        
        bool get_index(Eigen::Vector3d& refer_pose, Eigen::Vector3d& move_direction, Eigen::Vector3d& move_track, int& index);

        bool pruneGlobalPlan(double dist_behind_robot = 1);

        void getpoint(Eigen::Vector3d p, Eigen::Vector3d &refer_pose, Eigen::Vector3d &move_direction, Eigen::Vector3d &move_track, double &error, int &current);
  
        void saturation(Eigen::Vector3d &v, double vmax);

        bool spatialILC(std::vector<double>& v_list, int iteration);

        void dynamic(Eigen::Vector3d vdes, Eigen::Vector3d &v, Eigen::Vector3d &p);

        void iterate(std::vector<double> error_list);

        bool computeVelocityCommands(Eigen::Vector3d &cmd_vel, Eigen::Vector3d &refer_pose, double &yaw_des, Eigen::Vector3d robot_pose);

        bool get_v_list(Eigen::Vector3d robot_pose, Eigen::Vector3d &refer_pose, Eigen::Vector3d& move_direction, Eigen::Vector3d& move_track, int& index);

        Eigen::Vector3d getCurvature(int l);
};

#endif
