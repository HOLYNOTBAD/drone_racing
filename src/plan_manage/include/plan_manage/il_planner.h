#ifndef _IL_PLANNER_H_
#define _IL_PLANNER_H_


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


class il_planner
{
    private:
        /* ros param */
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

        /* variables */
        double _last_error = 0.0;

    public:
        il_planner(){};
        ~il_planner(){};

        /* General variables */
        std::vector<Eigen::Vector3d> _global_plan;
        std::vector<double> vel_list;

        /* ILP variables */
        Eigen::Vector3d _robot_pose;
        Eigen::Vector3d _robot_vel;
        
        /* ILC variables */
        bool use_ilc_ ; 
        std::vector<double> error_list_ilc; // real flight error list
        int current,current_all;
        int l,last_l;
        double error,last_error;

        // General Methods
        void set_param(double vmin, double vmax, double goalth, double kpvl, double kplaw, double kdlaw, std::vector<double> xth, double kppath, double kdpath, double tau, int iteration, bool use_ilc);
        bool setPlan(std::vector<Eigen::Vector3d> orig_global_plan, Eigen::Vector3d robot_pose, Eigen::Vector3d robot_vel);
        bool computeVelocityCommands(Eigen::Vector3d &cmd_vel, Eigen::Vector3d &refer_pose, double &yaw_des, Eigen::Vector3d robot_pose);

        // ILP Methods
        bool spatialILP(std::vector<double>& v_list, int iteration);
        
        // ILC Methods
        bool spatialILC_init(std::vector<double>& v_list);
        void spatialILC_record(Eigen::Vector3d pos);

        // Basic Methods
        void dynamic(Eigen::Vector3d vdes, Eigen::Vector3d &v, Eigen::Vector3d &p);
        void iterate(std::vector<double> error_list);      
        bool get_index(Eigen::Vector3d& refer_pose, Eigen::Vector3d& move_direction, Eigen::Vector3d& move_track, int& index);
        void getpoint(Eigen::Vector3d p, Eigen::Vector3d &refer_pose, Eigen::Vector3d &move_direction, Eigen::Vector3d &move_track, double &error, int &current);
        void saturation(Eigen::Vector3d &v, double vmax);
        bool get_v_list(Eigen::Vector3d robot_pose, Eigen::Vector3d &refer_pose, Eigen::Vector3d& move_direction, Eigen::Vector3d& move_track, int& index);
        Eigen::Vector3d getCurvature(int l);

        // TODO
        bool pruneGlobalPlan(double dist_behind_robot = 1);

};

#endif
