
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <plan_manage/racing_fsm.h>
#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "il_planner_node");
  ros::NodeHandle nh("~");

  int planner;
  nh.param("planner_node/planner", planner, -1);

  RacingFSM racing_fsm;

  racing_fsm.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
