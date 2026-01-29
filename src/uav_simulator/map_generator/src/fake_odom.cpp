/**
 * Simple fake odometry publisher for testing random_map_sensing
 * Publishes nav_msgs::Odometry on private topic "odometry" so that
 * random_map_sensing (which uses a private NodeHandle) can subscribe.
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

static double g_x = 0.0, g_y = 0.0, g_yaw = 0.0;
static bool g_has_goal = false;

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& p)
{
  // use 2D nav goal location; set z fixed to 1.0 when publishing
  g_x = p->pose.position.x;
  g_y = p->pose.position.y;
  g_yaw = tf::getYaw(p->pose.orientation);
  g_has_goal = true;
  ROS_INFO("[fake_odom] received goal x=%.3f y=%.3f yaw=%.3f (will publish z=1.0)", g_x, g_y, g_yaw);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fake_odom");
  ros::NodeHandle nh("~");

  double x=0.0, y=0.0, z=1.0;
  double yaw = 0.0;
  double rate_hz = 10.0;
  std::string frame_id = "world";
  std::string child_frame_id = "base_link";
  std::string topic = "odometry"; // private topic name
  std::string goal_topic = "/move_base_simple/goal"; // RViz 2D Nav Goal default

  nh.param("x", x, x);
  nh.param("y", y, y);
  // z is fixed to 1.0 altitude for published odometry
  nh.param("yaw", yaw, yaw);
  nh.param("rate", rate_hz, rate_hz);
  nh.param("frame_id", frame_id, frame_id);
  nh.param("child_frame_id", child_frame_id, child_frame_id);
  nh.param("topic", topic, topic);
  nh.param("goal_topic", goal_topic, goal_topic);

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(topic, 10);
  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic, 10, goalCallback);

  ros::Rate rate(rate_hz);

  // if no goal received, publish initial param pose (with z=1.0)
  if (!g_has_goal) {
    g_x = x;
    g_y = y;
    g_yaw = yaw;
    g_has_goal = true;
  }

  while (ros::ok()) {
    if (g_has_goal) {
      nav_msgs::Odometry msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = frame_id;
      msg.child_frame_id = child_frame_id;

      msg.pose.pose.position.x = g_x;
      msg.pose.pose.position.y = g_y;
      msg.pose.pose.position.z = z; // fixed 1m altitude

      tf::Quaternion q = tf::createQuaternionFromYaw(g_yaw);
      msg.pose.pose.orientation.x = q.x();
      msg.pose.pose.orientation.y = q.y();
      msg.pose.pose.orientation.z = q.z();
      msg.pose.pose.orientation.w = q.w();

      // zero velocity
      msg.twist.twist.linear.x = 0.0;
      msg.twist.twist.linear.y = 0.0;
      msg.twist.twist.linear.z = 0.0;

      odom_pub.publish(msg);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
