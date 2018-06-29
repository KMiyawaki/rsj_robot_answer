#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h> // <- スキャンデータのメッセージ型をinclude
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h> // 追記
#include <angles/angles.h>

class RsjRobotTestNode
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_scan_;     // <- URG用のサブスクライバを追加
  ros::Subscriber sub_clusters_; // 追記

  ros::Publisher pub_twist_;
  nav_msgs::Odometry odom_;

  void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
  }

  void cbScan(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
  }

  void cbCluster(const visualization_msgs::MarkerArray::ConstPtr &msg)
  {
    const visualization_msgs::Marker *target = NULL;
    for (visualization_msgs::MarkerArray::_markers_type::const_iterator
             it = msg->markers.cbegin(),
             it_end = msg->markers.cend();
         it != it_end; ++it)
    {
      const visualization_msgs::Marker &marker = *it;
      if (marker.ns == "target_cluster")
      {
        target = &marker;
      }
    }
    ROS_INFO("clusters: %zu", msg->markers.size());

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    if (target != NULL)
    {
      float dx = target->pose.position.x;
      float dy = target->pose.position.y;
      float d = ::hypot(dx, dy);
      if (d > 0.4)
      {
        float ang_max = angles::from_degrees(20.0);
        float lin_max = 0.2f;
        cmd_vel.angular.z = atan2(dy, dx);
        cmd_vel.linear.x = lin_max * (ang_max - fabs(cmd_vel.angular.z)) / ang_max;
        cmd_vel.linear.x = std::max<float>(0, cmd_vel.linear.x);
      }
      ROS_INFO("target: %f, %f velocities: %f(m/sec), %f(deg)", dx, dy, cmd_vel.linear.x, angles::to_degrees(cmd_vel.angular.z));
    }
    pub_twist_.publish(cmd_vel);
  }

public:
  RsjRobotTestNode()
      : nh_()
  {
    pub_twist_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    sub_odom_ = nh_.subscribe("odom", 5, &RsjRobotTestNode::cbOdom, this);
    sub_scan_ = nh_.subscribe("scan", 5, &RsjRobotTestNode::cbScan, this);            // <- URG用のサブスクライバ初期化コードを追加
    sub_clusters_ = nh_.subscribe("clusters", 5, &RsjRobotTestNode::cbCluster, this); // 追記
    odom_.pose.pose.orientation.w = 1.0;
  }
  void mainloop()
  {
    ROS_INFO("Hello ROS World!");

    ros::Rate rate(10.0);
    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rsj_robot_test_node");

  RsjRobotTestNode robot_test;

  robot_test.mainloop();
}
