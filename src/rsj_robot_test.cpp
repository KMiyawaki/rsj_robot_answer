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
  ros::Subscriber sub_scan_;
  ros::Subscriber sub_clusters_;

  ros::Publisher pub_twist_;
  nav_msgs::Odometry odom_;

  visualization_msgs::MarkerArray marker_array_;

  void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
  }

  void cbScan(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
  }

  void cbCluster(const visualization_msgs::MarkerArray::ConstPtr &msg)
  {
    marker_array_ = *msg;
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

    ros::Rate rate(20.0);
    while (ros::ok())
    {
      ros::spinOnce();

      const visualization_msgs::Marker *target = NULL;
      for (visualization_msgs::MarkerArray::_markers_type::const_iterator
               it = marker_array_.markers.cbegin(),
               it_end = marker_array_.markers.cend();
           it != it_end; ++it)
      {
        const visualization_msgs::Marker &marker = *it;
        if (marker.ns == "target_cluster")
        {
          target = &marker;
        }
      }

      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      if (target != NULL)
      {
        float dx = target->pose.position.x;
        float dy = target->pose.position.y;
        float dist = hypotf(dx, dy);
        float dir = atan2(dy, dx);

        ROS_INFO("Following the object in front (dist %0.3f, dir %0.3f)",
                 dist, dir);

        // 角加速度0.6rad/ssで、向きを最短時間制御
        cmd_vel.angular.z = sqrtf(2.0 * 0.6 * fabs(dir));
        if (dir < 0)
        {
          cmd_vel.angular.z *= -1;
        }
        if (cmd_vel.angular.z > 0.4)
        {
          cmd_vel.angular.z = 0.4;
        }
        else if (cmd_vel.angular.z < -0.4)
        {
          cmd_vel.angular.z = -0.4;
        }

        // 加速度0.3m/ssで、距離を最短時間制御
        cmd_vel.linear.x = sqrtf(2.0 * 0.3 * fabs(dist - 0.5));
        if (dist - 0.5 < 0)
        {
          cmd_vel.linear.x *= -1;
        }
        if (cmd_vel.linear.x > 0.2)
        {
          cmd_vel.linear.x = 0.2;
        }
        else if (cmd_vel.linear.x < -0.2)
        {
          cmd_vel.linear.x = -0.2;
        }
      }
      pub_twist_.publish(cmd_vel);
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
