#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>

class RsjRobotTestNode
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_scan_;
  ros::Subscriber sub_clusters_;

  ros::Publisher pub_twist_;
  ros::Publisher pub_goal_;

  nav_msgs::Odometry odom_;
  visualization_msgs::MarkerArray marker_array_;

  enum
  {
    STATE_TURN_1 = 1,
    STATE_TURN_2,
    STATE_TURN_3,
    STATE_SEARCHING,
    STATE_TRACING,
  };

  int state_;

  void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
    odom_ = *msg;
  }

  void cbScan(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
  }

  void cbCluster(const visualization_msgs::MarkerArray::ConstPtr &msg)
  {
    marker_array_ = *msg;
  }

  bool foundValidTarget(float &target_x, float &target_y, float min_distance = 0.3, float max_distance = 10.0) const
  {
    target_x = 0;
    target_y = 0;
    for (visualization_msgs::MarkerArray::_markers_type::const_iterator
             it = marker_array_.markers.cbegin(),
             it_end = marker_array_.markers.cend();
         it != it_end; ++it)
    {
      const visualization_msgs::Marker &marker = *it;
      if (marker.ns == "target_cluster")
      {
        target_x = marker.pose.position.x;
        target_y = marker.pose.position.y;
      }
    }
    float d = hypotf(target_x, target_y);
    return min_distance < d && d < max_distance;
  }

public:
  RsjRobotTestNode()
      : nh_(), state_(STATE_TURN_1)
  {
    pub_twist_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    sub_odom_ = nh_.subscribe("odom", 5, &RsjRobotTestNode::cbOdom, this);
    sub_scan_ = nh_.subscribe("scan", 5, &RsjRobotTestNode::cbScan, this);
    sub_clusters_ = nh_.subscribe("clusters", 5, &RsjRobotTestNode::cbCluster, this);
    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, true);
    odom_.pose.pose.orientation.w = 1.0;
  }
  void mainloop()
  {
    ROS_INFO("Hello ROS World!");
    ros::Rate rate(20.0);
    float ang_vel = angles::from_degrees(20);
    while (ros::ok())
    {
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      float yaw = angles::to_degrees(tf::getYaw(odom_.pose.pose.orientation));
      switch (state_)
      {
        // ロボットを少し回転させて自己位置推定をさせる。
      case STATE_TURN_1:
      {
        if (yaw < 90.0f)
        {
          cmd_vel.angular.z = ang_vel;
        }
        else
        {
          state_ = STATE_TURN_2;
        }
        pub_twist_.publish(cmd_vel);
      }
      break;
      case STATE_TURN_2:
      {
        if (yaw > -90.0f)
        {
          cmd_vel.angular.z = -ang_vel;
        }
        else
        {
          state_ = STATE_TURN_3;
        }
        pub_twist_.publish(cmd_vel);
      }
      break;
      case STATE_TURN_3:
      {
        if (yaw < 0.0f)
        {
          cmd_vel.angular.z = ang_vel;
        }
        else
        {
          state_ = STATE_SEARCHING;
        }
        pub_twist_.publish(cmd_vel);
      }
      break;
      case STATE_SEARCHING:
      {
        float target_x, target_y;
        if (foundValidTarget(target_x, target_y))
        {
          geometry_msgs::PoseStamped goal;
          goal.header.frame_id = "base_link"; // ロボットローカル座標におけるゴール位置を指定する。
          goal.pose.position.x = target_x;
          goal.pose.position.y = target_y;
          goal.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(target_y, target_x));
          pub_goal_.publish(goal);
          state_ = STATE_TRACING;
          ROS_INFO("Start Tracing. target %3.2f, %3.2f", target_x, target_y);
        }
      }
      break;
      }
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
