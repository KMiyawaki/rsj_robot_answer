#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

class RsjRobotTestNode
{
private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_odom_;
  ros::Subscriber sub_scan_;
  ros::Publisher pub_twist_;

  sensor_msgs::LaserScan latest_scan_;

  void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
  }

  void cbScan(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    // 受け取ったメッセージをコピーしておく
    latest_scan_ = *msg;
  }

public:
  RsjRobotTestNode()
      : nh_()
  {
    pub_twist_ = nh_.advertise<geometry_msgs::Twist>(
        "cmd_vel", 5);
    sub_odom_ = nh_.subscribe(
        "odom", 5, &RsjRobotTestNode::cbOdom, this);
    sub_scan_ = nh_.subscribe(
        "scan", 5, &RsjRobotTestNode::cbScan, this);
  }
  void mainloop()
  {
    ROS_INFO("Hello ROS World!");

    ros::Rate rate(20.0);
    while (ros::ok())
    {
      ros::spinOnce();

      if (latest_scan_.ranges.size() > 0)
      {
        // LaserScanメッセージをすでに受け取っている場合
        float hx = 0, hy = 0;
        int hnum = 0;

        // theta-range 座標系から x-y 座標系に変換
        for (unsigned int i = 0; i < latest_scan_.ranges.size(); i++)
        {
          if (!(latest_scan_.ranges[i] < latest_scan_.range_min ||
                latest_scan_.ranges[i] > latest_scan_.range_max ||
                std::isnan(latest_scan_.ranges[i])))
          {
            // 距離値がエラーでない場合
            float theta = latest_scan_.angle_min + i * latest_scan_.angle_increment;
            // x-y 座標系に変換
            float x = latest_scan_.ranges[i] * cosf(theta);
            float y = latest_scan_.ranges[i] * sinf(theta);

            if (fabs(y) < 0.5 && x > 0.05 && x < 1.05)
            {
              // ロボット正面方向、1m四方の領域にある点の重心を計算
              hx += x;
              hy += y;
              hnum++;
            }
          }
        }
        geometry_msgs::Twist cmd_vel;
        if (hnum > 0)
        {
          hy /= hnum;
          hx /= hnum;
          float dir = atan2(hy, hx);
          float dist = hypotf(hy, hx);

          ROS_INFO("Following the object in front (dist %0.3f, dir %0.3f)", dist, dir);

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
      }
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
