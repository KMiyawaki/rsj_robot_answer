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

    ros::Publisher pub_goal_;

    enum
    {
        STATE_SEARCHING = 1,
        STATE_TRACING,
    };

    int state_;
    float target_x_;
    float target_y_;

    void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
    {
        odom_ = *msg; // 追記
    }

    void cbScan(const sensor_msgs::LaserScan::ConstPtr &msg)
    { }

    void cbCluster(const visualization_msgs::MarkerArray::ConstPtr &msg)
    {
        for (visualization_msgs::MarkerArray::_markers_type::const_iterator
                 it = msg->markers.cbegin(),
                 it_end = msg->markers.cend();
             it != it_end; ++it)
        {
            const visualization_msgs::Marker &marker = *it;
            if (marker.ns == "target_cluster")
            {
                target_x_ = marker.pose.position.x;
                target_y_ = marker.pose.position.y;
            }
        }
    }

    bool foundValidTarget(float min_distance = 0.3, float max_distance = 10.0) const
    {
        float d = ::hypot(target_x_, target_y_);
        return min_distance < d && d < max_distance;
    }

  public:
    RsjRobotTestNode()
        : nh_(), state_(STATE_SEARCHING), target_x_(0), target_y_(0)
    {
        pub_twist_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
        sub_odom_ = nh_.subscribe("odom", 5, &RsjRobotTestNode::cbOdom, this);
        sub_scan_ = nh_.subscribe("scan", 5, &RsjRobotTestNode::cbScan, this);            // <- URG用のサブスクライバ初期化コードを追加
        sub_clusters_ = nh_.subscribe("clusters", 5, &RsjRobotTestNode::cbCluster, this); // 追記
        odom_.pose.pose.orientation.w = 1.0;
        pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, true);
    }
    void mainloop()
    {
        ROS_INFO("Hello ROS World!");

        ros::Rate rate(10.0);
        while (ros::ok())
        {
            if (state_ == STATE_SEARCHING)
            {
                if (foundValidTarget())
                {
                    geometry_msgs::PoseStamped goal;
                    goal.header.frame_id = "base_link"; // ロボットローカル座標におけるゴール位置を指定する。
                    goal.pose.position.x = target_x_;
                    goal.pose.position.y = target_y_;
                    goal.pose.orientation = tf::createQuaternionMsgFromYaw(0);
                    pub_goal_.publish(goal);
                    state_ = STATE_TRACING;
                    ROS_INFO("Start Tracing. target %3.2f, %3.2f", target_x_, target_y_);
                }
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
