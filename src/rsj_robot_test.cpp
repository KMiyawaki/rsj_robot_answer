#include <ros/ros.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

class rsj_robot_test_node
{
  private:
    nav_msgs::Odometry odom;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_scan;
    ros::Subscriber sub_clusters;
    ros::Publisher pub_twist;

    void cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
    {
        ROS_INFO("vel %f", msg->twist.twist.linear.x);
        odom = *msg;
    }

    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        int i = msg->ranges.size() / 2;
        if (msg->ranges[i] < msg->range_min || // エラー値の場合
            msg->ranges[i] > msg->range_max || // 測定範囲外の場合
            std::isnan(msg->ranges[i]))        // 無限遠の場合
        {
            ROS_INFO("front-range: measurement error");
        }
        else
        {
            ROS_INFO("front-range: %0.3f",
                     msg->ranges[msg->ranges.size() / 2]);
        }
    }

    void cb_cluster(const visualization_msgs::MarkerArray::ConstPtr &msg)
    {
        const visualization_msgs::Marker *target = NULL;
        for (visualization_msgs::MarkerArray::_markers_type::const_iterator it = msg->markers.cbegin(), it_end = msg->markers.cend();
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
        pub_twist.publish(cmd_vel);
    }

  public:
    rsj_robot_test_node()
    {
        ros::NodeHandle nh("~");
        pub_twist = nh.advertise<geometry_msgs::Twist>(
            "/ypspur_ros/cmd_vel", 5);
        sub_odom = nh.subscribe("/ypspur_ros/odom", 5,
                                &rsj_robot_test_node::cb_odom, this);
        sub_scan = nh.subscribe("/scan", 5,
                                &rsj_robot_test_node::cb_scan, this);
        sub_clusters = nh.subscribe("/rsj_pointcloud_test_node/clusters", 5,
                                    &rsj_robot_test_node::cb_cluster, this);
        odom.pose.pose.orientation.w = 1.0;
    }
    void mainloop()
    {
        ROS_INFO("Hello ROS World!");

        ros::Rate rate(10.0);
        while (ros::ok())
        {
            ros::spinOnce();
#if 0
            geometry_msgs::Twist cmd_vel;
            if (tf::getYaw(odom.pose.pose.orientation) > 1.57)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.1;
            }
            pub_twist.publish(cmd_vel);
#endif
            rate.sleep();
        }
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rsj_robot_test_node");

    rsj_robot_test_node robot_test;

    robot_test.mainloop();
}
