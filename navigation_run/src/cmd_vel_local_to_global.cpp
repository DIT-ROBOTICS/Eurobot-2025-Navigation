#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
double car_global_x = 0;
double car_global_y = 0;
double car_global_z = 0;
float th = (double)3.1415926/2;
geometry_msgs::PoseWithCovarianceStamped ekf_pose_sub;
nav_msgs::Odometry odom_pose_sub;

void Callback(const geometry_msgs::Twist& data)
{
    car_global_x = data.linear.x;
    car_global_y = data.linear.y;
    car_global_z = data.angular.z;
}
void Callback_q(const geometry_msgs::PoseWithCovarianceStamped& data)
{
    ekf_pose_sub = data;
}

void Callback_odom(const nav_msgs::Odometry& data)
{
    odom_pose_sub = data;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_local_to_global");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel_global", 1000);
    ros::Subscriber sub = nh.subscribe("/robot/cmd_vel", 1000, Callback);
    // ros::Subscriber sub_q = nh.subscribe("/robot/ekf_pose", 1000, Callback_q);
    ros::Subscriber sub_q = nh.subscribe("/robot/odom", 1000, Callback_odom);
    ros::Rate loop_rate(20.0);
    geometry_msgs::Twist data_global;
    nav_msgs::Path path_msg;
    while(ros::ok())
    {
        tf2::Quaternion q;
        // tf2::fromMsg(ekf_pose_sub.pose.pose.orientation, q);
        tf2::fromMsg(odom_pose_sub.pose.pose.orientation, q);
        tf2::Matrix3x3 qt(q);
        double _, yaw;
        qt.getRPY(_, _, yaw);
        th= yaw;
        data_global.linear.x = (car_global_x * cos(th) - car_global_y * sin(th)) ;
        data_global.linear.y = (car_global_x * sin(th) + car_global_y * cos(th)) ;
        data_global.angular.z = car_global_z ;
        pub.publish(data_global);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}