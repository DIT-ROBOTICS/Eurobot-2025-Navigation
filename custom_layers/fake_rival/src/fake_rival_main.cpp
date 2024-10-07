#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

geometry_msgs::Point current;
geometry_msgs::Point start;
geometry_msgs::Point goal;

double dx = 0;
double dy = 0;
double vel = 0; // m/s
int point = 0;

void move()
{
    dx = 0.02* vel* ((goal.x-current.x)/sqrt(pow(current.x-goal.x, 2) + pow(current.y-goal.y, 2)));
    dy = 0.02* vel* ((goal.y-current.y)/sqrt(pow(current.x-goal.x, 2) + pow(current.y-goal.y, 2)));
    if(abs(current.x-goal.x) > 0.2)
        current.x += dx;
    if(abs(current.y-goal.y) > 0.2)    
        current.y += dy;
}

void random_point()
{
    double now = ros::Time::now().toSec();
    goal.x = abs(((int)now*532)%275) *0.01 + 0.125;
    goal.y = abs(((int)now*762)%175) *0.01 + 0.125;
    vel = abs(((int)now*671)%100) *0.001+0.25;
}

void timerCallback(const ros::TimerEvent&)
{
    if(abs(current.x-goal.x) < 0.2 && abs(current.y-goal.y) < 0.2)
    {
        random_point();
    }

    move();
}


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "fake_rival");
    ros::NodeHandle nh;

    ros::Publisher fake_rival_pub = nh.advertise<nav_msgs::Odometry>("/rival/final_pose", 100);

    ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCallback);
    ros::Rate loop_rate(50);

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.w = 0.0;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.angular.z = 0.0;

    current.x = 0.0;
    current.y = 0.0;
    current.z = 0.0;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;
    goal.x = 0.0;
    goal.y = 0.0;
    goal.z = 0.0;



    // Main loop to publish messages
    while (ros::ok())
    {
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";

        odom.pose.pose.position = current;

        fake_rival_pub.publish(odom);
        
        std::cout<<"x = "<<goal.x<<"\n";
        std::cout<<"y = "<<goal.y<<"\n";

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
