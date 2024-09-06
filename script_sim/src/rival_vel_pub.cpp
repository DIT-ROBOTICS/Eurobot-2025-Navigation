#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "rival_vel_pub");
    ros::NodeHandle nh;
    ros::Publisher rival_vel_pub = nh.advertise<geometry_msgs::Twist>("/RivalVel", 1);
    ros::Time last_time = ros::Time::now();
    double cur_vel = 0.5;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if ((ros::Time::now() - last_time).toSec() > 3.5) {
            cur_vel = -cur_vel;
            last_time = ros::Time::now();
        }
        geometry_msgs::Twist msg;
        msg.linear.x = cur_vel;
        msg.linear.y = cur_vel;
        rival_vel_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}