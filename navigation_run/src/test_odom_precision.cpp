#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
int main(int argc, char** argv){
    ros::init(argc, argv, "test_odom_precision");
    ros::NodeHandle nh, private_nh("~");
    double time, x_vel ,y_vel ,z_vel,linear_acc,angular_acc,x_dis,y_dis,z_dis,stop_dis,stop_dacc,min_vel,test_param;
    private_nh.param<double>("time", time, 3);
    private_nh.param<double>("x_vel", x_vel, 0);
    private_nh.param<double>("y_vel", y_vel, 0);
    private_nh.param<double>("z_vel", z_vel, 0);
    private_nh.param<double>("linear_acc", linear_acc, 3);
    private_nh.param<double>("angular_acc", angular_acc, 0);
    private_nh.param<double>("x_dis", x_dis, 0);
    private_nh.param<double>("y_dis", y_dis, 0);
    private_nh.param<double>("z_dis", z_dis, 0);
    private_nh.param<double>("stop_dis", stop_dis, 0);
    private_nh.param<double>("stop_dacc", stop_dacc,0);
    private_nh.param<double>("min_vel", min_vel,0);
    private_nh.param<double>("test_param", test_param, 1);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    geometry_msgs::Twist cmd_msg;
    geometry_msgs::Twist odom_msg;
    cmd_msg.linear.x = 0;
    cmd_msg.linear.y = 0;
    cmd_msg.angular.z = 0;
    odom_msg.linear.x = 0;
    odom_msg.linear.y = 0;
    odom_msg.angular.z = 0;
    ros::Duration(2).sleep();
    ros::Time cur = ros::Time::now();
    ros::Time time_before = ros::Time::now();
    ros::Rate rate(20);
    ROS_INFO_STREAM("START");
    while (ros::ok()) {
        double dt = (ros::Time::now() - time_before).toSec();
        if (odom_msg.linear.x <= x_dis* test_param /*&& odom_msg.angular.z <= z_dis*/) {
            if(odom_msg.linear.x >= x_dis* test_param - stop_dis /*&& odom_msg.angular.z >= z_dis - stop_dis*/){
                cmd_msg.linear.x -= stop_dacc * dt;
                cmd_msg.linear.y -= 0;
                // cmd_msg.angular.z -= stop_dacc * dt;
                if(cmd_msg.linear.x < min_vel)cmd_msg.linear.x = min_vel;
                // if(cmd_msg.linear.y < y_vel)cmd_msg.linear.y = y_vel;
                // if(cmd_msg.angular.z < min_vel)cmd_msg.angular.z = min_vel;
                // ROS_ERROR("stop");
            }
            else{
                cmd_msg.linear.x += linear_acc * dt;
                cmd_msg.linear.y += 0;
                // cmd_msg.angular.z += angular_acc * dt;
            }
        }
        else {
            cmd_msg.linear.x = 0;
            cmd_msg.linear.y = 0;
            cmd_msg.angular.z = 0;
            ROS_ERROR("you are arrived");
        }
        // cmd_msg.angular.z = 0;
        if(cmd_msg.linear.x > x_vel)cmd_msg.linear.x = x_vel;
        if(cmd_msg.linear.y > y_vel)cmd_msg.linear.y = y_vel;
        // if(cmd_msg.angular.z > z_vel)cmd_msg.angular.z = z_vel;
        odom_msg.linear.x += cmd_msg.linear.x*dt;
        odom_msg.linear.y += cmd_msg.linear.y*dt;
        // odom_msg.angular.z += cmd_msg.angular.z*dt;
        ROS_INFO("cmd_msg.x:%f linear_acc:%f odom_x:%f",cmd_msg.linear.x,linear_acc,odom_msg.linear.x);
        time_before = ros::Time::now();
        pub.publish(cmd_msg);
        rate.sleep();
    }
}