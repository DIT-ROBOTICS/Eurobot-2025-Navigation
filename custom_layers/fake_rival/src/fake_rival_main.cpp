#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <vector>

//please go to the config/fake_rival_param.yaml to adjust the parameter
//reset rival is available(use /rival/reset)
//reset robot hasn't done

nav_msgs::Odometry odom;
geometry_msgs::Point goal;

double dx = 0;
double dy = 0;
double vel = 0;
bool enable = false;
bool random_mode = false;
std::vector<double> footprint_x;
std::vector<double> footprint_y;
int size = 0;
int point = 0;

void move();
void random_point();
void random_reset();
void timerCallback(const ros::TimerEvent&);
bool serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "fake_rival");
    ros::NodeHandle nh;
    ros::Publisher fake_rival_pub = nh.advertise<nav_msgs::Odometry>("/rival/final_pose", 100);
    ros::ServiceServer reset_service = nh.advertiseService("/rival/reset", serviceCallback);
    ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCallback);
    ros::Rate loop_rate(50);

    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.w = 0.0;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.angular.z = 0.0;

    goal.x = 0.0;
    goal.y = 0.0;
    goal.z = 0.0;

    nh.getParam("/enable", enable);
    nh.getParam("/random_mode", random_mode);
    nh.getParam("/velocity", vel);
    if(!enable){
        ROS_ERROR("Fake Rival NOT Enabled");
        return 0;
    }

    if(!random_mode)
    {
        if(nh.getParam("footprint_x", footprint_x) && nh.getParam("footprint_y", footprint_y))
        {
            odom.pose.pose.position.x = footprint_x[0];
            odom.pose.pose.position.y = footprint_y[0];
            goal.x = footprint_x[0];
            goal.y = footprint_y[0];
            size = footprint_x.size();
        }
        else
        {
            ROS_ERROR("FAIL TO GET FOOTPRINT");
        }
    }


    // Main loop to publish messages
    while (ros::ok())
    {
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        fake_rival_pub.publish(odom);
        ROS_ERROR("%f, %f", footprint_x[0], footprint_y[0]);
        ROS_ERROR("%f, %f,, %d", footprint_x[size-1], footprint_y[size-1], size);
        ROS_ERROR("%f, %f, %d", footprint_x[point], footprint_y[point], point);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


/**************************************************************************************************************************/
void move()
{
    dx = 0.02* vel* ((goal.x-odom.pose.pose.position.x)/sqrt(pow(odom.pose.pose.position.x-goal.x, 2) + pow(odom.pose.pose.position.y-goal.y, 2)));
    dy = 0.02* vel* ((goal.y-odom.pose.pose.position.y)/sqrt(pow(odom.pose.pose.position.x-goal.x, 2) + pow(odom.pose.pose.position.y-goal.y, 2)));
    if(abs(odom.pose.pose.position.x-goal.x) > 0.2)
        odom.pose.pose.position.x += dx;
    if(abs(odom.pose.pose.position.y-goal.y) > 0.2)    
        odom.pose.pose.position.y += dy;
}

void random_point()
{
    double now = ros::Time::now().toSec();
    goal.x = abs(((int)now*532)%275) *0.01 + 0.125;
    goal.y = abs(((int)now*762)%175) *0.01 + 0.125;
    vel = abs(((int)now*671)%100) *0.001+0.25;
}

bool order = true;
void timerCallback(const ros::TimerEvent&)
{
    if(random_mode)
    {
        if(abs(odom.pose.pose.position.x-goal.x) < 0.2 && abs(odom.pose.pose.position.y-goal.y) < 0.2)
        {
            random_point();
        }
    }
    else
    {
        if(abs(odom.pose.pose.position.x-goal.x) < 0.2 && abs(odom.pose.pose.position.y-goal.y) < 0.2)
        {
            goal.x = footprint_x[point];
            goal.y = footprint_y[point];
            if(point < size-1 && order){
                point ++;
            }else if(point == size-1 && order){
                order = false;
            }else if(!order && point > 0){
                point --;
            }
            else if(point == 0){
                order = true;
            }
        }
    }

    move();
}

bool serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    random_reset();
    return true;
}

void random_reset(){
    double now = ros::Time::now().toSec();
    odom.pose.pose.position.x = abs(((int)now*532)%275) *0.01 + 0.125;
    odom.pose.pose.position.y = abs(((int)now*762)%175) *0.01 + 0.125;
    point = 0;
    // goal.x = odom.pose.pose.position.x;
    // goal.y = odom.pose.pose.position.y;
}