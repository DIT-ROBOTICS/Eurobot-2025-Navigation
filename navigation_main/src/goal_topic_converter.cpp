#include <ros/ros.h>
#include <navigation_main/NavMissionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>



double x, y, z;
bool new_goal = false;
int goal_type = 0;  // 0 for path, 1 for dock ,2 for dock2
void rvizPathGoalCB(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    ROS_INFO("Received path goal from RViz");
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);   // transform quaternion to angle

    z = yaw;
    new_goal = true;
    goal_type = 0;
}

void rvizDock10GoalCB(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    ROS_INFO("Received dock10 goal from RViz");
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);   // transform quaternion to angle

    z = yaw;
    new_goal = true;
    goal_type = 10;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_topic_converter");
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<navigation_main::NavMissionAction> rviz_ac("navigation_main", true);
    ros::Subscriber goal_path_sub = nh.subscribe("rviz_path_goal", 1, rvizPathGoalCB);
    ros::Subscriber goal_dock10_sub = nh.subscribe("rviz_dock_goal", 1, rvizDock10GoalCB);

    rviz_ac.waitForServer();
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        if (new_goal) {
            navigation_main::NavMissionGoal rviz_goal;
            rviz_goal.nav_goal.header.stamp = ros::Time::now();
            rviz_goal.nav_goal.twist.linear.x = x;
            rviz_goal.nav_goal.twist.linear.y = y; 
            rviz_goal.nav_goal.twist.linear.z = z;
            rviz_goal.nav_goal.twist.angular.x = goal_type;
            rviz_ac.sendGoal(rviz_goal);
            ROS_INFO_STREAM(ros::this_node::getName() << " : Sent goal to navigation_main");
            new_goal = false;
        }
        rate.sleep();
    }
}