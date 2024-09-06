#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <navigation_main/NavMissionAction.h>
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_main");
    actionlib::SimpleActionClient<navigation_main::NavMissionAction> ac("navigation_main", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    while (ros::ok()) {
        navigation_main::NavMissionGoal goal;
        // get input from user
        ROS_INFO("Enter x, y, z, and mission type (0 for path, 1 for dock): ");
        std::cin >> goal.nav_goal.twist.linear.x >> goal.nav_goal.twist.linear.y >> goal.nav_goal.twist.linear.z >> goal.nav_goal.twist.angular.x;
        ac.sendGoal(goal);
        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached.");
        } else {
            ROS_INFO("Failed to reach goal.");
        }
    }
}