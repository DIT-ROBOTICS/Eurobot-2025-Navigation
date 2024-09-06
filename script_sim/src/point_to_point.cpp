#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <navigation_main/NavMissionAction.h>
#include <actionlib/client/simple_action_client.h>

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Char.h"
#include "yaml-cpp/yaml.h"

using namespace std;

bool check = true;


navigation_main::NavMissionGoal goal_point;
YAML::Node pathConfig;
int path_length;
tf2::Quaternion qt;

// void Check(const std_msgs::Bool::ConstPtr& msg) {
//     check = true;
//     if (!msg->data) {    ROS_INFO("[Dock Tracker]: Wrong format of frame id: '%s'", data.header.frame_id.c_str());
//         ROS_INFO("script_sim: cannot arrive the goal, next point!");
//     }
// }

void Check(const std_msgs::Char::ConstPtr& msg) {  
    if (msg->data == 2) {
        ROS_INFO("script_sim: cannot arrive the goal, next point!");
    }else if(msg->data == 1){
        ROS_INFO("script_sim: ARRIVED!!!!");
        check = true;
        
    }
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const navigation_main::NavMissionResultConstPtr& result)
{
    ROS_INFO_STREAM("script_sim: Finished");
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const navigation_main::NavMissionFeedbackConstPtr& feedback) 
{
  if (feedback->progress.data == 0) {
    ROS_INFO_STREAM("script_sim: Goal Reached!");
    check = true;
  }
  else if (feedback->progress.data == 2) {
    ROS_INFO_STREAM("script_sim: Goal Not Reached!");
    check = true;
  }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_to_point");
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<navigation_main::NavMissionAction> point_ac("navigation_main", true);
    ROS_INFO_STREAM("script_sim: Waiting for action server to start.");
    point_ac.waitForServer();

    string PathName(argv[1]);
    string PubName(argv[2]);
    string SubName(argv[3]);

    pathConfig = YAML::LoadFile(PathName);
    // ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(PubName, 100);
    // ros::Subscriber sub = nh.subscribe(SubName, 100, Check);

    ros::Duration(2).sleep();

    ros::Rate loop_rate(100);

    for (auto goal : pathConfig) {
        if (!ros::ok()) {
            break;
        }
        string goal_type = goal["xyz"][0].as<string>();
        // std::cout << goal_type << std::endl;
        if(goal_type == "path") {
            goal_point.nav_goal.twist.angular.x = 0;
        }else if(goal_type == "dock"){
            goal_point.nav_goal.twist.angular.x = 1;
        }
        goal_point.nav_goal.twist.linear.x = goal["xyz"][1].as<double>();
        goal_point.nav_goal.twist.linear.y = goal["xyz"][2].as<double>();
        goal_point.nav_goal.twist.linear.z = goal["xyz"][3].as<double>();
        point_ac.sendGoal(goal_point, &doneCb, &activeCb, &feedbackCb);

        // pub.publish(goal_point.nav_goal);    
        check = false;
        while (!check) {
            ros::spinOnce();
            // loop_rate.sleep();
        }
    }

    return 0;
}