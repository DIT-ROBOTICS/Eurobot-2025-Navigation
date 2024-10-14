#ifndef _NAVIGATION_MAIN_H_
#define _NAVIGATION_MAIN_H_

// ROS basic
#include "ros/ros.h"

// ROS msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "obstacle_detector/CircleObstacle.h"
#include "obstacle_detector/Obstacles.h"

// ROS srvs
#include "std_srvs/Empty.h"
#include "navigation_main/ready.h"

// ROS actionlib
#include <actionlib/server/simple_action_server.h>
#include <navigation_msgs/NavMissionAction.h>

// ROS param
// #include <dynamic_reconfigure/server.h>

// #include "navigation_main/navigation_main_paramConfig.h"

// Other lib
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

class Navigation_Main {
   public:
    Navigation_Main(ros::NodeHandle *nh_global, ros::NodeHandle *nh_local, std::string node_name);
    ~Navigation_Main();

    double GetUpdateFrequency();

    void Loop();

    // Actionlib
    void goalCB();
    void preemptCB();

   private:
    // Typedef
    // typedef struct {
    //     double x;
    //     double y;
    // } Point;

    bool UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void Initialize_Node();

    // void SetDynamicReconfigure();

    // void SetTimeout(geometry_msgs::Pose poseGoal);
    bool isTimeout();
    void FailToGoal(int fail_reason);

    // Callback functions
    // void Robot_Odom_type0_CB(const nav_msgs::Odometry::ConstPtr &msg);
    // void Robot_Odom_type1_CB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    // void Robot_Obs_Odom_type0_CB(const nav_msgs::Odometry::ConstPtr &msg);
    // void Robot_Obs_Odom_type1_CB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    // void Rival1_Odom_CB(const nav_msgs::Odometry::ConstPtr &msg);
    // void Rival2_Odom_CB(const nav_msgs::Odometry::ConstPtr &msg);
    // void RivalObstacles_CB(const obstacle_detector::Obstacles::ConstPtr &msg);
    void PathExecCmdVel_CB(const geometry_msgs::Twist::ConstPtr &msg);
    void DockExecCmdVel_CB(const geometry_msgs::Twist::ConstPtr &msg);
    void PathStatus_CB(const std_msgs::Char::ConstPtr &msg);
    void DockStatus_CB(const std_msgs::Char::ConstPtr &msg);
    // void RobotMissionState_CB(const std_msgs::Char::ConstPtr &msg);
    // void MainMission_CB(const geometry_msgs::PoseStamped::ConstPtr &msg);
    // void DynamicParam_CB(const navigation_main::navigation_main_paramConfig &config, uint32_t level);
    // void ResendGoal_CB(const ros::TimerEvent &event);

    // Other functions
    // bool isCloseToOtherRobots();
    // double Distance_Between_A_and_B(const geometry_msgs::Pose poseA, const geometry_msgs::Pose poseB);
    // double Distance_Between_A_and_B(const Point pointA, const Point pointB);
    // void Check_Odom_CB_Timeout();
    // bool isInBlockArea();
    // bool isPointInPolygon(const Point point, const Point polygon[], int polygon_size);
    // void HandleGoalUnreachable(bool reachable);
    bool ReadySignal_CB(navigation_main::ready::Request &req, navigation_main::ready::Response &res);

    // NodeHandle
    ros::NodeHandle *nh_local_;
    ros::NodeHandle *nh_global_;

    // Subscriber
    // ros::Subscriber robot_odom_sub_;
    // ros::Subscriber robot_obs_odom_sub_;
    // ros::Subscriber rival_odom_sub_[2];
    // ros::Subscriber rival_obstacle_sub_;
    ros::Subscriber path_exec_cmd_vel_sub_;
    ros::Subscriber dock_exec_cmd_vel_sub_;
    ros::Subscriber path_status_sub_;
    ros::Subscriber dock_status_sub_;
    // ros::Subscriber robot_mission_state_sub_;
    // ros::Subscriber main_mission_state_sub_;

    // Publisher
    // ros::Publisher main_mission_state_pub_;
    ros::Publisher path_exec_goal_pub_;
    ros::Publisher dock_exec_goal_pub_;
    ros::Publisher dock10_exec_goal_pub_;
    ros::Publisher cmd_vel_pub_;

    // Server
    ros::ServiceServer ready_server;
    // ros::ServiceServer param_srv_;

    // Actionlib
    actionlib::SimpleActionServer<navigation_msgs::NavMissionAction> mission_as_;
    navigation_msgs::NavMissionFeedback mission_feedback_;
    navigation_msgs::NavMissionResult mission_result_;


    // Parameters

    // bool param_active_; 
    // bool param_publish_;
    // bool param_update_params_;
    // bool param_use_dynamic_reconfigure_;

    // int param_resend_goal_time_;

    double param_update_frequency_;
    int param_replan_threshold_;
    int param_replan_sleep_duration_;
    double param_path_timeout_;
    double param_dock_timeout_;
    double param_status_update_timeout_;
    // double param_timeout_a_;
    // double param_timeout_b_;
    // double param_timeout_min_;
    // double param_timeout_max_;
    // double param_resend_goal_frequency_;
    // double param_goal_stop_distance_;
    // double param_rival_stop_distance_;
    // double param_odom_timeout_;
    // double param_block_mode_distance_a_;
    // double param_block_mode_distance_b_;
    // double param_block_mode_distance_c_;

    std::string param_node_name_;
    // std::string param_robot_odom_topic_;
    // std::string param_robot_obs_odom_topic_;
    // std::string param_rival_odom_topic_[2];
    // std::string param_rival_obstacle_topic_;
    // std::string param_robot_mission_state_topic_;
    std::string param_path_exec_goal_topic_;
    std::string param_dock_exec_goal_topic_;
    std::string param_dock10_exec_goal_topic_;
    std::string param_cmd_vel_topic_;
    std::string param_path_exec_cmd_vel_topic_;
    std::string param_dock_exec_cmd_vel_topic_;
    std::string param_path_exec_status_topic_;
    std::string param_dock_exec_status_topic_;
    // std::string param_main_mission_topic_;
    // std::string param_main_mission_state_topic_;
    std::string param_tf_prefix_;

    // Variables
    // bool is_mission_start_;
    // bool is_reach_goal_;
    // bool is_robot_obs_odom_timeout_;
    // bool is_rival1_odom_timeout_;
    // bool is_rival2_odom_timeout_;
    // bool is_rival_obstacle_timeout_;
    double cur_timeout_;
    geometry_msgs::PoseStamped goal_msg_;
    char mission_type_;

    enum MISSION_TYPE {
        // Do mission
        PATH_EXEC = 0,
        DOCK_EXEC = 1,
        SLOW_DOCK_EXEC = 2,

        IDLE = 3,
        DOCK10_EXEC = 10,
        DOCK10_ESCAPE = 11,
    };
    MISSION_TYPE mission_status_;

    // enum class ODOM_CALLBACK_TYPE {
    //     nav_msgs_Odometry = 0,
    //     geometry_msgs_PoseWithCovarianceStamped = 1
    // };
    // ODOM_CALLBACK_TYPE odom_type_;

    // Block Mode
    // const Point CHERRY_DISPENSER[2] = {{0.3, 1.0},
    //                                    {2.7, 1.0}};
    // const double MAP_WIDTH = 3.0;   // x axis
    // const double MAP_HEIGHT = 2.0;  // y axis

    // Timeout
    ros::Time start_time_;
    ros::Time status_update_time_;
    // ros::Timer resend_goal_timer_;
    // int resend_goal_time_;

    int cur_replan_cnt_;

    // ros::Time robot_obs_odom_time_;
    ros::Time rival_odom_time_[1];
    // ros::Time rival_obstacle_time_;

    // Robot Odometry
    geometry_msgs::Vector3 robot_goal_;
    geometry_msgs::Pose robot_odom_;
    // geometry_msgs::Pose robot_obs_odom_;
    geometry_msgs::Pose rival_odom_[1];
    // obstacle_detector::Obstacles rival_obstacles_;
    geometry_msgs::Twist robot_cmd_vel_;
    char robot_status_; // 0: running, 1: success, 2: fail
};


#endif