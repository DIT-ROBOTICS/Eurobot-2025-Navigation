#include "navigation_main/navigation_main.h"


Navigation_Main::Navigation_Main(ros::NodeHandle *nh_global, ros::NodeHandle *nh_local, std::string node_name) :
  mission_as_(*nh_global, node_name, false) {
    nh_global_ = nh_global;
    nh_local_ = nh_local;
    param_node_name_ = node_name;
    robot_goal_.x = robot_goal_.y = robot_goal_.z = -100.0;
    robot_odom_.position.x = robot_odom_.position.y = -100.0;
    rival_odom_[0].position.x = rival_odom_[0].position.y = -100.0;
    cur_timeout_ = 0;
    rival_odom_time_[0] = ros::Time::now();
    mission_status_ = MISSION_TYPE::IDLE;
    goal_msg_.pose.position.x = goal_msg_.pose.position.y = -1;

    std_srvs::Empty empty_srv;
    if (this->UpdateParams(empty_srv.request, empty_srv.response)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Parameters Init Successfully.");
    } else {
        ROS_WARN_STREAM("[" << param_node_name_ << "]: " << "Parameters Init Failed.");
    }
    Initialize_Node();

    mission_as_.registerGoalCallback(boost::bind(&Navigation_Main::goalCB, this));
    mission_as_.registerPreemptCallback(boost::bind(&Navigation_Main::preemptCB, this));
    mission_as_.start();
    ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Action Server Started.");
}

Navigation_Main::~Navigation_Main() {
    // shutdown subscriber, publisher, and service.
    mission_as_.shutdown();
    path_exec_goal_pub_.shutdown();
    dock_exec_goal_pub_.shutdown();
    path_exec_cmd_vel_sub_.shutdown();
    dock_exec_cmd_vel_sub_.shutdown();
    cmd_vel_pub_.shutdown();
    path_status_sub_.shutdown();
    dock_status_sub_.shutdown();
}

void Navigation_Main::goalCB() {
    // accept the new goal
    // if (mission_status_ != MISSION_TYPE::IDLE) {
    //     ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Previous Mission Being Preempted.");
    //     FailToGoal(0);
    // }
    if (mission_as_.isNewGoalAvailable()) {
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "New Mission Received.");
    }
    else {
        ROS_WARN_STREAM("[" << param_node_name_ << "]: " << "No New Mission Received.");
        // return ;
    }
    /*if (goal_msg_.pose.position.x == -1 && goal_msg_.pose.position.y == -1) {
        navigation_msgs::NavMissionGoal new_goal = *mission_as_.acceptNewGoal();
        robot_goal_ = new_goal.nav_goal.twist.linear;
        goal_msg_.header.stamp = ros::Time::now();
        goal_msg_.header.frame_id = param_tf_prefix_ + "/map";
        goal_msg_.pose.position.x = robot_goal_.x;
        goal_msg_.pose.position.y = robot_goal_.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, robot_goal_.z);
        goal_msg_.pose.orientation = tf2::toMsg(q);
        mission_type_ = new_goal.nav_goal.twist.angular.x;
        // start_time_ = ros::Time::now();
        cur_replan_cnt_ = 0;
    }*/
    if (goal_msg_.pose.position.x == -1 && goal_msg_.pose.position.y == -1) {
        navigation_msgs::NavMissionGoal new_goal = *mission_as_.acceptNewGoal();
        goal_msg_.header.stamp = ros::Time::now();
        goal_msg_.header.frame_id = param_tf_prefix_ + "/map";
        goal_msg_.pose.position.x = new_goal.nav_goal.twist.linear.x;
        goal_msg_.pose.position.y = new_goal.nav_goal.twist.linear.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, new_goal.nav_goal.twist.linear.z);
        goal_msg_.pose.orientation = tf2::toMsg(q);
        mission_type_ = new_goal.nav_goal.twist.angular.x;
        // start_time_ = ros::Time::now();
        cur_replan_cnt_ = 0;
    }
    if (mission_type_ == 0) {
        mission_status_ = MISSION_TYPE::PATH_EXEC;
        cur_timeout_ = param_path_timeout_;
        path_exec_goal_pub_.publish(goal_msg_);
    }
    else if (mission_type_ == 1) {
        goal_msg_.header.frame_id = "dock";
        mission_status_ = MISSION_TYPE::DOCK_EXEC;
        cur_timeout_ = param_dock_timeout_;
        dock_exec_goal_pub_.publish(goal_msg_);
    }
    else if (mission_type_ == 2) {
        goal_msg_.header.frame_id = "slow-dock";
        
        mission_status_ = MISSION_TYPE::SLOW_DOCK_EXEC;
        cur_timeout_ = param_dock_timeout_;
        dock_exec_goal_pub_.publish(goal_msg_);
    }
    else if (mission_type_ == 10) {
        goal_msg_.header.frame_id = "dock10";
        ROS_INFO("i m here");
        mission_status_ = MISSION_TYPE::DOCK10_EXEC;
        cur_timeout_ = param_dock_timeout_;
        dock10_exec_goal_pub_.publish(goal_msg_);
    }
    else if (mission_type_ == 11) {
        goal_msg_.header.frame_id = "dock10_escape";
        ROS_INFO("call for escape");
        mission_status_ = MISSION_TYPE::DOCK10_ESCAPE;
        cur_timeout_ = param_dock_timeout_;
        dock10_exec_goal_pub_.publish(goal_msg_);
    }
    
}

void Navigation_Main::preemptCB() {
    // set the action state to preempted
    ROS_INFO_STREAM(param_node_name_ << " preempted " << mission_as_.isActive());


    geometry_msgs::PoseStamped stop_signal;
    stop_signal.header.stamp = ros::Time::now();
    stop_signal.pose.position.x = stop_signal.pose.position.y = -1;
    goal_msg_.pose.position.x = goal_msg_.pose.position.y = -1;
    path_exec_goal_pub_.publish(stop_signal);
    dock_exec_goal_pub_.publish(stop_signal);
    mission_status_ = MISSION_TYPE::IDLE;

    robot_cmd_vel_.linear.x = robot_cmd_vel_.linear.y = robot_cmd_vel_.angular.z = 0;
    cmd_vel_pub_.publish(robot_cmd_vel_);
    mission_feedback_.progress.data = 2;
    mission_result_.outcome.data = 0;
    mission_as_.setPreempted(mission_result_);
}



double Navigation_Main::GetUpdateFrequency() {
    return this->param_update_frequency_;
}

void Navigation_Main::Loop() {
    if (mission_status_ == MISSION_TYPE::IDLE) {
        robot_cmd_vel_.linear.x = robot_cmd_vel_.linear.y = robot_cmd_vel_.angular.z = 0;
        cmd_vel_pub_.publish(robot_cmd_vel_);
        return ;
    }
    // if (mission_status_ != MISSION_TYPE::IDLE && isTimeout()) {
    //     FailToGoal(0);
    //     return ;
    // }

    // Check status update timeout
    // if ((ros::Time::now() - status_update_time_).toSec() > param_status_update_timeout_) {
    //     ROS_WARN_STREAM("[" << param_node_name_ << "]: " << "Status Update Timeout.");
    //     double time_remain = cur_timeout_ - (ros::Time::now() - start_time_).toSec();
    //     if (time_remain > 3) {
    //         goalCB();
    //     }
    //     else {
    //         ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Failed " << "Reason: " << "Executor Not Responding.");
    //         FailToGoal(1);
    //         return ;
    //     }
    //     return ;
    // }

    mission_feedback_.progress.data = 1;
    if (robot_status_ == 1) {
        // path/dock success
        robot_status_ = 0;
        mission_feedback_.progress.data = 0;
        mission_result_.outcome.data = 1;
        mission_status_ = MISSION_TYPE::IDLE;
        robot_cmd_vel_.linear.x = robot_cmd_vel_.linear.y = robot_cmd_vel_.angular.z = 0;
        mission_as_.setSucceeded(mission_result_);
        goal_msg_.pose.position.x = goal_msg_.pose.position.y = -1;
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Accomplished."); 
    }
    else if (robot_status_ == 2) {
        robot_status_ = 0;
        // path/dock failed
        // Try for n times, if still failed, then it should be goal blocked.
        // double time_remain = cur_timeout_ - (ros::Time::now() - start_time_).toSec();
        // ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Time Remain: " << time_remain << "s");
        cur_replan_cnt_++;
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Replan Chance Remain: " << param_replan_threshold_ - cur_replan_cnt_);
        if (param_replan_threshold_ - cur_replan_cnt_ > 0) {
            robot_cmd_vel_.linear.x = robot_cmd_vel_.linear.y = robot_cmd_vel_.angular.z = 0;
            cmd_vel_pub_.publish(robot_cmd_vel_);    
            ros::Duration(param_replan_sleep_duration_).sleep();
            goalCB();
        }
        else {
            ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Failed " << "Reason: " << "Goal Blocked");
            FailToGoal(1);
            return ;
        }
    }
    mission_as_.publishFeedback(mission_feedback_);
    cmd_vel_pub_.publish(robot_cmd_vel_);    
}

bool Navigation_Main::isTimeout() {
    if (cur_timeout_ == 0) {
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Timeout is not set.");
        return false;
    }
    return ((ros::Time::now() - start_time_).toSec() > cur_timeout_);
}

bool Navigation_Main::UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    // Update parameters
    if (this->nh_local_->param<double>("update_frequency", this->param_update_frequency_, 5.0))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "update_frequency =>" << param_update_frequency_);
    if (this->nh_local_->param<double>("path_timeout", this->param_path_timeout_, 12.0))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "timeout =>" << param_path_timeout_);
    if (this->nh_local_->param<int>("replan_threshold", this->param_replan_threshold_, 5))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "replan_threshold =>" << param_replan_threshold_);
    if (this->nh_local_->param<int>("replan_sleep_duration", this->param_replan_sleep_duration_, 1))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "replan_sleep_duration =>" << param_replan_sleep_duration_);
    if (this->nh_local_->param<double>("dock_timeout", this->param_dock_timeout_, 5.0))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "timeout =>" << param_path_timeout_);
    if (this->nh_local_->param<std::string>("path_exec_goal_topic", this->param_path_exec_goal_topic_, "path_exec_goal"))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "path_exec_goal_topic =>" << param_path_exec_goal_topic_);
    if (this->nh_local_->param<std::string>("dock_exec_goal_topic", this->param_dock_exec_goal_topic_, "dock_exec_goal"))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "dock_exec_goal_topic =>" << param_dock_exec_goal_topic_);
    if (this->nh_local_->param<std::string>("dock10_exec_goal_topic", this->param_dock10_exec_goal_topic_, "dock10_exec_goal"))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "dock10_exec_goal_topic =>" << param_dock10_exec_goal_topic_);    
    if (this->nh_local_->param<std::string>("tf_prefix", this->param_tf_prefix_, "robot"))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "tf_prefix =>" << param_tf_prefix_);
    if (this->nh_local_->param<std::string>("path_exec_cmd_vel_topic", this->param_path_exec_cmd_vel_topic_, "path_exec_cmd_vel"))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "path_exec_cmd_vel_topic =>" << param_path_exec_cmd_vel_topic_);
    if (this->nh_local_->param<std::string>("dock_exec_cmd_vel_topic", this->param_dock_exec_cmd_vel_topic_, "dock_exec_cmd_vel"))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "dock_exec_cmd_vel_topic =>" << param_dock_exec_cmd_vel_topic_);
    if (this->nh_local_->param<std::string>("cmd_vel_topic", this->param_cmd_vel_topic_, "cmd_vel"))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "cmd_vel_topic =>" << param_cmd_vel_topic_);
    if (this->nh_local_->param<std::string>("path_exec_status_topic", this->param_path_exec_status_topic_, "path_exec_status"))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "path_exec_status_topic =>" << param_path_exec_status_topic_);
    if (this->nh_local_->param<std::string>("dock_exec_status_topic", this->param_dock_exec_status_topic_, "dock_exec_status"))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "dock_exec_status_topic =>" << param_dock_exec_status_topic_);
    if (this->nh_local_->param<double>("status_update_timeout", this->param_status_update_timeout_, 1.0))
        ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "status_update_timeout =>" << param_status_update_timeout_);
    return true;
}

void Navigation_Main::Initialize_Node() {
    path_exec_goal_pub_ = nh_global_->advertise<geometry_msgs::PoseStamped>(param_path_exec_goal_topic_, 100);
    dock_exec_goal_pub_ = nh_global_->advertise<geometry_msgs::PoseStamped>(param_dock_exec_goal_topic_, 100);
    dock10_exec_goal_pub_ = nh_global_->advertise<geometry_msgs::PoseStamped>(param_dock10_exec_goal_topic_, 100);
    path_exec_cmd_vel_sub_ = nh_global_->subscribe(param_path_exec_cmd_vel_topic_, 100, &Navigation_Main::PathExecCmdVel_CB, this);
    dock_exec_cmd_vel_sub_ = nh_global_->subscribe(param_dock_exec_cmd_vel_topic_, 100, &Navigation_Main::DockExecCmdVel_CB, this);
    cmd_vel_pub_ = nh_global_->advertise<geometry_msgs::Twist>(param_cmd_vel_topic_, 100);
    path_status_sub_ = nh_global_->subscribe(param_path_exec_status_topic_, 100, &Navigation_Main::PathStatus_CB, this);
    dock_status_sub_ = nh_global_->subscribe(param_dock_exec_status_topic_, 100, &Navigation_Main::DockStatus_CB, this);
    ready_server = nh_global_->advertiseService("startup/ready_signal_feedback", &Navigation_Main::ReadySignal_CB, this); 
}

void Navigation_Main::FailToGoal(int fail_reason) {
    ROS_INFO_STREAM(param_node_name_ << " failtogoal " << mission_as_.isActive());
    geometry_msgs::PoseStamped stop_signal;
    stop_signal.header.stamp = ros::Time::now();
    stop_signal.pose.position.x = stop_signal.pose.position.y = -1;
    goal_msg_.pose.position.x = goal_msg_.pose.position.y = -1;

    geometry_msgs::PoseStamped escape_signal;
    escape_signal.header.stamp = ros::Time::now();
    escape_signal.pose.position.x = escape_signal.pose.position.y = -2;
    switch(fail_reason) {
        case 0:
            // Timeout
            if (mission_status_ == MISSION_TYPE::PATH_EXEC) {
                ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Failed " << "Reason: " << "Path Timeout");
                path_exec_goal_pub_.publish(stop_signal);
            }
            else if (mission_status_ == MISSION_TYPE::DOCK_EXEC) {
                ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Failed " << "Reason: " << "Dock Timeout");
                dock_exec_goal_pub_.publish(stop_signal);
            }
            else if (mission_status_ == MISSION_TYPE::SLOW_DOCK_EXEC) {
                ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Failed " << "Reason: " << "Slow Dock Timeout");
                dock_exec_goal_pub_.publish(stop_signal);
            }
            else if(mission_status_ == MISSION_TYPE::DOCK10_EXEC) {
                ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Failed " << "Reason: " << "Dock10 Timeout");
                dock10_exec_goal_pub_.publish(escape_signal);
            }
            else {
                ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Failed " << "Reason: " << "Unknown");
            }
            break;
        case 1:
            // Goal Blocked
            if (mission_status_ == MISSION_TYPE::PATH_EXEC) {
                ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Failed " << "Reason: " << "Goal Blocked");
                path_exec_goal_pub_.publish(stop_signal);
            }
            else if (mission_status_ == MISSION_TYPE::DOCK_EXEC) {
                ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Failed " << "Reason: " << "Goal Blocked");
                dock_exec_goal_pub_.publish(stop_signal);
            }
            else if (mission_status_ == MISSION_TYPE::SLOW_DOCK_EXEC) {
                ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Failed " << "Reason: " << "Goal Blocked");
                dock_exec_goal_pub_.publish(stop_signal);
            }
            else if(mission_status_ == MISSION_TYPE::DOCK10_EXEC) {
                ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Failed " << "Reason: " << "Dock10 Timeout");
                dock10_exec_goal_pub_.publish(escape_signal);
            }
            else {
                ROS_INFO_STREAM("[" << param_node_name_ << "]: " << "Mission Failed " << "Reason: " << "Unknown");
            }
            break;
    }
    mission_status_ = MISSION_TYPE::IDLE;

    robot_cmd_vel_.linear.x = robot_cmd_vel_.linear.y = robot_cmd_vel_.angular.z = 0;
    cmd_vel_pub_.publish(robot_cmd_vel_);
    mission_feedback_.progress.data = 2;
    mission_result_.outcome.data = 0;

    mission_as_.setAborted(mission_result_);
    mission_as_.publishFeedback(mission_feedback_);
}


void Navigation_Main::PathExecCmdVel_CB(const geometry_msgs::Twist::ConstPtr &msg) {
    if (mission_status_ == MISSION_TYPE::PATH_EXEC) {
        robot_cmd_vel_ = *msg;
    }
}

void Navigation_Main::DockExecCmdVel_CB(const geometry_msgs::Twist::ConstPtr &msg) {
    if (mission_status_ == MISSION_TYPE::DOCK_EXEC || mission_status_ == MISSION_TYPE::SLOW_DOCK_EXEC|| mission_status_ == MISSION_TYPE::DOCK10_EXEC) {
        robot_cmd_vel_ = *msg;
    }
}

void Navigation_Main::PathStatus_CB(const std_msgs::Char::ConstPtr &msg) {
    if (mission_status_ == MISSION_TYPE::PATH_EXEC) {
        robot_status_ = msg->data;
        status_update_time_ = ros::Time::now();
    }
}

void Navigation_Main::DockStatus_CB(const std_msgs::Char::ConstPtr &msg) {
    if (mission_status_ == MISSION_TYPE::DOCK_EXEC || mission_status_ == MISSION_TYPE::SLOW_DOCK_EXEC) {
        robot_status_ = msg->data;
        status_update_time_ = ros::Time::now();
    }
}

bool Navigation_Main::ReadySignal_CB(navigation_main::ready::Request &req, navigation_main::ready::Response &res) {
    if (req.group == 4) {
        res.success = true;
    }
    return true;
}