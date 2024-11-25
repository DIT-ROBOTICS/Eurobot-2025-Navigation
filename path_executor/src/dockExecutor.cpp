#include "dockExecutor.h"

DockExecutor::DockExecutor(ros::NodeHandle& nh, ros::NodeHandle& nh_local) {
    nh_ = nh;
    nh_local_ = nh_local;
    std_srvs::Empty empt;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("params", &DockExecutor::initializeParams, this);
    initializeParams(empt.request, empt.response);
    initialize();
}

DockExecutor::~DockExecutor() {
    nh_local_.deleteParam("active");
    nh_local_.deleteParam("control_frequency");
    nh_local_.deleteParam(robot_type_ + "/linear_max_velocity");
    nh_local_.deleteParam(robot_type_ + "/profile_percent");
    nh_local_.deleteParam(robot_type_ + "/stop_tolerance");
    nh_local_.deleteParam(robot_type_ + "/pose_type");
    nh_local_.deleteParam(robot_type_ + "/angular_kp");
    nh_local_.deleteParam(robot_type_ + "/fast_angular_max_vel");
    nh_local_.deleteParam(robot_type_ + "/fast_angular_kp");
    // nh_local_.deleteParam("rival_tolerance");
}

void DockExecutor::initialize() {
    // zeroing the arrays
    goal_[0] = 0.0;
    goal_[1] = 0.0;
    goal_[2] = 0.0;

    pose_[0] = 0.0;
    pose_[1] = 0.0;
    pose_[2] = 0.0;

    vel_[0] = 0.0;
    vel_[1] = 0.0;
    vel_[2] = 0.0;

    dock_dist_ = 0.05;
    if_get_goal_ = false;
    count_dock_dist_ = false;
    // rival_dist_ = 10.0;
    dist_ = 0.0;
    vibrate_time_now_ = 0;
    // linear_max_vel_ = 0.0;
    // angular_max_vel_ = 0.0;

    mode_ = MODE::IDLE;

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &DockExecutor::timerCB, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency_), false);
    timer_.start();

    rival_dist_ = 100;
    mini_dist_ = 100;

    debounce = 0;
    debounce_ang = 0;

    rival_x_ = 0;
    rival_y_ = 0;
}

bool DockExecutor::initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    // Load parameter
    bool get_param_ok = true;
    bool prev_active = p_active_;
    get_param_ok = nh_local_.param<bool>("active", p_active_, true);
    // get_param_ok = nh_local_.param<string>("", _, "");
    get_param_ok &= nh_local_.param<double>("control_frequency", control_frequency_, 50);
    get_param_ok &= nh_local_.param<std::string>("robot_type", robot_type_, "differential");
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_max_velocity", linear_max_vel_, 0.3);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_kp", linear_kp_, 2.0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_kd", linear_kd_, 1.0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_acceleration", linear_acceleration_, 0.15);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/slow_linear_max_velocity", slow_linear_max_vel_, 0.7);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/slow_linear_kp", slow_linear_kp_, 3.5);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/slow_linear_acceleration", slow_acceleration_, 1.0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_min_velocity", linear_min_vel_, 0.1); 

    get_param_ok &= nh_local_.param<double>(robot_type_ + "/angular_max_velocity", angular_max_vel_, 1);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/angular_min_velocity", angular_min_vel_, 0.3);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/angular_kp", angular_kp_, 2.5);
    get_param_ok &= nh_local_.param<double>("angular_velocity_divider", div_, 3.0);                               // Not in yaml
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/fast_angular_max_vel", fast_angular_max_vel_, 3.0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/fast_angular_kp", fast_angular_kp_, 2.0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/profile_percent", profile_percent_, 0.2);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/point_stop_tolerance", tolerance_, 0.005);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/angle_stop_tolerance", ang_tolerance_, 0.01);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/first_angle_stop_tolerance", first_ang_tolerance_, 0.01);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/move_fail_tolerance", move_fail_tolerance_, 1.05);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/move_over_tolerance", move_over_tolerance_, 0.02);
    get_param_ok &= nh_local_.param<double>("first_rotate_range", first_rotate_range_, 0.1);                     // Not in yaml
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/angular_adjust_kp", angular_adjust_kp_, 0.8);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/angular_adjust_limit", angular_adjust_limit_, 0.1);


    get_param_ok &= nh_local_.param<int>("pose_type", pose_type_, 0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/rival_tolerance", rival_tolerance_, 0.40);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "rival_parallel_tolerance", rival_parallel_tolerance_, 0.4);

    if (p_active_ != prev_active) {
        if (p_active_) {
            goal_sub_ = nh_.subscribe("dock_exec_goal", 50, &DockExecutor::goalCB, this);
            if (pose_type_ == 0) {
                pose_sub_ = nh_.subscribe("odom", 50, &DockExecutor::poseCB_Odometry, this);
            } else if (pose_type_ == 1) {
                pose_sub_ = nh_.subscribe("final_pose", 50, &DockExecutor::poseCB_Odometry, this);
            }
            rival_sub_ = nh_.subscribe("/rival/final_pose", 50, &DockExecutor::rivalCB_Odometry, this);
            // rival2_sub_ = nh_.subscribe("/rival2/odom", 50, &DockExecutor::rivalCB_Odometry, this);
            pub_ = nh_.advertise<geometry_msgs::Twist>("dock_exec_cmd_vel", 1);
            goalreachedPub_ = nh_.advertise<std_msgs::Char>("dock_exec_status", 1);
            fast_mode_client = nh_.serviceClient<std_srvs::SetBool>("fast_spin");
        } else {
            goal_sub_.shutdown();
            pose_sub_.shutdown();
            pub_.shutdown();
        }
    }

    if (get_param_ok) {
        ROS_INFO_STREAM("[Docking Executor]: "
                        << "Set params ok");
    } else {
        ROS_WARN_STREAM("[Docking Executor]: "
                        << "Set params failed");
    }
    return true;
}

double DockExecutor::distance(double x1, double y1, double x2, double y2) {
    double distance = 0.0;
    distance = hypot((x2 - x1), (y2 - y1));
    return distance;
}

void DockExecutor::velocityPUB() {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel_[0];
    cmd_vel.linear.y = vel_[1];
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = vel_[2];
    pub_.publish(cmd_vel);
}

void DockExecutor::timerCB(const ros::TimerEvent& e) {
    if (if_get_goal_) {

        dist_ = distance(pose_[0], pose_[1], goal_[0], goal_[1]);
        ang_diff_ = goal_[2] - pose_[2];
        first_ang_diff_ = atan2((goal_[1] - pose_[1]),(goal_[0] - pose_[0])) - pose_[2];
        if_drive_straight_ = 1;
        if (first_ang_diff_ > M_PI * 3.0 / 2.0) first_ang_diff_ = first_ang_diff_ - 2 * M_PI;
        else if (first_ang_diff_ < - M_PI * 3.0 / 2.0) first_ang_diff_ = first_ang_diff_ + 2 * M_PI;

        if (first_ang_diff_ > M_PI / 2.0) {
            if_drive_straight_ = -1;
            first_ang_diff_ = first_ang_diff_ - M_PI;
        }
        else if (first_ang_diff_ < -M_PI / 2.0) {
            if_drive_straight_ = -1;
            first_ang_diff_ = M_PI + first_ang_diff_;
        }

        if (last_drive_straight_ == 0) last_drive_straight_ = if_drive_straight_;
        
        if(dist_ < mini_dist_){
            mini_dist_ = dist_;
        }


        t_now_ = ros::Time::now().toSec();
        dt_ = t_now_ - t_bef_;

        switch (mode_) {
            case MODE::MOVE: {
                // ROS_INFO_STREAM("start move");
                move();
                break;
            }
            case MODE::ROTATE: {
                rotate();
                break;
            }
            case MODE::FIRSTROTATE: {
                first_rotate();
                break;
            }
            case MODE::FASTROTATE: {
                fast_rotate();
                break;
            }
            case MODE::IDLE: {
                break;
            }
            case MODE::STOP: {
                stop();
                break;
            }
        }
        // publish cmd_vel
        velocityPUB();
    }

    // ROS_INFO("%f %f %f", vel_[0], vel_[1], dt_);

    // remember the time when leaving this loop
    t_bef_ = ros::Time::now().toSec();
}

void DockExecutor::move() {
    // calculate current distance to dock goal

    // 1. rivals appear
    double ang_with_rival = acos( ((rival_x_ - pose_[0]) * cos(pose_[2]) + (rival_y_ - pose_[1]) * sin(pose_[2])) / distance(pose_[0], pose_[1], rival_x_, rival_y_) );
    if (if_drive_straight_ == -1) ang_with_rival = M_PI - ang_with_rival;
    if (robot_type_ == "holonomic") ang_with_rival = std::min(ang_with_rival, M_PI - ang_with_rival);
    // ROS_INFO_STREAM("[Dock Executor]: " << "ang with rival: " << ang_with_rival);
    // if(rival_dist_ <= rival_tolerance_ && ang_with_rival < M_PI / 2.0 && rival_dist_ * sin(ang_with_rival) < rival_parallel_tolerance_){
    //     vel_[0] = vel_[1] = vel_[2] = 0.0;
    //     velocityPUB();
    //     std_msgs::Char finished;
    //     finished.data = 2;
    //     goalreachedPub_.publish(finished);
    //     ROS_WARN("[Dock Executor] : Meet rival! Waiting navigation_main to resend goal...");
    //     return;
    // }

    double last_vel = fabs(vel_[0]);
    double dist_to_goal = distance(pose_[0], pose_[1], goal_[0], goal_[1]);
    if (robot_type_ == "holonomic") {
        last_vel = sqrt(pow(vel_[0], 2) + pow(vel_[1], 2));
        dist_to_goal = distance(pose_[0], pose_[1], goal_[0], goal_[1]);
    }

    // calculate x-direction to goal
    if (robot_type_ == "differential") {
        double angle_with_goal = acos((cos(pose_[2]) * (goal_[0] - pose_[0]) + sin(pose_[2]) * (goal_[1] - pose_[1])) / dist_to_goal);
        if (angle_with_goal > M_PI / 2.0) {
            angle_with_goal = M_PI - angle_with_goal;
        }
        dist_to_goal = dist_to_goal * cos(angle_with_goal);
    }
    // =================================
    double cur_vel = dist_to_goal * cur_linear_kp_;
    cur_vel = std::min(cur_linear_max_vel_, cur_vel);
    cur_vel = std::min(last_vel + (cur_linear_acceleration_ * dt_), cur_vel);
    cur_vel = std::max(cur_vel, linear_min_vel_);
    if (dacc_start_) {
        double diff_value = (cur_vel - last_vel) / dt_ * linear_kd_;
        if (fabs(diff_value) < cur_vel) {
            // ROS_INFO_STREAM("[Dock Executor]: " << "kd " << diff_value << " " << cur_vel);
            cur_vel -= fabs(diff_value);
        }
    }
    if (fabs(last_vel) > fabs(cur_vel)) dacc_start_ = true;
    
    

    // double cur_vel = last_vel;

    // if (dacc_start_ || last_vel * expec_dacc_duration / 2.0 >= dist_) {
    //     cur_vel -= linear_acceleration_ * dt_;
    //     dacc_start_ = true;
    // }
    // else {
    //     cur_vel = std::min(last_vel + (linear_acceleration_ * dt_), linear_max_vel_);
    // }

    if (robot_type_ == "differential" && (cur_vel <= 0 || last_drive_straight_ * if_drive_straight_ < 0)) {
        vel_[0] = 0.0;
        vel_[1] = 0.0;
        vel_[2] = 0.0;

        velocityPUB();
        ROS_INFO("[Dock Executor] : Move Successfully docked-move!");
        // mode_ = MODE::ROTATE;
        std_srvs::SetBool fast_spin_srv;
        fast_spin_srv.request.data = true;
        if (fast_mode_client.call(fast_spin_srv)) {
            mode_ = MODE::FASTROTATE;
            ROS_INFO("[Dock Executor]: Set Mode to DOCK FASTROTATE!");
        }
        else {
            mode_ = MODE::ROTATE;
            ROS_WARN("[Dock Executor]: Set Mode to DOCK ROTATE!");
        }
        return;
    }
    if (dist_to_goal < tolerance_) { // mostly for holonomic
        vel_[0] = 0.0;
        vel_[1] = 0.0;
        vel_[2] = 0.0;

        velocityPUB();
        ROS_INFO("[Dock Executor] : Move Successfully docked-move!");
        // mode_ = MODE::ROTATE;
        std_srvs::SetBool fast_spin_srv;
        fast_spin_srv.request.data = true;
        if (fast_mode_client.call(fast_spin_srv)) {
            mode_ = MODE::FASTROTATE;
            ROS_INFO("[Dock Executor]: Set Mode to DOCK FASTROTATE!");
        }
        else {
            mode_ = MODE::ROTATE;
            ROS_WARN("[Dock Executor]: Set Mode to DOCK ROTATE!");
        }
        return;
    }
    // 2. on the point
    // if (dist_ < tolerance_) {
    //     vel_[0] = 0.0;
    //     vel_[1] = 0.0;
    //     vel_[2] = 0.0;
    //     velocityPUB();
    //     ROS_INFO("[Dock Executor] : Move Successfully docked-move!");
    //     mode_ = MODE::ROTATE;
    //     ROS_INFO("[Dock Executor]: Set Mode to DOCK ROTATE!");
    //     return;
    // }

    // if(dist_ > first_rotate_range_ && fabs(first_ang_diff_) > move_fail_tolerance_){
    //     vel_[0] = 0.0;
    //     vel_[1] = 0.0;
    //     vel_[2] = 0.0;
    //     velocityPUB();
    //     ROS_ERROR("[Dock Executor] : Move Failed docked-move!");
    //     mode_ = MODE::FIRSTROTATE;
    //     ROS_INFO("[Dock Executor]: Set Mode to DOCK FIRST ROTATE!");
    //     return;
    // }

    // if(dist_ > mini_dist_ && dist_ < move_over_tolerance_){
    //     vel_[0] = 0.0;
    //     vel_[1] = 0.0;
    //     vel_[2] = 0.0;
    //     velocityPUB(); // if(fabs(first_ang_diff_ ) < 0.02)
    // //     ang_vel = ang_vel/2;
    //     ROS_ERROR("[Dock Executor] : Move Over the docker goal!");
    //     mode_ = MODE::ROTATE;
    //     ROS_INFO("[Dock Executor]: Set Mode to DOCK ROTATE!");
    //     return;
    // }


        // current time when going in this loop
    double angle_vel = first_ang_diff_ * angular_adjust_kp_;
    if (dist_to_goal < 0.07) {
        ROS_WARN_STREAM("[Dock] Enter no adjustment distance");
        angle_vel = 0; 
    }
    if (angle_vel > angular_adjust_limit_) angle_vel = angular_adjust_limit_;
    if (angle_vel < -angular_adjust_limit_) angle_vel = -angular_adjust_limit_;

    if (robot_type_ == "differential") {
        vel_[0] = cur_vel * if_drive_straight_;
        vel_[1] = 0;
        vel_[2] = angle_vel; 
    }
    else if (robot_type_ == "holonomic") {
        vel_[0] = cur_vel * cos(first_ang_diff_) * if_drive_straight_;
        vel_[1] = cur_vel * sin(first_ang_diff_) * if_drive_straight_;
        vel_[2] = 0;
    }
    else {
        ROS_WARN_STREAM("[DockExecutor]" << "No such type: " << robot_type_);
    }
    // t_bef_ = ros::Time::now().toSec();
    velocityPUB();
}

void DockExecutor::rotate() {
    ang_diff_ = goal_[2] - pose_[2];

    while (ang_diff_ >= M_PI) {
        ang_diff_ -= 2 * M_PI;
    }
    while (ang_diff_ <= -M_PI) {
        ang_diff_ += 2 * M_PI;
    }
    double ang_vel = std::min(angular_max_vel_, fabs(ang_diff_) * angular_kp_);
    // if (ang_vel < angular_min_vel_) ang_vel = angular_min_vel_;
    // ROS_INFO("%f, %f", ang_diff_, ang_tolerance_);
    if (fabs(ang_diff_) < ang_tolerance_) {
        debounce_ang ++;
    } else {
        debounce_ang = 0;
    }

    if(debounce_ang >= 6){
        vel_[0] = vel_[1] = vel_[2] = 0.0;
        velocityPUB();
        ROS_INFO("[Dock Executor] : Successfully dock-rotated!");
        if_get_goal_ = false;
        std_msgs::Char finished;
        finished.data = 1;
        goalreachedPub_.publish(finished);
        ROS_INFO("[Dock Executor] : Successfully dock-arrive-goal!");
    } else {
        // ROS_INFO("[Dock Executor] : debounce_ang -> %d", debounce_ang);
    }

    if (ang_diff_ > 0)
        ang_vel = ang_vel;
    else if (ang_diff_ < 0)
        ang_vel = -1 * ang_vel;

    // ROS_INFO("ang_diff_: %f",ang_diff_);
    // ROS_INFO("ang_vel: %f",ang_vel);
    vel_[0] = vel_[1] = 0.0;
    vel_[2] = ang_vel;
    velocityPUB();
}

void DockExecutor::fast_rotate() {
    ang_diff_ = goal_[2] - pose_[2];

    while (ang_diff_ >= M_PI) {
        ang_diff_ -= 2 * M_PI;
    }
    while (ang_diff_ <= -M_PI) {
        ang_diff_ += 2 * M_PI;
    }
    double ang_vel = std::min(fast_angular_max_vel_, fabs(ang_diff_) * fast_angular_kp_);
    // ang_vel = std::max(ang_vel, angular_min_vel_);
    if (fabs(ang_diff_) < ang_tolerance_) {
        debounce_ang ++;
    } else {
        debounce_ang = 0;
    }

    int debounce_threshold = 3;
    if(debounce_ang >= debounce_threshold){
        vel_[0] = vel_[1] = vel_[2] = 0.0;
        velocityPUB();
        ROS_INFO("[Dock Executor] : Successfully dock-fast-rotated!");
        if_get_goal_ = false;
        std_msgs::Char finished;
        finished.data = 1;
        goalreachedPub_.publish(finished);
        ROS_INFO("[Dock Executor] : Successfully dock-arrive-goal!");

        std_srvs::SetBool fast_spin_srv;
        fast_spin_srv.request.data = false;
        if (fast_mode_client.call(fast_spin_srv)) {
            ROS_INFO_STREAM("[Dock]: " << "Switch back to normal localization");
        }
        else {
            ROS_WARN_STREAM("[Dock]: " << "Switch back to normal localization failed");
        }
        mode_ = MODE::IDLE;
        return ;
    } 
    else if (debounce_ang >= debounce_threshold / 2) {
        std_srvs::SetBool fast_spin_srv;
        fast_spin_srv.request.data = false;
        if (fast_mode_client.call(fast_spin_srv)) {
            ROS_INFO_STREAM("[Dock]: " << "Switch back to normal localization");
        }
        else {
            ROS_WARN_STREAM("[Dock]: " << "Switch back to normal localization failed");
        }
    }
    else {
        // ROS_INFO("[Dock Executor] : debounce_ang -> %d", debounce_ang);
    }

    if (ang_diff_ > 0)
        ang_vel = ang_vel;
    else if (ang_diff_ < 0)
        ang_vel = -1 * ang_vel;

    // ROS_INFO("ang_diff_: %f",ang_diff_);
    // ROS_INFO("ang_vel: %f",ang_vel);
    vel_[0] = vel_[1] = 0.0;
    vel_[2] = ang_vel;
    velocityPUB();
}

void DockExecutor::first_rotate() {

    // ROS_INFO_STREAM("dock exec first_rotate " << first_ang_diff_ * 180 / M_PI << " deg");
    // double ang_vel = std::min(angular_max_vel_, fabs(first_ang_diff_) * angular_kp_);
    double ang_vel = angular_max_vel_;
    // t_now_ = ros::Time::now().toSec();

    // while (first_ang_diff_ >= M_PI) {
    //     first_ang_diff_-= 2 * M_PI;
    // }
    // while (first_ang_diff_ <= -M_PI) {
    //     first_ang_diff_ += 2 * M_PI;
    // }
    // ROS_INFO("%f, %f", ang_diff_, ang_tolerance_);

    if (dist_ < tolerance_){
        vel_[0] = vel_[1] = vel_[2] = 0.0;
        velocityPUB();
        ROS_INFO("[Dock Executor] : Only rotate !");
        std_srvs::SetBool fast_spin_srv;
        fast_spin_srv.request.data = true;
        if (fast_mode_client.call(fast_spin_srv)) {
            mode_ = MODE::FASTROTATE;
            ROS_INFO("[Dock Executor]: Set Mode to DOCK FASTROTATE!");
        }
        else {
            mode_ = MODE::ROTATE;
            ROS_INFO("[Dock Executor] : Set Mode to DOCK ROTATE!");
        }
        return;
    }

    if(first_rot_need_time_ <= t_now_ - t_first_rot_ || first_rot_need_time_ <= first_ang_tolerance_ / angular_max_vel_){
        vel_[0] = vel_[1] = vel_[2] = 0.0;
        velocityPUB();
        ROS_INFO("[Dock Executor] : Successfully dock-first-rotated!");
        mode_ = MODE::MOVE;
        last_drive_straight_ = 0;
        ROS_INFO("[Dock Executor]: Set Mode to DOCK MOVE!");
        return;
    }
    ROS_INFO_STREAM("dock exec enter first rotate"); 

    // if (fabs(first_ang_diff_) < first_ang_tolerance_) {
    //     vel_[0] = vel_[1] = vel_[2] = 0.0;
    //     velocityPUB();
    //     ROS_INFO("[Dock Executor] : Successfully dock-first-rotated!");
    //     mode_ = MODE::MOVE;
    //     last_drive_straight_ = 0;
    //     ROS_INFO("[Dock Executor]: Set Mode to DOCK MOVE!");
    //     return;
    // }

    if (first_ang_diff_ > 0)
        ang_vel = 1 * ang_vel;
    else if (first_ang_diff_ < 0)
        ang_vel = -1 * ang_vel;
    
    // if(fabs(first_ang_diff_ ) < 0.02)
    //     ang_vel = ang_vel/2;
    // ROS_INFO_STREAM(angular_max_vel_);
    // ROS_INFO("ang_diff_: %f",ang_diff_);
    ROS_WARN("first rotate ang_vel: %f",ang_vel);
    vel_[0] = vel_[1] = 0.0;
    vel_[2] = ang_vel;
    velocityPUB();
}

void DockExecutor::stop(){
    vel_[0] = 0.0;
    vel_[1] = 0.0;
    vel_[2] = 0.0;
    ROS_ERROR("[Dock Executor]: STOP!!!!!!!!!!!!!!!");
}

void DockExecutor::goalCB(const geometry_msgs::PoseStamped& data) {
    ROS_INFO_STREAM("[Dock Executor]: In the goalCB!");

    vel_[0] = vel_[1] = vel_[2] = 0.0;
    if (data.pose.position.x == -1 && data.pose.position.y == -1) {
        ROS_INFO("[Dock Executor]: Mission Abort!");
        mode_ = MODE::IDLE;
        return;
    }
    // ROS_INFO("[Dock Executor]: Dock goal received! (%f, %f)", data.pose.position.x, data.pose.position.y);

    tf2::Quaternion q;
    tf2::fromMsg(data.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    t_now_ = ros::Time::now().toSec();

    if (data.header.frame_id == "dock") {
        cur_linear_max_vel_ = linear_max_vel_;
        cur_linear_acceleration_ = linear_acceleration_;
        cur_linear_kp_ = linear_kp_;
    }
    else if (data.header.frame_id == "slow-dock") {
        cur_linear_max_vel_ = slow_linear_max_vel_;
        cur_linear_acceleration_ = slow_acceleration_;
        cur_linear_kp_ = slow_linear_kp_;
    }
    else {
        ROS_WARN_STREAM("No such dock mode: " << data.header.frame_id);
        cur_linear_max_vel_ = linear_max_vel_;
        cur_linear_acceleration_ = linear_acceleration_;
        cur_linear_kp_ = linear_kp_;
    }

    goal_[0] = data.pose.position.x;  // + dock_dist_*cos(yaw);
    goal_[1] = data.pose.position.y;  // + dock_dist_*sin(yaw);
    goal_[2] = yaw;


    dist_ = distance(pose_[0], pose_[1], goal_[0], goal_[1]);
    ang_diff_ = goal_[2] - pose_[2];
    first_ang_diff_ = atan2((goal_[1] - pose_[1]),(goal_[0] - pose_[0])) - pose_[2];

    // ROS_INFO(" pose : %f %f %f vel : %f %f %f", pose_  [0],pose_[1],pose_[2], vel_[0],vel_[1],vel_[2]);
    // ROS_INFO(" dis : %f ,ang_dif :%f ,first_angle_diff: %f ", dist_,ang_diff_,first_ang_diff_);
    
    if_drive_straight_ = 1;
    if (first_ang_diff_ > M_PI * 3.0 / 2.0) first_ang_diff_ = first_ang_diff_ - 2 * M_PI;
    else if (first_ang_diff_ < - M_PI * 3.0 / 2.0) first_ang_diff_ = first_ang_diff_ + 2 * M_PI;

    if (first_ang_diff_ > M_PI / 2.0) {
        if_drive_straight_ = -1;
        first_ang_diff_ = first_ang_diff_ - M_PI;
    }
    else if (first_ang_diff_ < -M_PI / 2.0) {
        if_drive_straight_ = -1;
        first_ang_diff_ = M_PI + first_ang_diff_;
    }
    // ROS_INFO(" dis : %f ,ang_dif :%f ,first_angle_diff: %f ", dist_,ang_diff_,first_ang_diff_);

    first_rot_need_time_ = fabs(first_ang_diff_ / angular_max_vel_);

    t_first_rot_ = t_now_;

    ROS_INFO_STREAM(t_now_);
    mode_ = MODE::FIRSTROTATE;
    ROS_INFO("[Dock Executor]: Set Mode to DOCK FIRST ROTATE!");
    // ROS_INFO("[Dock Executor]: Set Mode to DOCK FIRST ROTATE!");

    mini_dist_ = 100;
    if_get_goal_ = true;
    dacc_start_ = false;
    debounce_ang = 0;
    t_bef_ = ros::Time::now().toSec();
}

void DockExecutor::poseCB_Odometry(const nav_msgs::Odometry& data) {
    pose_[0] = data.pose.pose.position.x;
    pose_[1] = data.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(data.pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    pose_[2] = yaw;
    // ROS_INFO("odom: %f %f", pose_[0], pose_[1]);
}

void DockExecutor::poseCB_PoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped& data) {
    pose_[0] = data.pose.pose.position.x;
    pose_[1] = data.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(data.pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    pose_[2] = yaw;
    //** ROS_INFO("odom: %f %f", pose_[0], pose_[1]);
}

void DockExecutor::rivalCB_Odometry(const nav_msgs::Odometry& data){
    double rival_x_ = data.pose.pose.position.x;
    double rival_y_ = data.pose.pose.position.y;
    if (rival_x_ < 0 || rival_x_ > 3.0 || rival_y_ < 0 || rival_y_ > 2.0) {
        rival_dist_ = 100;
        return ;
    }
    double dist = distance(pose_[0], pose_[1], data.pose.pose.position.x, data.pose.pose.position.y);
    rival_dist_ =  dist;
    // ROS_INFO("[Dock Executor]: Rival distance: %f",rival_dist_);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dockExecutor");
    ros::NodeHandle nh(""), nh_local("~");
    DockExecutor dockExecutor(nh, nh_local);

    while (ros::ok()) {
        ros::spin();
    }
}
// void DockExecutor::vibrate() {
//     // vibrate n times.   n = vibrate_time_goal 
//     if(vibrate_time_now_ >= vibrate_time_goal_){
//         vel_[0] = vel_[1] = vel_[2] = 0.0;
//         if_get_goal_ = false;
//         vibrate_time_now_ = 0;
//         std_msgs::Char finished;
//         finished.data = 1;
//         goalreachedPub_.publish(finished);
//         ROS_INFO("[Dock Executor] : Successfully dock-vibrated!");
//         return;
//     }

//     double lin_vel = vibrate_linear_max_vel_;
//     if(vibrate_time_now_ % 2 == 0){
//         goal_[0] = vibrate_pos_start_x_;
//         goal_[1] = vibrate_pos_start_y_ + vibrate_lin_dist_ / 2.0;
//     }else if(vibrate_time_now_ % 2 == 1){
//         goal_[0] = vibrate_pos_start_x_;
//         goal_[1] = vibrate_pos_start_y_ - vibrate_lin_dist_ / 2.0;
//     }
//     if(vibrate_time_now_ == vibrate_time_goal_-1){
//         goal_[0] = vibrate_pos_start_x_;
//         goal_[1] = vibrate_pos_start_y_;
//     }

//     // check if reach goal
//     // ROS_INFO("x:%f, y:%f, goal_x:%f, goal_y:%f",pose_[0], pose_[1], goal_[0], goal_[1] );
//     dist_ = distance(pose_[0], pose_[1], goal_[0], goal_[1]);
//     if (dist_ < vibrate_tolerance_) {
//         vel_[0] = vel_[1] = vel_[2] = 0.0;
//         vibrate_time_now_++;
//         ROS_INFO("[Dock Executor] : Vibrate times:%d", vibrate_time_now_);
//     }else{
//         // calculate the angle that we send to odometry mcu
//         cosx_ = ((goal_[0] - pose_[0]) * cos(pose_[2]) + (goal_[1] - pose_[1]) * sin(pose_[2])) / dist_;
//         sinx_ = sqrt(1 - pow(cosx_, 2));
//         if ((cos(pose_[2]) * (goal_[1] - pose_[1])) - (sin(pose_[2]) * (goal_[0] - pose_[0])) < 0)
//             sinx_ *= -1;

//         vel_[0] = vibrate_linear_max_vel_ * cosx_;
//         vel_[1] = vibrate_linear_max_vel_ * sinx_;
//     }

// }