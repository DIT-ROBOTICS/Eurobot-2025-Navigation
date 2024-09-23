#include "dock10Executor.h"
Dock10Executor::Dock10Executor(ros::NodeHandle& nh, ros::NodeHandle& nh_local) {
    nh_ = nh;
    nh_local_ = nh_local;
    std_srvs::Empty empt;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("params", &Dock10Executor::initializeParams, this);
    initializeParams(empt.request, empt.response);
    initialize();
}

Dock10Executor::~Dock10Executor() {
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

void Dock10Executor::initialize() {
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

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &Dock10Executor::timerCB, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency_), false);
    timer_.start();

    rival_dist_ = 100;
    mini_dist_ = 100;

    debounce = 0;
    debounce_ang = 0;

    rival_x_ = 0;
    rival_y_ = 0;
}

bool Dock10Executor::initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    // Load parameter
    bool get_param_ok = true;
    bool prev_active = p_active_;
    get_param_ok = nh_local_.param<bool>("active", p_active_, true);
    // get_param_ok = nh_local_.param<string>("", _, "");
    get_param_ok &= nh_local_.param<double>("control_frequency", control_frequency_, 50);
    get_param_ok &= nh_local_.param<std::string>("robot_type", robot_type_, "holonomic");
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_max_velocity", linear_max_vel_, 0.3);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_kp", linear_kp_, 2.0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_kd", linear_kd_, 1.0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_acceleration", linear_acceleration_, 0.0001);
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
    get_param_ok &= nh_local_.param<std::string>("robot_name",robot_name_,"robot");
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/front_check_d", front_check_dx_, 0.005);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/front_check_d", front_check_dy_, 0.005);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/front_check_dist", front_check_dist_, 0.1);

    get_param_ok &= nh_local_.param<int>("pose_type", pose_type_, 0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/rival_tolerance", rival_tolerance_, 0.40);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "rival_parallel_tolerance", rival_parallel_tolerance_, 0.4);

    if (p_active_ != prev_active) {
        if (p_active_) {
            goal_sub_ = nh_.subscribe("dock10_exec_goal", 50, &Dock10Executor::goalCB, this);
            if (pose_type_ == 0) {
                pose_sub_ = nh_.subscribe("odom", 50, &Dock10Executor::poseCB_Odometry, this);
            } else if (pose_type_ == 1) {
                pose_sub_ = nh_.subscribe("final_pose", 50, &Dock10Executor::poseCB_Odometry, this);
            }
            // costmap_2d::Costmap2DROS global_costmap("global_costmap", nh_);
            // costmap_2d::Costmap2D* master_grid = global_costmap.getCostmap();
            rival_sub_ = nh_.subscribe("/rival/final_pose", 50, &Dock10Executor::rivalCB_Odometry, this);
            
            // rival2_sub_ = nh_.subscribe("/rival2/odom", 50, &DockExecutor::rivalCB_Odometry, this);
            std::string jkl = robot_name_ + "/move_base/global_costmap/costmap";
            costmap2d_sub_ = nh_.subscribe( "/" + robot_name_ + "/move_base/global_costmap/costmap",50, &Dock10Executor::costmap2dCB, this);         
            ROS_WARN("12313213233131321 %s",  jkl.c_str());
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
        ROS_INFO_STREAM("[Docking10 Executor]: "
                        << "Set params ok");
    } else {
        ROS_WARN_STREAM("[Docking10 Executor]: "
                        << "Set params failed");
    }
    return true;
}

double Dock10Executor::distance(double x1, double y1, double x2, double y2) {
    double distance = 0.0;
    distance = hypot((x2 - x1), (y2 - y1));
    return distance;
}

void Dock10Executor::velocityPUB() {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel_[0];
    cmd_vel.linear.y = vel_[1];
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = vel_[2];
    pub_.publish(cmd_vel);
}
void Dock10Executor::costmap2dCB(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    costmap_data = msg->data;
    //ROS_INFO("cost %d", costmap_data[41360]);
    // float reso = msg->info.resolution;
    // int width = msg->info.width;
    int height = msg->info.height;
    // // ROS_ERROR("costmap_data[1] = %d", costmap_data[1]);
    // ROS_INFO("reso is %f", reso);
    // ROS_INFO("w is %d", width);
    //ROS_INFO("h is %d", height);

}

void Dock10Executor::timerCB(const ros::TimerEvent& e) {
    if (if_get_goal_) {

        dist_ = distance(pose_[0], pose_[1], goal_[0], goal_[1]);
        ang_diff_ = goal_[2] - pose_[2];
        first_ang_diff_ = atan2((goal_[1] - pose_[1]),(goal_[0] - pose_[0])) - pose_[2];
        /*if_drive_straight_ = 1;
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
        }*/


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
            case MODE::IDLE: {
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
void Dock10Executor::move() {
    if(dist_ <= tolerance_){
        ROS_INFO("[dock10 Executor]: move sucessfully");
        vel_[0] = 0.0;
        vel_[1] = 0.0;
        vel_[2] = 0.0;
        velocityPUB();
        mode_ = MODE::ROTATE;
    }
    else{
        slope_x_ = (goal_[1] - pose_[1]) / (goal_[0] - pose_[0]);
        slope_y_ = (goal_[0] - pose_[0]) / (goal_[1] - pose_[1]);
        double front_check_dist_x = fabs(front_check_dist_ * cos(second_ang_diff_));
        double front_check_dist_y = fabs(front_check_dist_ * cos(M_PI / 2 + second_ang_diff_));
        double exact_front_check_dist_x = std::min(front_check_dist_x, dist_);
        double exact_front_check_dist_y = std::min(front_check_dist_y, dist_);
        int front_check_num_x = exact_front_check_dist_x / front_check_dx_;
        int front_check_num_y = exact_front_check_dist_y / front_check_dy_;
        
        
        front_check_point_.clear();

        
            //ROS_WARN("66666666666");
            //ROS_WARN("fabs second_ang_diff_ is %lf", fabs(second_ang_diff_));

            if(fabs(second_ang_diff_) >= (M_PI / 4) && fabs(second_ang_diff_) <= (M_PI * 3 / 4)){
                for(int i = 0; i < front_check_num_y; i++){
                    if(goal_[1] >= pose_[1]){
                        //ROS_WARN("case 1");
                        double x_p = (i+1) * front_check_dy_ * slope_y_ + pose_[0];
                        double y_p = (i+1) * front_check_dy_ + pose_[1];
                        //ROS_WARN("X = %lf Y = %lf",x_p,y_p);
                        //ROS_WARN("77777777777777");
                        front_check_point_.push_back(std::make_pair(x_p,y_p));
                    }
                    else{
                        //ROS_WARN("case 2");
                        double x_p = -(i+1) * front_check_dy_ * slope_y_ + pose_[0];
                        double y_p = -(i+1) * front_check_dy_ + pose_[1];
                        //ROS_WARN("X = %lf Y = %lf",x_p,y_p);
                        //ROS_WARN("77777777777777");
                        front_check_point_.push_back(std::make_pair(x_p,y_p));
                    }  
                }
                
            }
            else{
                for(int i = 0; i < front_check_num_x; i++){
                    if(goal_[0] >= pose_[0]){
                    //ROS_WARN("case 3");
                    double x_p = (i+1) * front_check_dx_ + pose_[0];
                    double y_p = (i+1) * front_check_dx_ * slope_x_ + pose_[1];
                    //ROS_WARN("X = %lf Y = %lf",x_p,y_p);
                    //ROS_WARN("77777777777777");
                    front_check_point_.push_back(std::make_pair(x_p,y_p));
                    }
                    else{
                        //ROS_WARN("case 4");
                        double x_p = -(i+1) * front_check_dx_ + pose_[0];
                        double y_p = -(i+1) * front_check_dx_ * slope_x_ + pose_[1];
                        //ROS_WARN("X = %lf Y = %lf",x_p,y_p);
                        //ROS_WARN("77777777777777");
                        front_check_point_.push_back(std::make_pair(x_p,y_p));
                    }
                }
                    
            }
        // for(int i = 0; i < front_check_num_; i++){
        //     //ROS_WARN("66666666666");
        //     // ROS_WARN("fabs second_ang_diff_ is %lf", fabs(second_ang_diff_));
        //     if(fabs(second_ang_diff_) >= (M_PI / 4) && fabs(second_ang_diff_) <= (M_PI * 3 / 4)){
        //         if(goal_[1] >= pose_[1]){
        //             //ROS_WARN("case 1");
        //             double x_p = (i+1) * front_check_dy_ * slope_y_ + pose_[0];
        //             double y_p = (i+1) * front_check_dy_ + pose_[1];
        //             //ROS_WARN("X = %lf Y = %lf",x_p,y_p);
        //             //ROS_WARN("77777777777777");
        //             front_check_point_.push_back(std::make_pair(x_p,y_p));
        //         }
        //         else{
        //             //ROS_WARN("case 2");
        //             double x_p = -(i+1) * front_check_dy_ * slope_y_ + pose_[0];
        //             double y_p = -(i+1) * front_check_dy_ + pose_[1];
        //             //ROS_WARN("X = %lf Y = %lf",x_p,y_p);
        //             //ROS_WARN("77777777777777");
        //             front_check_point_.push_back(std::make_pair(x_p,y_p));
        //         }  
        //     }
        //     else{
                
        //         if(goal_[0] >= pose_[0]){
        //             //ROS_WARN("case 3");
        //             double x_p = (i+1) * front_check_dx_ + pose_[0];
        //             double y_p = (i+1) * front_check_dx_ * slope_x_ + pose_[1];
        //             //ROS_WARN("X = %lf Y = %lf",x_p,y_p);
        //             //ROS_WARN("77777777777777");
        //             front_check_point_.push_back(std::make_pair(x_p,y_p));
        //         }
        //         else{
        //             //ROS_WARN("case 4");
        //             double x_p = -(i+1) * front_check_dx_ + pose_[0];
        //             double y_p = -(i+1) * front_check_dx_ * slope_x_ + pose_[1];
        //             //ROS_WARN("X = %lf Y = %lf",x_p,y_p);
        //             //ROS_WARN("77777777777777");
        //             front_check_point_.push_back(std::make_pair(x_p,y_p));
        //         }
        //     }
            
            
        //}
        for(int i = front_check_point_.size() - 1; i >= 0; i--){
            //ROS_WARN("i = %d", i);

            int mapX = front_check_point_[i].first * 100;
            int mapY = front_check_point_[i].second * 100;
            int cost = (mapY-1) * 300 + mapX;
            //ROS_INFO("cost = %d, costmap_data.size = %ld", cost,costmap_data.size());
            
            if(costmap_data[cost] > 0 || costmap_data[cost] == -1){
                ROS_WARN("i = %d", i);
                ROS_ERROR("cost = %d", cost);
                ROS_ERROR("costmap_data[cost] = %d", costmap_data[cost]);
                mode_ = MODE::IDLE;
                goal_[0] = pose_[0];
                goal_[1] = pose_[1];
                goal_[2] = pose_[2];
                front_check_point_.clear();
                break;
            }
        }
        // ROS_INFO("[Docking10 Executor]:Not finish yet");
        if(mode_ == MODE::IDLE){
            ROS_INFO("[Docking10 Executor]: There is an obstacle");
            vel_[0] = 0;
            vel_[1] = 0;
            vel_[2] = 0;
            velocityPUB();
        }
        else{
            // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
            // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
            double vel_x = 0.0;
            double vel_y = 0.0;
            double max_vel = 0.0;
            
            max_vel = std::min((dist_) * cur_linear_kp_, linear_max_vel_);
            vel_x = max_vel * cos(first_ang_diff_);
            vel_y = max_vel * sin(first_ang_diff_);
            if(vel_x >= 0){
                // ROS_INFO("[Docking10 Executor]: Linear_acceleration %lf", linear_acceleration_);
                vel_[0] += 1.0 / control_frequency_ * linear_acceleration_ * cos(first_ang_diff_);
                vel_[0] = std::min(vel_[0], vel_x);
            }
            else{
                vel_[0] += 1.0 / control_frequency_ * linear_acceleration_ * cos(first_ang_diff_);
                vel_[0] = std::max(vel_[0], vel_x);
            }
            
            if(vel_y >= 0){
                vel_[1] += 1.0 / control_frequency_ * linear_acceleration_ * sin(first_ang_diff_);
                vel_[1] = std::min(vel_[1], vel_y);
            }
            else{
                vel_[1] += 1.0 / control_frequency_ * linear_acceleration_ * sin(first_ang_diff_);
                vel_[1] = std::max(vel_[1], vel_y);
            }
            
            vel_[2] = 0;  
            velocityPUB();
        }
        
    }
    
}

void Dock10Executor::rotate() {
    //ROS_INFO("goal angle is %lf",goal_[2]);
    //ROS_INFO("pose angle is %lf",pose_[2]);
    ang_diff_ = goal_[2] - pose_[2];
    vel_[0] = 0.0;
    vel_[1] = 0.0;
    if(fabs(ang_diff_) < ang_tolerance_){
        ROS_INFO("[dock10 Executor]: rotate sucessfully");
        vel_[2] = 0.0;
        mode_ = MODE::IDLE;
    }
    else{
        if(pose_[2] >= 0 && goal_[2] >= 0){
            if(ang_diff_ >= 0){
                vel_[2] = std::min((ang_diff_ * angular_kp_), angular_max_vel_);
                //ROS_INFO("case 1");
            }
            else{
                vel_[2] = std::max((ang_diff_ * angular_kp_), -angular_max_vel_);
                //ROS_INFO("case 2");
            }
        }
        else if(pose_[2] < 0 && goal_[2] < 0){
           if(ang_diff_ >= 0){
                vel_[2] = std::min((ang_diff_ * angular_kp_), angular_max_vel_);
                //ROS_INFO("case 3");
            }
            else{
                vel_[2] = std::max((ang_diff_ * angular_kp_), -angular_max_vel_);
                //ROS_INFO("case 4");
            } 
        }
        else if(pose_[2] < 0 && goal_[2] >= 0){
            if((fabs(pose_[2]) + goal_[2]) >= M_PI){
                vel_[2] = std::max((-ang_diff_ * angular_kp_), -angular_max_vel_);
                //ROS_INFO("case 5");
            }
            else{
                vel_[2] = std::min((ang_diff_ * angular_kp_), angular_max_vel_);
                //ROS_INFO("case 6");
            }
        }
        else{
            if((pose_[2] + fabs(goal_[2])) <= M_PI){
                vel_[2] = std::max((ang_diff_ * angular_kp_), -angular_max_vel_);
                //ROS_INFO("case 7");
            }
            else{
                vel_[2] = std::min((-ang_diff_ * angular_kp_), angular_max_vel_);
                //ROS_INFO("case 8");
            }
        }
    }
    velocityPUB();
}

void Dock10Executor::goalCB(const geometry_msgs::PoseStamped& data) {
    ROS_INFO_STREAM("[Dock10 Executor]: In the goalCB!");

    vel_[0] = vel_[1] = vel_[2] = 0.0;
    if (data.pose.position.x == -1 && data.pose.position.y == -1) {
        ROS_INFO("[Dock10 Executor]: Mission Abort!");
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
    else if (data.header.frame_id == "dock10") {
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
    second_ang_diff_ = atan2((goal_[1] - pose_[1]),(goal_[0] - pose_[0]));
    ROS_WARN(" goal : %f %f %f ", goal_[0], goal_[1], goal_[2]);
    mode_ = MODE::MOVE;
    
    
    
    // global_costmap->getCostmap()->worldToMap(goal_[0], goal_[1], mx, my);
    // if (master_grid->worldToMap(goal_[0], goal_[1], mx, my)) {
        
    //     unsigned char cost = master_grid->getCost(mx, my);

        
    //     ROS_INFO("Cost at position (%f, %f) in world coordinates is: %u"goal_[0], goal_[1], cost);
    // } else {
        
    //     ROS_WARN("World coordinates (%f, %f) are out of bounds!",goal_[0], goal_[1]);
    // }
    // // if (global_costmap->getCostmap()->worldToMap(goal_[0], goal_[1], mx, my)) {
        
    //     unsigned char cost = global_costmap->getCostmap()->getCost(mx, my);
        
        
    //     ROS_INFO("Goal point cost: %d", cost);
    // }
    // else{
    //     ROS_INFO("Goal point cost cant get");
    // }
    // ROS_INFO(" dis : %f ,ang_dif :%f ,first_angle_diff: %f ", dist_,ang_diff_,first_ang_diff_);
    
    
    // ROS_INFO(" dis : %f ,ang_dif :%f ,first_angle_diff: %f ", dist_,ang_diff_,first_ang_diff_);

    first_rot_need_time_ = fabs(first_ang_diff_ / angular_max_vel_);

    t_first_rot_ = t_now_;

    ROS_INFO_STREAM(t_now_);
    
   
    mini_dist_ = 100;
    if_get_goal_ = true;
    dacc_start_ = false;
    debounce_ang = 0;
    t_bef_ = ros::Time::now().toSec();
}

void Dock10Executor::poseCB_Odometry(const nav_msgs::Odometry& data) {
    pose_[0] = data.pose.pose.position.x;
    pose_[1] = data.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(data.pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    pose_[2] = yaw;

    
}

void Dock10Executor::poseCB_PoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped& data) {
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

void Dock10Executor::rivalCB_Odometry(const nav_msgs::Odometry& data){
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
    ros::init(argc, argv, "dock10Executor");
    ros::NodeHandle nh(""), nh_local("~");
    Dock10Executor dock10Executor(nh, nh_local);
    ROS_WARN("this node is running");
    while (ros::ok()) {
        ros::spin();
    }
}


