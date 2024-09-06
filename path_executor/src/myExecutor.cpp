#include "myExecutor.h"

MyExecutor::MyExecutor(ros::NodeHandle& n, ros::NodeHandle& nh_l){
    nh = n;
    nh_local = nh_l;
    std_srvs::Empty empt;
    p_active = false;
    params_srv = nh_local.advertiseService("params", &MyExecutor::initializeParams, this);
    initializeParams(empt.request, empt.response);
    initialize();
}

MyExecutor::~MyExecutor() {
    nh_local.deleteParam("control_frequency");
}

void MyExecutor::initialize() {
    goal[0] = 0.0;
    goal[1] = 0.0;
    goal[2] = 0.0;

    pose[0] = 0.0;
    pose[1] = 0.0;
    pose[2] = 0.0;

    vel[0] = 0.0;
    vel[1] = 0.0;
    vel[2] = 0.0;

    dock_dist = 0.05;
    if_get_goal = false;
    count_dock_dist = false;
    dist = 0.0;
    cur_mode = MODE::IDLE;

    timer = nh.createTimer(ros::Duration(1.0/ control_frequency), &MyExecutor::timerCB, this, false);
    timer.setPeriod(ros::Duration(1.0/ control_frequency), false);
    timer.start();

    mini_dist = 100;
    ROS_INFO("Init finish");
}

bool MyExecutor::initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    bool get_param_ok = true;
    bool prev_active = p_active;
    get_param_ok = nh_local.param<bool>("active", p_active, true);
    get_param_ok &= nh_local.param<double>("control_frequency", control_frequency, 50);
    get_param_ok &= nh_local.param<double>(robot_type + "/linear_max_velocity", linear_max_vel, 0.3);
    get_param_ok &= nh_local.param<double>(robot_type + "/linear_kp", linear_kp, 2.0);
    get_param_ok &= nh_local.param<double>(robot_type + "/linear_kd", linear_kd, 1.0);
    get_param_ok &= nh_local.param<double>(robot_type + "/linear_acceleration", linear_acceleration, 0.15);
    get_param_ok &= nh_local.param<double>(robot_type + "/linear_min_velocity", linear_min_vel, 0.1); 
    get_param_ok &= nh_local.param<double>("/point_stop_tolerance", tolerance, 0.005);
    get_param_ok &= nh_local.param<std::string>("robot_type", robot_type, "differential");
    get_param_ok &= nh_local.param<int>("pose_type", pose_type, 0);
    
    if(p_active != prev_active){
        if(p_active){
            goal_sub = nh.subscribe("dock_exec_goal", 50, &MyExecutor::goalCB, this);
            pose_sub = (pose_type == 0) ? nh.subscribe("odom", 50, &MyExecutor::poseCB_Odometry, this) : nh.subscribe("final_pose", 50, &MyExecutor::poseCB_Odometry, this);

            pub = nh.advertise<geometry_msgs::Twist>("dock_exec_cmd_vel",1);
            goalreachedPub = nh.advertise<std_msgs::Char>("dock_exec_status",1);
            fast_mode_client = nh.serviceClient<std_srvs::SetBool>("fast_spin");
        }
        else{
            goal_sub.shutdown();
            pose_sub.shutdown();
            pub.shutdown();
        }
    }
    
    if(get_param_ok){
        ROS_INFO_STREAM("SET params ok");
    }
    else ROS_INFO_STREAM("a mess");

    return true;
}
void MyExecutor::poseCB_Odometry(const nav_msgs::Odometry& data){
    pose[0] = data.pose.pose.position.x;
    pose[1] = data.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(data.pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double a, yaw;
    qt.getRPY(a,a,yaw);
    pose[2] = yaw;
    ROS_INFO("pose odometry: %f, %f, %f\n",pose[0],pose[1],pose[2]);
}

void MyExecutor::poseCB_PoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped& data) {
    pose[0] = data.pose.pose.position.x;
    pose[1] = data.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(data.pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    pose[2] = yaw;
    ROS_INFO("pose covariance stamp: %f, %f, %f\n",pose[0],pose[1],pose[2]);
}

void MyExecutor::goalCB(const geometry_msgs::PoseStamped& data){
    ROS_INFO_STREAM("[My Executor]: In the goalCB");

    vel[0] = vel[1] = vel[2] = 0.0;
    if (data.pose.position.x == -1 && data.pose.position.y == -1) {
        ROS_INFO("[Dock Executor]: Mission Abort!");
        cur_mode = MODE::IDLE;
        return;
    }
    tf2::Quaternion q;
    tf2::fromMsg(data.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double a, yaw;
    qt.getRPY(a,a,yaw);

    time_now = ros::Time::now().toSec();

    cur_linear_max_vel = linear_max_vel;
    cur_linear_acceleration = linear_acceleration;
    cur_linear_kp = linear_kp;

    goal[0] = data.pose.position.x;
    goal[1] = data.pose.position.y;
    goal[2] = yaw;

    ROS_INFO("calculate dist from goalCB");
    dist = calculate_dist(pose[0], pose[1], goal[0], goal[1]);
    ang_diff = goal[2] - pose[2];
    first_ang_diff = atan2((goal[1]-pose[1]),(goal[0]-pose[0])) - pose[2];
    cur_mode = MODE::MOVE;
    if_drive_straight = 1;
    if_get_goal = true;
    dacc_start = false;
    t_bef = ros::Time::now().toSec();
    ROS_INFO("get goal: %f, %f, %f\n",goal[0],goal[1],goal[2]);
}
double MyExecutor::calculate_dist(double x1, double y1, double x2, double y2){
    double dis = 0.0;
    dis = hypot((x2-x1),(y2-y1));
    ROS_INFO("distance calculation %f", dis);
    return dis;
}

void MyExecutor::velocityPUB() {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel[0];
    cmd_vel.linear.y = vel[1];
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = vel[2];
    ROS_INFO("Senting out velocity: %f, %f, %f",cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    pub.publish(cmd_vel);
    ROS_INFO("publish success");
}

void MyExecutor::timerCB(const ros::TimerEvent& e){
    //ROS_INFO("%f",goal[0]);
    ROS_INFO("in timer back");
    if(if_get_goal){
        ROS_INFO("claculate dist from timerCB");
        dist = calculate_dist(pose[0],pose[1],goal[0],goal[1]);
        ang_diff = goal[2] - pose[2];
        first_ang_diff = atan2((goal[1]-pose[1]),(goal[0]-pose[0]) - pose[2]);
        if_drive_straight = 1;
        if(dist < mini_dist) mini_dist = dist;

        time_now = ros::Time::now().toSec();
        dt = time_now - time_before;
        switch(cur_mode){
            case MODE::MOVE:
                move();
                break;
            case MODE::IDLE:
                break;
            case MODE::STOP:
                stop();
                break;
        }
        velocityPUB();
    }

    time_before = ros::Time::now().toSec();
}

void MyExecutor::move(){
    double last_vel = fabs(vel[0]);
    double dist_to_goal = calculate_dist(pose[0], pose[1], goal[0],goal[1]);
    last_vel = sqrt(pow(vel[0],2) + pow(vel[1],2));
    double cur_vel = dist_to_goal * cur_linear_kp;
    cur_vel = std::min(cur_linear_max_vel, cur_vel);
    cur_vel  = std::min(last_vel + (cur_linear_acceleration * dt), cur_vel);
    cur_vel = std::max(cur_vel, linear_min_vel);

    if(dist_to_goal < tolerance) {
        for(int i = 0;i<3;i++) vel[i] = 0;
        ROS_INFO("publish from move");
        velocityPUB();
        ROS_INFO("[My Executor] : Move Successfully docked-move!");
        return;
    }
    vel[0] = cur_vel * cos(first_ang_diff);
    vel[1] = cur_vel * sin(first_ang_diff);
    vel[2] = 0;
    ROS_INFO("publish from move");
    velocityPUB();
}

void MyExecutor::stop(){
    for(int i = 0;i<3;i++) vel[i] = 0;
    ROS_ERROR("STOP");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "myExecutor");
    ros::NodeHandle nh_(""), nh_local_("~");
    MyExecutor myExecutor(nh_, nh_local_);

    while (ros::ok()) {
        ros::spin();
    }
}
