#include <ros/console.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/SetBool.h>
#include <string>
#include <cmath>

#define _USE_MATH_DEFINES


enum class MODE{
    IDLE,MOVE,STOP,
};

class MyExecutor{
    public:
        MyExecutor(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
        ~MyExecutor();
        bool initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
        void initialize();
    private:
        ros::NodeHandle nh;
        ros::NodeHandle nh_local;
        ros::ServiceServer params_srv;

        // Subscriber
        ros::Subscriber goal_sub;
        ros::Subscriber pose_sub;

        void goalCB(const geometry_msgs::PoseStamped& data);
        void poseCB_Odometry(const nav_msgs::Odometry& data);
        void poseCB_PoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped& data);
        // Publisher
        ros::Publisher pub;
        ros::Publisher goalreachedPub;
        void velocityPUB();

        // Server
        ros::ServiceClient fast_mode_client;

        // Timer
        ros::Timer timer;
        void timerCB(const ros::TimerEvent& e);

        void move();
        void stop();

        bool move_finished;

        std::string robot_type;
        int pose_type; 
        double goal[3];
        double pose[3];
        double vel[3];
        double dock_dist;
        bool if_get_goal;
        bool count_dock_dist;
        bool dacc_start;
        double mini_dist = 1000;
        MODE cur_mode;
        bool p_active;
        double control_frequency;
        double distance;
        double time_now;
        double dt;
        double t_bef;
        double time_before;
        double tolerance;
        double dist;
        double calculate_dist(double x1, double y1, double x2, double y2);
    
        double linear_min_vel;
        double linear_max_vel;
        double linear_kp;
        double linear_kd;
        double linear_acceleration;
        double cur_linear_max_vel;
        double cur_linear_acceleration;
        double cur_linear_kp;
        double if_drive_straight = 1;
        double last_drive_straight;
        double ang_diff;
        double first_ang_diff;
};