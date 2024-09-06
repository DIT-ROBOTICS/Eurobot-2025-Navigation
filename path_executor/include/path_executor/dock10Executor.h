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

enum class MODE {
    MOVE = 0,
    ROTATE,
    IDLE,
};

class Dock10Executor {
   public:
    Dock10Executor(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    ~Dock10Executor();
    bool initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void initialize();

   private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;

    // Subscriber
    ros::Subscriber goal_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber rival_sub_;
    // ros::Subscriber rival2_sub_;
    void goalCB(const geometry_msgs::PoseStamped& data);
    void poseCB_Odometry(const nav_msgs::Odometry& data);
    void poseCB_PoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped& data);
    void rivalCB_Odometry(const nav_msgs::Odometry& data);

    // Publisher
    ros::Publisher pub_;
    ros::Publisher goalreachedPub_;
    void velocityPUB();
    
    // Server
    ros::ServiceClient fast_mode_client;

    // Timer
    ros::Timer timer_;
    void timerCB(const ros::TimerEvent& e);

    void move();
    void rotate();
    void first_rotate();
    void fast_rotate();
    void vibrate();
    void stop();

    double t_bef_;
    double t_now_;
    double dt_;

    bool move_finished_;
    bool rotate_finished_;
    bool dacc_start_;
    // bool rival_appeared_;

    std::string robot_type_; 

    double goal_[3];
    double pose_[3];
    double vel_[3];
    double dock_dist_;
    bool if_get_goal_;
    bool count_dock_dist_;
    double a_;
    double dist_;
    double first_rotate_range_;
    double ang_diff_;
    double first_ang_diff_;
    double cosx_;
    double sinx_;
    double mini_dist_ = 1000;
    double if_drive_straight_ = 1;
    double last_drive_straight_;
    double rival_dist_;
    double first_rot_need_time_;
    double t_first_rot_;
    

    int debounce;
    int debounce_ang;

    double rival_x_;
    double rival_y_;

    bool p_active_;
    double control_frequency_;
    double linear_max_vel_;
    double linear_kp_;
    double linear_kd_;
    double linear_acceleration_;
    double slow_linear_max_vel_;
    double slow_linear_kp_;
    double slow_acceleration_;
    double linear_min_vel_;

    double cur_linear_max_vel_;
    double cur_linear_acceleration_;
    double cur_linear_kp_;


    double angular_max_vel_;
    double angular_min_vel_;
    double angular_kp_;
    double fast_angular_max_vel_;
    double fast_angular_kp_;
    double cake_linear_max_vel_;
    double cake_angular_max_vel_;
    double cherry_linear_max_vel_;
    double cherry_angular_max_vel_;
    double div_;
    double profile_percent_;
    double tolerance_;
    double ang_tolerance_;
    double first_ang_tolerance_;
    double move_fail_tolerance_;
    double move_over_tolerance_;
    int pose_type_;
    double rival_tolerance_;
    double rival_parallel_tolerance_;
    double angular_adjust_kp_;
    double angular_adjust_limit_;

    // vibrate-mode
    int vibrate_time_now_;
    int vibrate_time_goal_;
    double vibrate_linear_max_vel_;
    double vibrate_pos_start_x_;
    double vibrate_pos_start_y_;
    double vibrate_tolerance_;
    double vibrate_lin_dist_;

    MODE mode_;
    double distance(double x1, double y1, double x2, double y2);
};