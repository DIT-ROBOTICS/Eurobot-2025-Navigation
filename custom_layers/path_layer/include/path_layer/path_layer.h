#ifndef PATH_LAYER_H_
#define PATH_LAYER_H_

// ros
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <ros/time.h>

// costmap
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

// msgs
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "obstacle_detector/CircleObstacle.h"
#include "obstacle_detector/Obstacles.h"

// other
#include <algorithm>
#include <cmath>
#include <string>
#include <Eigen/Dense>

namespace path_layer_namespace {

enum class ROBOT_TYPE {
    ROBOT = 0,
    // ROBOT1,
    // ROBOT2,
    RIVAL,
    // RIVAL1,
    // RIVAL2,
    OBSTACLE,
    POT,
    TRAJ_CIRCLE,
};

enum OdomCallbackType {
    nav_msgs_Odometry = 0,
    geometry_msgs_PoseWithCovarianceStamped = 1
};

// public costmap_2d::CostmapLayer
// public costmap_2d::Layer, public costmap_2d::Costmap2D
class PathLayer : public costmap_2d::CostmapLayer {
   public:
    PathLayer();
    ~PathLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x, double* max_y);

    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    bool isDiscretized() {
        return true;
    }

    virtual void matchSize();

   private:
    void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;

    int RobotType;
    OdomCallbackType OdomType;

    // ------------------------- Inflation -------------------------
    // Robot Param
    double RobotCostScalingFactor;
    double RobotInscribedRadius;
    double RobotInflationRadius;

    // Rival Param
    double RivalCostScalingFactor;
    double RivalInscribedRadius;
    double RivalInflationRadius;

    // double MaxDistance;
    double RivalRadiusDecline;
    double RivalInflationDecline;

    // Function
    void ExpandPointWithCircle(double x, double y, double Radius);

    // ------------------------- RobotPath -------------------------
    // Sub
    ros::Subscriber RobotPath_Sub;
    ros::Subscriber RobotOdom_Sub;

    nav_msgs::Path RobotPath;

    nav_msgs::Odometry RobotOdom_type0;
    geometry_msgs::PoseWithCovariance RobotOdom_type1;

    void RobotPath_CB(const nav_msgs::Path& Path);

    void RobotOdom_type0_CB(const nav_msgs::Odometry& Odom);
    void RobotOdom_type1_CB(const geometry_msgs::PoseWithCovarianceStamped& Odom);

    void Pot_CB(const obstacle_detector::Obstacles& Obstacle);
    // Time
    ros::Time RobotPathLastTime;
    ros::Time RobotOdomLastTime;
    // Time for clearup costmap
    ros::Time lastClearMap;
    double ClearMapPeriod;
    double RobotPathTimeout;
    double RobotOdomTimeout;
    bool isRobotPath;
    bool isRobotOdom;

    // Param
    int RobotPredictLength;

    // ------------------------- RivalOdom -------------------------
    // Sub
    ros::Subscriber RivalOdom_Sub[2];
    ros::Subscriber RivalObstacle_Sub;

    nav_msgs::Odometry RivalOdom[2];
    // obstacle_detector::Obstacles RivalObstacle;
    nav_msgs::Odometry RivalObstacle;
    nav_msgs::Odometry LastRivalObstacle;
    ros::Time LastUpdateRivalTime;
    double CalRivalVelInterval; 
    double CurRivalVelX, CurRivalVelY;
    double LastRivalPosX, LastRivalPosY;

    double RivalOdom_Resolution;
    double RivalOdom_PredictTime;
    double RivalOdom_MaxLength;

    void RivalOdom_CB(const nav_msgs::Odometry& Odom);
    void RivalOdom2_CB(const nav_msgs::Odometry& Odom);

    void RivalObstacle_CB(const nav_msgs::Odometry& Obstacle);


    // ------------------------- Pot -------------------------
    ros::Subscriber Pot_Sub;
    obstacle_detector::Obstacles Pot;
    std::string Pot_TopicName;
    double PotCostScalingFactor;
    bool isPot;

    // ------------------------- Predict Rival Traj -------------------------
    ros::Subscriber Circle_Sub;
    void Circle_CB(const geometry_msgs::Twist& Circle);
    double odom_x;
    double odom_y;
    double slope;
    double intercept;
    double se;
    double dir;
    bool isTrajCircle;
    ros::Time RivalTrajLastTime;

    // ------------------------- Functions -------------------------

    void InflatePredictPath(ROBOT_TYPE type);
    void InflatePoint(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius);

    // Timeout
    ros::Time RivalOdomLastTime[2];
    double RivalOdomTimeout;
    bool isRivalOdom[2];

    ros::Time RivalObstacleLastTime;
    double RivalObstacleTimeout;
    bool isRivalObstacle;
    // std::ofstream myfile;
};

}  // namespace path_layer_namespace

#endif