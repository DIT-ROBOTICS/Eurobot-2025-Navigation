// plugin
#include <pluginlib/class_list_macros.h>

// include
#include "path_layer/path_layer.h"

PLUGINLIB_EXPORT_CLASS(path_layer_namespace::PathLayer, costmap_2d::Layer)

namespace path_layer_namespace {

PathLayer::PathLayer() {
    // myfile.open ("/home/user/Eurobot2024-Navigation-ws/RivalOdom.txt");
}

PathLayer::~PathLayer() {
    // myfile.close();
}

void PathLayer::onInitialize() {
    ROS_INFO_STREAM("[path layer]" << "on initialize");
    ros::NodeHandle nh("~/" + name_);

    std::string RobotOdom_CB_TopicName;
    std::string RobotPath_CB_TopicName;
    std::string RivalOdom_CB_TopicName[2];
    std::string Rival_Obstacle_CB_TopicName;
    int temp_OdomType;

    // ---------------- Read YAML parameter ----------------
    nh.param("enabled", enabled_, true);
    nh.param("RobotType", RobotType, 1);
    nh.param("OdomCallbackType", temp_OdomType, 1);

    // Inflation
    nh.param("Inflation/Robot/CostScalingFactor", RobotCostScalingFactor, 10.0);
    nh.param("Inflation/Robot/InscribedRadius", RobotInscribedRadius, 0.1);
    nh.param("Inflation/Robot/InflationRadius", RobotInflationRadius, 0.3);
    nh.param("Inflation/Rival/CostScalingFactor", RivalCostScalingFactor, 10.0);
    nh.param("Inflation/Rival/InscribedRadius", RivalInscribedRadius, 0.2);
    nh.param("Inflation/Rival/InflationRadius", RivalInflationRadius, 0.35);
    nh.param("Inflation/Rival/InscribedRadiusDecline", RivalRadiusDecline, 0.001);
    nh.param("Inflation/Rival/InflationRadiusDecline", RivalInflationDecline, 0.9);
    nh.param("Inflation/Pot/CostScalingFactor", PotCostScalingFactor, 10.0);

    // Topic
    nh.param<std::string>("Topic/Robot/Odom", RobotOdom_CB_TopicName, "/robot/odom_obseleted");
    nh.param<std::string>("Topic/Robot/Path", RobotPath_CB_TopicName, "/move_base/GlobalPlanner/plan");
    nh.param<std::string>("Topic/Rival/Odom", RivalOdom_CB_TopicName[0], "/RivalOdom");
    // nh.param<std::string>("Topic/Rival/Odom2", RivalOdom_CB_TopicName[1], "/RivalOdom_2");
    nh.param<std::string>("Topic/Rival/Obstacle", Rival_Obstacle_CB_TopicName, "/rival/final_pose");
    nh.param<std::string>("Topic/Pot", Pot_TopicName, "/pot");

    // Timeout
    nh.param("Timeout/Robot/Odom", RobotOdomTimeout, 1.0);
    nh.param("Timeout/Robot/Path", RobotPathTimeout, 1.0);
    nh.param("Timeout/Rival/Odom", RivalOdomTimeout, 1.0);
    nh.param("Timeout/Rival/Obstacle", RivalObstacleTimeout, 1.0);

    // PredictLength
    nh.param("PredictLength/Robot/Path", RobotPredictLength, 1);
    nh.param("PredictLength/Rival/Odom", RivalOdom_PredictTime, 0.1);
    nh.param("PredictLength/Rival/MaxLength", RivalOdom_MaxLength, 1.0);
    nh.param("PredictLength/Rival/Resolution", RivalOdom_Resolution, 0.01);
    nh.param("PredictLength/Rival/CalRivalVelInterval", CalRivalVelInterval, 0.5);

    nh.param("ClearMapPeriod", ClearMapPeriod, 2.0);
    // ---------------- Read YAML parameter ----------------

    // Subscriber
    // RobotPath_Sub = nh.subscribe(RobotPath_CB_TopicName, 1000, &PathLayer::RobotPath_CB, this);
    Pot_Sub = nh.subscribe(Pot_TopicName, 1000, &PathLayer::Pot_CB, this);
    Circle_Sub = nh.subscribe("/robot/circle_fit", 1000, &PathLayer::Circle_CB, this);

    switch (temp_OdomType) {
        case 0:
            OdomType = nav_msgs_Odometry;
            // RobotOdom_Sub = nh.subscribe(RobotOdom_CB_TopicName, 1000, &PathLayer::RobotOdom_type0_CB, this);
            break;
        case 1:
            OdomType = geometry_msgs_PoseWithCovarianceStamped;
            // RobotOdom_Sub = nh.subscribe(RobotOdom_CB_TopicName, 1000, &PathLayer::RobotOdom_type1_CB, this);
            break;
    }

    RivalOdom_Sub[0] = nh.subscribe(RivalOdom_CB_TopicName[0], 1000, &PathLayer::RivalOdom_CB, this);
    // RivalOdom_Sub[1] = nh.subscribe(RivalOdom_CB_TopicName[1], 1000, &PathLayer::RivalOdom2_CB, this);
    RivalObstacle_Sub = nh.subscribe(Rival_Obstacle_CB_TopicName, 1000, &PathLayer::RivalObstacle_CB, this);
    LastUpdateRivalTime = ros::Time::now(); 
    CurRivalVelX = CurRivalVelY = 0;
    LastRivalPosX = LastRivalPosY = 0;

    // Init variable
    isRobotPath = isRobotOdom = isRivalOdom[0]  = isRivalObstacle = isTrajCircle = false;
    RobotPathLastTime = RivalOdomLastTime[0] = RivalObstacleLastTime = RivalTrajLastTime = ros::Time::now();

    current_ = true;
    default_value_ = costmap_2d::NO_INFORMATION;

    // Resize map
    matchSize();

    // Register layer
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&PathLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    lastClearMap = ros::Time::now();
}

void PathLayer::matchSize() {
    Costmap2D* master = layered_costmap_->getCostmap();
    ROS_INFO_STREAM("[path layer]" << "match size" << master->getSizeInCellsX() << " " << master->getSizeInCellsY() << " " << master->getResolution() << " " << master->getOriginX() << " " << master->getOriginY());
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
              master->getResolution(),
              master->getOriginX(), master->getOriginY());
}

void PathLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level) {
    enabled_ = config.enabled;
}

void PathLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                             double* min_x, double* min_y, double* max_x, double* max_y) {
    if (!enabled_)
        return;

    // Timeout.
    double CurrentTime = ros::Time::now().toSec();
    if (RobotPathTimeout != -1 && CurrentTime - RobotPathLastTime.toSec() > RobotPathTimeout)
        isRobotPath = false;
    if (RobotOdomTimeout != -1 && CurrentTime - RobotOdomLastTime.toSec() > RobotOdomTimeout)
        isRobotOdom = false;
    // for (int i = 0; i < 2; i++) {
    if (RivalOdomTimeout != -1 && CurrentTime - RivalOdomLastTime[0].toSec() > RivalOdomTimeout)
        isRivalOdom[0] = false;
    // }
    if (RivalObstacleTimeout != -1 && CurrentTime - RivalObstacleLastTime.toSec() > RivalObstacleTimeout)
        isRivalObstacle = false;
    if (RivalObstacleTimeout != -1 && CurrentTime - RivalTrajLastTime.toSec() > RivalObstacleTimeout) {
        isTrajCircle = false;
    }
    // Get the Costmap lock. (Optional)
    boost::unique_lock<mutex_t> lock(*(getMutex()));

    // Clean up the old costmap.
    if ((ros::Time::now() - lastClearMap).toSec() > ClearMapPeriod) {
        resetMaps();
        lastClearMap = ros::Time::now();
    }

    // Inflation Robot Odom
    if (isRobotOdom) {
        if (OdomType == nav_msgs_Odometry) {
            InflatePoint(RobotOdom_type0.pose.pose.position.x, RobotOdom_type0.pose.pose.position.y, costmap_2d::LETHAL_OBSTACLE, RobotInflationRadius, RobotCostScalingFactor, RobotInscribedRadius);
        } else {
            InflatePoint(RobotOdom_type1.pose.position.x, RobotOdom_type1.pose.position.y, costmap_2d::LETHAL_OBSTACLE, RobotInflationRadius, RobotCostScalingFactor, RobotInscribedRadius);
        }
    }

    // Inflation Robot Path
    if (isRobotPath) {
        InflatePredictPath(ROBOT_TYPE::ROBOT);
    }

    // Add Rival Path to costmap.
    if (isRivalOdom[0]) {
        InflatePredictPath(ROBOT_TYPE::RIVAL);
    }
    if (isRivalObstacle) {
        InflatePredictPath(ROBOT_TYPE::OBSTACLE);
    }
    if (isTrajCircle) {
        InflatePredictPath(ROBOT_TYPE::TRAJ_CIRCLE);
    }
    // for (int i = 0; i < Pot.circles.size(); ++i) {
    //     ROS_INFO_STREAM("[path layer]" << "inflate pot " << Pot.circles[i].center.x << " " << Pot.circles[i].center.y << " " << Pot.circles[i].radius);
    //     InflatePoint(Pot.circles[i].center.x, Pot.circles[i].center.y, 252, Pot.circles[i].true_radius, PotCostScalingFactor, Pot.circles[i].radius);
    // }
    InflatePredictPath(ROBOT_TYPE::POT);
    *min_x = 0.0;
    *max_x = getSizeInCellsX();
    *min_y = 0.0;
    *max_y = getSizeInCellsY();
}

void PathLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    // if (!enabled_ || !(isRobotOdom || isRobotPath || isRivalOdom[0] || isRivalOdom[1] || isRivalObstacle))
        // return;

    // Get the costmap lock.
    // ROS_INFO_STREAM("pathplanner updatecost");
    boost::unique_lock<mutex_t> lock(*(getMutex()));

    // updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    updateWithMax(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());
}

void PathLayer::ExpandPointWithCircle(double x, double y, double Radius) {
    for (int angle = 0; angle < 360; angle++) {
        double Rad = angle * M_PI / 180.0;
        double mark_x = x + Radius * cos(Rad);
        double mark_y = y + Radius * sin(Rad);
        unsigned int mx;
        unsigned int my;
        if (worldToMap(mark_x, mark_y, mx, my)) {
            setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
        }
    }
}

void PathLayer::InflatePredictPath(ROBOT_TYPE type) {
    if (type == ROBOT_TYPE::ROBOT) {
        for (int i = 0; i < RobotPredictLength; i++) {
            if (RobotPath.poses.size() <= i)
                break;

            double mark_x = RobotPath.poses[i].pose.position.x;
            double mark_y = RobotPath.poses[i].pose.position.y;
            unsigned int mx;
            unsigned int my;
            if (worldToMap(mark_x, mark_y, mx, my)) {
                InflatePoint(mark_x, mark_y, 252, RobotInflationRadius, RobotCostScalingFactor, RobotInscribedRadius);
            }
        }
    } else if (type == ROBOT_TYPE::RIVAL) {
        const char idx = 0;

        double mark_x = RivalOdom[idx].pose.pose.position.x;
        double mark_y = RivalOdom[idx].pose.pose.position.y;

        // Rival Object
        InflatePoint(mark_x, mark_y, costmap_2d::LETHAL_OBSTACLE, RivalInflationRadius, RivalCostScalingFactor, RivalInscribedRadius);

        double len = sqrt(pow(RivalOdom[idx].twist.twist.linear.x, 2) + pow(RivalOdom[idx].twist.twist.linear.y, 2)) * RivalOdom_PredictTime;
        if (len == 0.0) {
            return;
        } else if (len > RivalOdom_MaxLength) {
            len = RivalOdom_MaxLength;
        }

        double theta = 0.0;
        if (RivalOdom[idx].twist.twist.linear.x == 0.0) {
            theta = RivalOdom[idx].twist.twist.linear.y >= 0 ? M_PI_2 : -M_PI_2;
        } else {
            theta = std::atan(RivalOdom[idx].twist.twist.linear.y / RivalOdom[idx].twist.twist.linear.x);
            if (RivalOdom[idx].twist.twist.linear.x <= 0) {
                theta += M_PI;
            }
        }

        const double IncX = RivalOdom_Resolution * std::cos(theta);
        const double IncY = RivalOdom_Resolution * std::sin(theta);
        double InflationRadius = RivalInflationRadius;
        double InscribedRadius = RivalInscribedRadius;

        unsigned int mx;
        unsigned int my;
        for (double i = 0.0; i < len; i += RivalOdom_Resolution) {
            mark_x += IncX;
            mark_y += IncY;
            if (worldToMap(mark_x, mark_y, mx, my)) {
                InflatePoint(mark_x, mark_y, 252, InflationRadius, RivalCostScalingFactor, InscribedRadius);
            } else {
                break;
            }
            // InflationRadius *= RivalRadiusDecline;
            InscribedRadius *= RivalRadiusDecline;
        }
    } else if (type == ROBOT_TYPE::OBSTACLE) {
        double mark_x = RivalObstacle.pose.pose.position.x;
        double mark_y = RivalObstacle.pose.pose.position.y;

        // Rival Object
        InflatePoint(mark_x, mark_y, costmap_2d::LETHAL_OBSTACLE, RivalInflationRadius, RivalCostScalingFactor, RivalInscribedRadius);

        double len = sqrt(pow(RivalObstacle.twist.twist.linear.x, 2) + pow(RivalObstacle.twist.twist.linear.y, 2)) * RivalOdom_PredictTime;
        if (len == 0.0) {
            return ;
        } else if (len > RivalOdom_MaxLength) {
            len = RivalOdom_MaxLength;
        }

        double theta = 0.0;
        if (RivalObstacle.twist.twist.linear.x == 0.0) {
            theta = RivalObstacle.twist.twist.linear.y >= 0 ? M_PI_2 : -M_PI_2;
        } else {
            theta = std::atan(RivalObstacle.twist.twist.linear.y / RivalObstacle.twist.twist.linear.x);
            if (RivalObstacle.twist.twist.linear.x <= 0) {
                theta += M_PI;
            }
        }

        const double IncX = RivalOdom_Resolution * std::cos(theta);
        const double IncY = RivalOdom_Resolution * std::sin(theta);
        double InflationRadius = RivalInflationRadius;
        double InscribedRadius = RivalInscribedRadius;

        unsigned int mx;
        unsigned int my;
        for (double i = 0.0; i < len; i += RivalOdom_Resolution) {
            mark_x += IncX;
            mark_y += IncY;
            if (worldToMap(mark_x, mark_y, mx, my)) {
                InflatePoint(mark_x, mark_y, 252, InflationRadius, RivalCostScalingFactor, InscribedRadius);
            } else {
                break;
            }
            // InflationRadius *= ;
            InscribedRadius *= RivalRadiusDecline;
        }
    }
    else if (type == ROBOT_TYPE::POT) {
        for (auto pot : Pot.circles) {
            double mark_x = pot.center.x;
            double mark_y = pot.center.y;
            InflatePoint(mark_x, mark_y, 252, pot.true_radius, PotCostScalingFactor, pot.radius);
        }
    }
    else if (type == ROBOT_TYPE::TRAJ_CIRCLE) {
        int goal_steps = (se / resolution_);
        // ROS_INFO_STREAM("steps " << se << " " << goal_steps);
        double InscribedRadius = RivalInscribedRadius;
        double InflationRadius = RivalInflationRadius;
        double mark_x = 0, mark_y = 0;
        double cos_th = 1.0 / sqrt(1 + pow(slope, 2));
        double sin_th = slope / sqrt(1 + pow(slope, 2));


        unsigned int mx, my; 
        if (goal_steps == 0 && worldToMap(odom_x, odom_y, mx, my)) {
            InflatePoint(odom_x, odom_y, 254, InflationRadius, RivalCostScalingFactor, InscribedRadius);
        }
        // InscribedRadius *= pow(RivalRadiusDecline, std::min(goal_steps, 30));
        InflationRadius *= pow(RivalInflationDecline, goal_steps);
        for (int i = 0; i < goal_steps; i += 1) {
            mark_x += resolution_ * cos_th * dir;
            mark_y += resolution_ * sin_th * dir;
            if (worldToMap(mark_x + odom_x, mark_y + odom_y, mx, my)) {
                InflatePoint(mark_x + odom_x, mark_y + odom_y, 254, InflationRadius, RivalCostScalingFactor, InscribedRadius);
            }
            InscribedRadius /= RivalRadiusDecline;
            InflationRadius /= RivalInflationDecline;
        } 
    }
}

void PathLayer::InflatePoint(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius) {
    // MaxDistance = 6.22258 / CostScalingFactor + InscribedRadius;  // 6.22258 = -ln(0.5/252.0)
    // MaxDistance = (double)(((int)(MaxDistance / resolution_) + resolution_) * resolution_);
    // ROS_INFO_STREAM("[path layer]" << "inflate point");
    double MaxX = x + InflationRadius;
    double MinX = x - InflationRadius;
    double MaxY;
    double MinY;

    double mark_x = 0.0;
    double mark_y = 0.0;
    unsigned int mx;
    unsigned int my;

    double cost;
    double Distance;

    for (double currentPointX = MinX; currentPointX <= MaxX; currentPointX += resolution_) {
        mark_x = currentPointX;
        MaxY = y + sqrt(pow(InflationRadius, 2) - pow(fabs(currentPointX - x), 2));
        MinY = 2 * y - MaxY;

        for (double currentPointY = MinY; currentPointY <= MaxY; currentPointY += resolution_) {
            mark_y = currentPointY;
            if (worldToMap(mark_x, mark_y, mx, my)) {
                Distance = sqrt(pow(fabs(x - currentPointX), 2) + pow(fabs(y - currentPointY), 2));

                cost = round(252 * exp(-CostScalingFactor * (Distance - InscribedRadius)));
                cost = std::max(std::min(cost, MaxCost), 0.0);

                // ROS_INFO_STREAM("pathlayer hello" << mx << " " << my << " " << cost);
                if (getCost(mx, my) != costmap_2d::NO_INFORMATION) {
                    setCost(mx, my, std::max((unsigned char)cost, getCost(mx, my)));
                } else {
                    setCost(mx, my, cost);
                }
            }
        }
    }
}

// ---------------------------- Callback ---------------------------- //

void PathLayer::RobotPath_CB(const nav_msgs::Path& Path) {
    this->RobotPath = Path;
    isRobotPath = true;
    RobotPathLastTime = ros::Time::now();
}

void PathLayer::RobotOdom_type0_CB(const nav_msgs::Odometry& Odom) {
    this->RobotOdom_type0 = Odom;
    isRobotOdom = true;
    RobotOdomLastTime = ros::Time::now();
}
void PathLayer::RobotOdom_type1_CB(const geometry_msgs::PoseWithCovarianceStamped& Odom) {
    this->RobotOdom_type1 = Odom.pose;
    isRobotOdom = true;
    RobotOdomLastTime = ros::Time::now();
}

void PathLayer::RivalOdom_CB(const nav_msgs::Odometry& Odom) {
    RivalOdom[0] = Odom;
    isRivalOdom[0] = true;
    RivalOdomLastTime[0] = ros::Time::now();
}

void PathLayer::Circle_CB(const geometry_msgs::Twist& Circle) {
    odom_x = Circle.linear.x;
    odom_y = Circle.linear.y;
    dir = Circle.linear.z;
    slope = Circle.angular.x;
    intercept = Circle.angular.y;
    se = Circle.angular.z;
    isTrajCircle = true;
    RivalTrajLastTime = ros::Time::now();
}

void PathLayer::RivalObstacle_CB(const nav_msgs::Odometry& Obstacle) {
    // RivalObstacle = Obstacle;


    // if (LastRivalPosX == 0 && LastRivalPosY == 0) {
    //     LastRivalPosX = Obstacle.pose.pose.position.x;
    //     LastRivalPosY = Obstacle.pose.pose.position.y;
    //     LastUpdateRivalTime = ros::Time::now();
    // }
    // double time_passed = (ros::Time::now() - LastUpdateRivalTime).toSec();
    // if (time_passed > CalRivalVelInterval) {
    //     CurRivalVelX = (Obstacle.pose.pose.position.x - LastRivalPosX) / time_passed;
    //     CurRivalVelY = (Obstacle.pose.pose.position.y - LastRivalPosY) / time_passed; 
        
    //     LastRivalPosX = Obstacle.pose.pose.position.x;
    //     LastRivalPosY = Obstacle.pose.pose.position.y;
    //     LastUpdateRivalTime = ros::Time::now();
    // }
    // RivalObstacle.pose.pose.position.x = LastRivalPosX;
    // RivalObstacle.pose.pose.position.y = LastRivalPosY;
    // RivalObstacle.twist.twist.linear.x = CurRivalVelX;
    // RivalObstacle.twist.twist.linear.y = CurRivalVelY;


    // if (fabs(RivalObstacle.twist.twist.linear.x) < 0.1) {
    //     RivalObstacle.twist.twist.linear.x = 0;
    // }
    // if (fabs(RivalObstacle.twist.twist.linear.y) < 0.1) {
    //     RivalObstacle.twist.twist.linear.y = 0;
    // }


    // isRivalObstacle = true;
    // RivalObstacleLastTime = ros::Time::now();
}

void PathLayer::Pot_CB(const obstacle_detector::Obstacles& Obstacle) {
    Pot = Obstacle;
    if (Pot.circles.size() == 0) isPot = false;
    else isPot = true;
}
}  // namespace path_layer_namespace