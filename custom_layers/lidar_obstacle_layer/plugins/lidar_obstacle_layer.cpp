#include <lidar_obstacle_layer/lidar_obstacle_layer.h>
#include <pluginlib/class_list_macros.h>
#include <obstacle_detector/Obstacles.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/time.h"
#define _USE_MATH_DEFINES
#include <math.h>

PLUGINLIB_EXPORT_CLASS(lidar_obstacle_layer_namespace::LidarObstacleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace lidar_obstacle_layer_namespace
{

LidarObstacleLayer::LidarObstacleLayer() 
{
}

void LidarObstacleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);

  // read YAML parameter
  std::string _;
  nh.param("obstacle_topic", obstacle_topic_, _);
  nh.param("obstacle_radius", obstacle_radius_, 0.1);
  nh.param("expand_division", div_, 30.0);

  sub_ = g_nh_.subscribe(obstacle_topic_, 10, &LidarObstacleLayer::obsCallback, this);
    
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &LidarObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void LidarObstacleLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void LidarObstacleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void LidarObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;


  resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());

  for(int i=0; i<obstacles_.circles.size(); i++){
    double mark_x = obstacles_.circles[i].center.x;
    double mark_y = obstacles_.circles[i].center.y;
    
    expandObstacle(mark_x, mark_y, obstacle_radius_);

    *min_x = std::min(*min_x, mark_x - obstacle_radius_);
    *min_x = std::max(*min_x, 0.0);
    *min_y = std::min(*min_y, mark_y - obstacle_radius_);
    *min_y = std::max(*min_y, 0.0);

    *max_x = std::max(*max_x, mark_x + obstacle_radius_);
    *max_x = std::min(*max_x, getSizeInMetersX());
    *max_y = std::max(*max_y, mark_y + obstacle_radius_);
    *max_y = std::min(*max_y, getSizeInMetersY());

    // ROS_INFO("X:%f Y:%f", getSizeInMetersX(), getSizeInMetersY());
  }
  
}

void LidarObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  // if (!enabled_)
  //   return;

  // for (int j = min_j; j < max_j; j++)
  // {
  //   for (int i = min_i; i < max_i; i++)
  //   {
  //     int index = getIndex(i, j);
  //     if (costmap_[index] == NO_INFORMATION)
  //       continue;
  //     master_grid.setCost(i, j, costmap_[index]); 
  //   }
  // }
  
  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  // updateWithAddition(master_grid, min_i, min_j, max_i, max_j);
  // updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void LidarObstacleLayer::obsCallback(const obstacle_detector::Obstacles& obs)
{

  obstacles_ = obs;
  // ROS_INFO("Circles: %f %f\n", obstacles.circles[0].center.x, obstacles.circles[0].center.y);
}

void LidarObstacleLayer::expandObstacle(double x, double y, double radius)
{
  for (int i=0; i<div_; i++){
    double mark_x = x + radius*cos((2*M_PI/div_)*i);
    double mark_y = y + radius*sin((2*M_PI/div_)*i);
    unsigned int mx;
    unsigned int my;
    if(worldToMap(mark_x, mark_y, mx, my)){
      setCost(mx, my, LETHAL_OBSTACLE);
    }
    // std::cout << mx << " " << my << std::endl;
  }
}

} // end namespace