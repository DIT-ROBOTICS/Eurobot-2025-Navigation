#ifndef LIDAR_OBSTACLE_LAYER_H_
#define LIDAR_OBSTACLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <obstacle_detector/Obstacles.h>
#include "geometry_msgs/PoseArray.h"
#include "ros/time.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace lidar_obstacle_layer_namespace
{
// public costmap_2d::CostmapLayer
// public costmap_2d::Layer, public costmap_2d::Costmap2D
class LidarObstacleLayer : public costmap_2d::CostmapLayer
{
public:
  LidarObstacleLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();
  void obsCallback(const obstacle_detector::Obstacles& obs);
  
  /// @brief expand the center to the radius
  /// @param config 
  /// @param level 
  void expandObstacle(double x, double y, double radius);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  ros::Subscriber sub_;
  ros::NodeHandle g_nh_;
  double update_frequency_;
  std::string obstacle_topic_;
  double obstacle_radius_;
  double div_;

  obstacle_detector::Obstacles obstacles_;
};
}
#endif