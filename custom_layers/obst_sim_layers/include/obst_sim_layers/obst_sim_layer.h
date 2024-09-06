#ifndef OBST_SIM_LAYER_H_
#define OBST_SIM_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>

namespace obst_sim_layer_namespace
{

class ObstSimLayer : public costmap_2d::Layer
{
public:
  ObstSimLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  double mark_x_, mark_y_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  ros::Subscriber obst_sim_sub;
  void obstSimCB(const geometry_msgs::PoseArray::ConstPtr& msg);
  std::vector<std::vector<double>> obstacle_pos;
  double expand_radius;
  // void clear_radius
  int dir_[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,-1},{-1,1},{1,-1}};
  // std::map<std::pair<int, int>, bool> vis;
  bool vis_[305][205];
};
}
#endif