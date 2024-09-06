#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/time.h"

namespace simple_layer_namespace
{

enum class ODOM_CALLBACK_TYPE {
    nav_msgs_Odometry = 0,
    geometry_msgs_PoseWithCovarianceStamped
};

/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
      index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};

class Obstacle
{
public:
  Obstacle();
  Obstacle(double x, double y, ros::Time t);
  void set_x(double x){
    this->x_ = x;
  }
  void set_y(double y){
    this->y_ = y;
  }
  double get_x() const{
    return x_;
  }
  double get_y() const{
    return y_;
  }
  ros::Time get_time() const{
    return stamp_;
  }

private:
  double x_;
  double y_;
  double z_;
  double vx_;
  double vy_;
  double w;
  double yaw_;
  
  std::vector<double> obs_pos[20];

  ros::Time stamp_;

};
//  public costmap_2d::CostmapLayer
// public costmap_2d::Layer, public costmap_2d::Costmap2D
class GridLayer : public costmap_2d::CostmapLayer
{
public:
  GridLayer();

  /**
   * @brief callback function for adding obstacles
   * @param poses which type is geometry_msgs/PoseArray
   * poses.header.frame_id = which source_type is. ex: "camera", "tracker", "lidar"
   * poses.header.stamp = ros::Time::now()
   * poses.poses[??].x = position_x 
   * poses.poses[??].y = ...
   */
  void obsCallback(const geometry_msgs::PoseArray& poses);
  void poseType0Callback(const nav_msgs::Odometry& pose);
  void poseType1Callback(const geometry_msgs::PoseWithCovarianceStamped &pose);

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  std::vector<Obstacle> obstacle_pos_;
  ros::Subscriber sub_;
  ros::Subscriber sub_ekf_;
  ros::NodeHandle g_nh_;
  double update_frequency_;

  double inflation_radius_;
  double inscribed_radius_;
  double cost_factor_;

  double clear_radius_;
  int odom_callback_type_temp_;
  ODOM_CALLBACK_TYPE odom_callback_type_;
  std::string odom_topic_;

  ros::Timer timer_;

  /** @brief tolerance that we think two obstacle are the same obstacle
   *  there are two type of them: sample and rival (other team's robot)
   */
  double tolerance_;
  double threshold_time_;

  void inflate(double x, double y);
  double** min_dist_check;

  double ekf_x_;
  double ekf_y_;
};
}
#endif