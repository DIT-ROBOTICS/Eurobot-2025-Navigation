#include <simple_layers/grid_layer.h>
#include <pluginlib/class_list_macros.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/time.h"
#include "simple_layers/grid_layer.h"
#include <cmath>


PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

namespace simple_layer_namespace
{
  Obstacle::Obstacle(double x, double y, ros::Time t)
  {
    x_ = x;
    y_ = y;
    stamp_ = t;
  }

  GridLayer::GridLayer() 
  {
  }

  void GridLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_);
    // read YAML parameter

    nh.param("inflation_radius", inflation_radius_, 0.25);
    nh.param("inscribed_radius", inscribed_radius_, 0.15);
    nh.param("cost_factor", cost_factor_, 3.0);

    nh.param("tolerance", tolerance_, 0.1);
    nh.param("threshold_time", threshold_time_, 0.1);
    nh.param("clear_radius", clear_radius_, 0.2);

    nh.param("odom_callback_type", odom_callback_type_temp_, 0);
    nh.param<std::string>("odom_topic", odom_topic_, "ekf_pose");
    ROS_INFO("%d, %s", odom_callback_type_temp_, odom_topic_.c_str());

    if (odom_callback_type_temp_ == 0) {
        odom_callback_type_ = ODOM_CALLBACK_TYPE::nav_msgs_Odometry;
    } else if (odom_callback_type_temp_ == 1){
        odom_callback_type_ = ODOM_CALLBACK_TYPE::geometry_msgs_PoseWithCovarianceStamped;
    }

    sub_ = g_nh_.subscribe("/obstacle_position_array", 10, &GridLayer::obsCallback, this);
    if(odom_callback_type_ == ODOM_CALLBACK_TYPE::nav_msgs_Odometry){
      sub_ekf_ = g_nh_.subscribe(odom_topic_, 10, &GridLayer::poseType0Callback, this);  
    }else if(odom_callback_type_ == ODOM_CALLBACK_TYPE::geometry_msgs_PoseWithCovarianceStamped){
      sub_ekf_ = g_nh_.subscribe(odom_topic_, 10, &GridLayer::poseType1Callback, this);
    }

    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    // dynamically declare array
    int ny = getSizeInCellsY();
    int nx = getSizeInCellsX();
    min_dist_check = new double* [nx];
    for(int i=0; i<nx; i++) min_dist_check[i] = new double[ny];

    // initialize 
    for(int i = 0; i < nx; i++){
      for(int j = 0; j < ny; j++) min_dist_check[i][j] = DBL_MAX;
    }

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
        &GridLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void GridLayer::obsCallback(const geometry_msgs::PoseArray& poses)
  {
    // ROS_INFO("clear");
    obstacle_pos_.clear();
    unsigned int obstacle_num = poses.poses.size();
    for (int i = 0; i < obstacle_num; i++){
      Obstacle obs(poses.poses[i].position.x, poses.poses[i].position.y, poses.header.stamp);
      // ROS_INFO("%f, %f", obs.get_x(), ekf_x_);
      if(hypot(obs.get_x() - ekf_x_, obs.get_y() - ekf_y_) < clear_radius_){
        ROS_INFO("[GridLayer]: clear obstacle [x:%f, y:%f]", obs.get_x(), obs.get_y());
        continue;
      }
      obstacle_pos_.push_back(obs);
    }
    // ROS_INFO("size %d", obstacle_pos_.size());
  }

  void GridLayer::poseType0Callback(const nav_msgs::Odometry &pose)
  {
    ekf_x_ = pose.pose.pose.position.x;
    ekf_y_ = pose.pose.pose.position.y;
  }

  void GridLayer::poseType1Callback(const geometry_msgs::PoseWithCovarianceStamped &pose)
  {
    ekf_x_ = pose.pose.pose.position.x;
    ekf_y_ = pose.pose.pose.position.y;
  }
  
  void GridLayer::matchSize()
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }

  void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }

  void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                            double* min_y, double* max_x, double* max_y)
  {
    if (!enabled_)
      return;

    resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());

    if (obstacle_pos_.size() == 0)
      return;

    // clear check dist array with MAX value but ignore -1(254)
    int ny = getSizeInCellsY();
    int nx = getSizeInCellsX();

    for(int i = 0; i < nx; i++){
      for(int j = 0; j < ny; j++){
        min_dist_check[i][j] = (min_dist_check[i][j] == -1) ? -1 : DBL_MAX;
      }
    }

    for(int i = 0; i<obstacle_pos_.size(); i++){
      double mark_x = obstacle_pos_[i].get_x();
      double mark_y = obstacle_pos_[i].get_y();
      inflate(mark_x, mark_y); // Inflate a radius!
      
      // Surround the most outside boundaries and do the local update!
      // min_x, y
      // *min_x = std::min(*min_x, mark_x - inflation_radius_);
      // *min_x = std::max(*min_x, 0.0);
      // *min_y = std::min(*min_y, mark_y - inflation_radius_);
      // *min_y = std::max(*min_y, 0.0);
      //max_x, y
      // *max_x = std::max(*max_x, mark_x + inflation_radius_);
      // *max_x = std::min(*max_x, getSizeInMetersX());
      // *max_y = std::max(*max_y, mark_y + inflation_radius_);
      // *max_y = std::min(*max_y, getSizeInMetersY());

      // ROS_INFO("i: %d, x:%f, y:%f", i,  mark_x, mark_y);
    }
    *min_x = 0;
    *min_y = 0;
    *max_x = getSizeInMetersX();
    *max_y = getSizeInMetersY();
    // ROS_INFO("finished");
  }

  void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
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
    boost::unique_lock<mutex_t> lock(*(getMutex()));
    // Get static map's info of LETHAL_OBSTACLE and marked -1
    int ny = getSizeInCellsY();
    int nx = getSizeInCellsX();

    unsigned char* master_array = master_grid.getCharMap();
    for(int i = 0; i < nx; i++){
      for(int j = 0; j < ny; j++){
        int idx = getIndex(i, j);
        if(master_array[idx] == LETHAL_OBSTACLE){
          min_dist_check[i][j] = -1;
          // ROS_INFO("LETHAL");
        }
      }
    } 
    
    // updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
    // updateWithAddition(master_grid, min_i, min_j, max_i, max_j);
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }

  void GridLayer::inflate(double x, double y)
  {
    // Change inflation radius unit into pixel(resolution = 0.01 m/pxl)
    int inflation_pixel = 100 * inflation_radius_;
    double d = 0;
    double distance_now = 0;
    unsigned int mx;
    unsigned int my;
    // Get mark_x, mark_y in pxl
    worldToMap(x, y, mx, my);
    int inf_cost = LETHAL_OBSTACLE;
    int max_cost = LETHAL_OBSTACLE;
    // Set the transversing origin on the top-left corner!
    int i_start = mx - inflation_pixel;
    int j_start = my - inflation_pixel;
    int i_end = mx+inflation_pixel + 1;
    int j_end = my+inflation_pixel + 1;

    // transverse x
    for (int i = i_start; i < i_end; i++){
      // transverse y
      for (int j = j_start; j < j_end; j++){
        // check if the pxl coord is in the map
        if(i >= 0 && j >= 0 && i < getSizeInCellsX() && j < getSizeInCellsY()){
          // leave LETHAL_OBSTACLE cell unaffected
          if(min_dist_check[i][j] == -1){
            continue;
          }
          // set cost regarding the sensor info
          else{
            // calculate the pixel-to-center distance(in pxl = 1cm)
            distance_now = getResolution() * sqrt(pow((int)mx - i, 2) + pow((int)my - j, 2));
            // keep the highest cost in a cell and neglect the same cost(dist)
            if(distance_now < min_dist_check[i][j]){
              min_dist_check[i][j] = distance_now;
              d = min_dist_check[i][j];
            }
            else{
              continue;
            }
            // case 1: In between the inflation and inscribed radius
            if(inscribed_radius_ <= d && d <= inflation_radius_){
              inf_cost = round(max_cost * exp(-cost_factor_ * (d - inscribed_radius_)));
              unsigned char INFLATED_COST = inf_cost;
              setCost(i, j, INFLATED_COST);
            }
            // case 2: Inside the inscribed radius, center included
            else if(0 < d && d < inscribed_radius_){
              setCost(i, j, max_cost); // <-> LETHAL_OBSTACLE
            }
            // case 3: center point is LETHAL
            else if(d == 0){
              setCost(i, j, max_cost);
            }
            // case 4: Outside inflation but in the outscribed square
            else if(inflation_radius_ <= d && d <= sqrt(2) * inflation_radius_){
              setCost(i, j, FREE_SPACE);
            }
            // case 4: Error
            else{
              ROS_INFO("GridLayer: Inflating Error!");
              // ROS_INFO("%d %d %d %d", i, j, mx, my);
              ROS_INFO("%lf", d);
            }
          }
        }
        // Outside the map!
        else{
          continue;
        }
      }
    }
  }

} // end namespace