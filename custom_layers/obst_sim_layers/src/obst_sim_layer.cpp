#include<obst_sim_layers/obst_sim_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(obst_sim_layer_namespace::ObstSimLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;


namespace obst_sim_layer_namespace
{

ObstSimLayer::ObstSimLayer() {}

void ObstSimLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &ObstSimLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  obst_sim_sub = nh.subscribe("/robot/obstacle_position_array", 1, &ObstSimLayer::obstSimCB, this);
  expand_radius = 0.15;
}


void ObstSimLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void ObstSimLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  // mark_x_ = robot_x + cos(robot_yaw);
  // mark_y_ = robot_y + sin(robot_yaw);

  // *min_x = std::min(*min_x, mark_x_);
  // *min_y = std::min(*min_y, mark_y_);
  // *max_x = std::max(*max_x, mark_x_);
  // *max_y = std::max(*max_y, mark_y_);
}

void ObstSimLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  // unsigned int mx;
  // unsigned int my;
  // if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
  //   master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  // }
  // std::cout << "size: " << obstacle_pos.size() << std::endl;
  ROS_INFO_STREAM("[obst_sim_layer] size: " << obstacle_pos.size());
  for (int i = 0; i < obstacle_pos.size(); ++i) {
    memset(vis_, 0, sizeof(vis_));
    unsigned int mx;
    unsigned int my;
    // if (master_grid.worldToMap(obstacle_pos[i][0], obstacle_pos[i][1], mx, my)) {
    //   master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    // }
    std::pair<int, int> now;
    master_grid.worldToMap(obstacle_pos[i][0], obstacle_pos[i][1], mx, my);
    now.first = mx;
    now.second = my;
    std::queue<std::pair<int, int>> q;
    q.push(now);
    
    vis_[now.first][now.second] = true;
    double real_x = obstacle_pos[i][0], real_y = obstacle_pos[i][1];
    // int dist = pow(q.front().first - real_x, 2) + pow(q.front().second - real_y, 2);
    // std::cout << "dist:" << dist << " " << pow(expand_radius, 2) << std::endl;
    // ROS_INFO("[obst_sim_layer] dist: %d %d", dist, pow(expand_radius, 2));
    while (!q.empty()) {
      now = q.front();
      q.pop(); 
      master_grid.mapToWorld(now.first, now.second, real_x, real_y);
      if (pow(obstacle_pos[i][0] - real_x, 2) + pow(obstacle_pos[i][1] - real_y, 2) < pow(expand_radius, 2)) {
        // std::cout << "break" << std::endl;
        ROS_INFO("[obst_sim_layer] break");
        break;
      }
      // std::cout << "now " << now.first << " " << now.second << std::endl;
      ROS_INFO("[obst_sim_layer] now %d %d", now.first, now.second);
      for (int j = 0; j < 8; ++j) {
        int nxt_x = now.first + dir_[j][0];
        int nxt_y = now.second + dir_[j][1];
        if (master_grid.worldToMap(nxt_x, nxt_y, mx, my) && !vis_[nxt_x][nxt_y]) {
          ROS_INFO("[obst_sim_layer] vis %d %d", mx, my);
          master_grid.setCost(mx, my, LETHAL_OBSTACLE);
          q.push(std::make_pair(nxt_x, nxt_y));
          vis_[nxt_x][nxt_y] = true;
        }
      }
    }
    if(master_grid.worldToMap(obstacle_pos[i][0], obstacle_pos[i][1], mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
  }
}

void ObstSimLayer::obstSimCB(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  // std::cout << "get obstacle position" << std::endl;
  // clear the costmap
  // for (int i = 0; i < obstacle_pos.size(); ++i) {
  //   clear_radius(obstacle_pos[i][0], obstacle_pos[i][1], expand_radius);
  // } 
  obstacle_pos.clear();
  // std::vector<std::vector<double>> obstacle_pos;
  for(int i = 0; i < msg->poses.size(); i++)
  {
    std::vector<double> pos;
    pos.push_back(msg->poses[i].position.x);
    pos.push_back(msg->poses[i].position.y);
    obstacle_pos.push_back(pos);
  }
  ROS_INFO("[obst_sim_layer] get obstacle position %ld", obstacle_pos.size());
}

} // end namespace
