#include <pluginlib/class_list_macros.h>
#include "path_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(path_planner::PathPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace path_planner {

PathPlanner::PathPlanner (){
}


PathPlanner::PathPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}


void PathPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
        costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
        costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_

        // initialize other planner parameters
        ros::NodeHandle private_nh("~/" + name);
        world_model_ = new base_local_planner::CostmapModel(*costmap_);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        make_plan_srv_ = private_nh.advertiseService("make_plan", &PathPlanner::makePlanService, this);

        frame_id_ = costmap_ros_->getGlobalFrameID();

        initialized_ = true;
    }
    else
        ROS_WARN("This planner has already been initialized... doing nothing");
}

// double PathPlanner::footprintCost(double x_i, double y_i, double theta_i){
//     if(!initialized_){
//         ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
//         return -1.0;
//     }

//     std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
//     //if we have no footprint... do nothing
//     if(footprint.size() < 3)
//         return -1.0;

//     //check if the footprint is legal
//     double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
//     return footprint_cost;
// }

void PathPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value){
    unsigned char* pc = costarr;
    for(int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for(int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for(int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for(int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

bool PathPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan ){
    if(!initialized_){
        ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
        return false;
    }
    boost::mutex::scoped_lock lock(mutex_);
    // ROS_INFO_STREAM("[path_planner]" << __LINE__ << " [makePlan] start " << ros::Time::now());
    // ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();
    
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
        ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
        return false;
    }

    x_cell_sz_ = costmap_->getSizeInCellsX();
    y_cell_sz_ = costmap_->getSizeInCellsY();
    // ROS_INFO_STREAM("[path_planner]" << __LINE__ << " cell_size: " << x_cell_sz_ << " " << y_cell_sz_);
    // ROS_INFO_STREAM("[path_planner]" << "resolution " << costmap_->getResolution());
    // ROS_INFO_STREAM("[path_planner]" << __LINE__ << " get size in x:" << costmap_->getSizeInMetersX() << " y: " << costmap_->getSizeInMetersY());

    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    
    // tf2::Quaternion q();
    // tf2::fromMsg(start.pose.orientation, q);
    // tf2::Matrix3x3 qt(q());   
    // double _, start_yaw;
    // qt.getRPY(_, _, start_yaw); 

    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    // tf2::fromMsg(goal.pose.orientation, q);
    // qt = tf2::Matrix3x3(q());
    // double goal_yaw;
    // qt.getRPY(_, _, goal_yaw);

    outlineMap(costmap_->getCharMap(), x_cell_sz_, y_cell_sz_, costmap_2d::LETHAL_OBSTACLE);
    unsigned int start_mx, start_my, goal_mx, goal_my;
    costmap_->worldToMap(start_x, start_y, start_mx, start_my);
    costmap_->worldToMap(goal_x, goal_y, goal_mx, goal_my);
    // if (costmap_->getCost(goal_mx, goal_my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE * 0.6) {
    //     ROS_WARN("[path_planner]The goal is set to be in an obstacle, please reset the goal!");
    //     plan.clear();
    //     plan_.clear();
    //     return false;
    // }
    // ROS_INFO_STREAM("[path_planner]" << __LINE__ << " start: " << start_mx << "|" << start_my << " " << goal_mx << " " << goal_my);
    clearRobotCell(start_mx, start_my);
    bool res;
    res = AStarSearch(Node(start_mx, start_my), Node(goal_mx, goal_my));
    plan = plan_;
    // ROS_INFO_STREAM("[path_planner>>>>]" << __LINE__ << " Plan size: " << plan.size());
    publishPlan(plan);
    return res;
}

void PathPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }
    // ROS_INFO_STREAM("[path_planner]" << __LINE__ << "Published Plan!!! ");
    // plan_pub_.publish(gui_path);
}

void PathPlanner::clearRobotCell(int mx, int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
    // int clear_start_x = mx - 14;
    // int clear_start_y = my - 14;
    // int clear_end_x = mx + 14;
    // int clear_end_y = my + 14;
    // for (int i = clear_start_x; i <= clear_end_x; ++i) {
    //     for (int j = clear_start_y; j <= clear_end_y; ++j) {
    //         if (i < 0 || i >= x_cell_sz_ || j < 0 || j >= y_cell_sz_) continue;
    //         if (pow(i - mx, 2) + pow(j - my, 2) <= 14*14) {
    //             costmap_->setCost(i, j, costmap_2d::FREE_SPACE);
    //         }
    //     }
    // }
}

bool PathPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    // ROS_INFO_STREAM("[path_planner]" << " makePlanService start " << ros::Time::now());
    // if (!mutex_.try_lock()) {
        // return false;
    // }
    ros::Time start_time = ros::Time::now();
    resp.plan.poses.clear();
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;
    // resp.plan.poses.resize(plan_.size());
    // for (int i = 0; i < plan_.size(); ++i) {
    //     resp.plan.poses[i] = plan_[i];
    // }
    // ROS_INFO_STREAM("[path_planner]" << " plan time " << (ros::Time::now()-start_time).toSec());
    // mutex_.unlock();
    // ROS_INFO_STREAM("[path_planner]" << " makePlanService end " << ros::Time::now());
    return true;
}

bool PathPlanner::AStarSearch(Node start, Node goal) {
    plan_.clear();
    std::priority_queue<Node> pq;
    // for (int i = 0; i <= 300; ++i) {
    memset(from_x_, -1, sizeof(from_x_));
    memset(from_y_, -1, sizeof(from_y_));
    memset(last_dir_idx_, -1, sizeof(last_dir_idx_));
    memset(gScore, -1, sizeof(gScore));
    // }
    // ROS_INFO_STREAM("[path_planner]" << __LINE__ << " start: " << start.x << " " << start.y << "|" << goal.x << " " << goal.y);
    pq.push(start);
    int cnt = 60000;
    // TODO: size of from_x_, from_y_ are not dynamic allocated
    from_x_[(int)start.x][(int)start.y] = 0;
    from_y_[(int)start.x][(int)start.y] = 0;
    while (!pq.empty() && cnt--) {

        Node cur = pq.top();
        // if (cur.heuristic < 0) 
            // ROS_WARN_STREAM("[path_planner]" << "cost " << cur.heuristic << " might be overflow");
        Node cur_world;
        costmap_->mapToWorld(cur.x, cur.y, cur_world.x, cur_world.y);
        // ROS_INFO_STREAM("[path_planner]" << __LINE__ << " cur: " << cur.x << " " << cur.y);
        pq.pop();
        if (cur.costmap_cost >= 250) continue;
        std::vector<geometry_msgs::PoseStamped> tmp_plan;
        if (cur.x == goal.x && cur.y == goal.y) {
            int cost_total = 0;
            // ROS_INFO_STREAM("[path_planner]" << __LINE__ << " Find path!");
            double now_from_x = cur.x;
            double now_from_y = cur.y;
            geometry_msgs::PoseStamped push_pose;
            push_pose.header.frame_id = frame_id_;
            push_pose.pose.orientation.w = 1.0;
            int tmp = 60000;
            while (now_from_x != 0 && now_from_y != 0 && tmp--) {
                cost_total += costmap_->getCost(now_from_x, now_from_y);
                double wx, wy;
                costmap_->mapToWorld(now_from_x, now_from_y, wx, wy);
                push_pose.pose.position.x = wx;
                push_pose.pose.position.y = wy; 
                // plan_.push_back(push_pose);
                tmp_plan.push_back(push_pose);
                int tmp_x = now_from_x;
                int tmp_y = now_from_y;
                now_from_x = from_x_[tmp_x][tmp_y];
                now_from_y = from_y_[tmp_x][tmp_y];
            }
            if (tmp == 0) return false;
            std::reverse(tmp_plan.begin(), tmp_plan.end());

            // Pos Smoothing
            plan_.clear();
            geometry_msgs::PoseStamped tk = tmp_plan[0];
            plan_.push_back(tk);
            geometry_msgs::PoseStamped si;
            for (int i = 0; i < tmp_plan.size()-1; ++i) {
                si = tmp_plan[i+1];
                unsigned int tk_mx, tk_my, si_mx, si_my;
                if (!costmap_->worldToMap(tk.pose.position.x, tk.pose.position.y, tk_mx, tk_my)) return false;
                if (!costmap_->worldToMap(si.pose.position.x, si.pose.position.y, si_mx, si_my)) return false;
                // ROS_INFO_STREAM("path planner " << tk_mx << " " << tk_my << " " << si_mx << " " << si_my);
                if (!lineOfSight(tk_mx, tk_my, si_mx, si_my))  {
                    tk = tmp_plan[i];
                    plan_.push_back(tk);
                    // ROS_INFO_STREAM("[path_planner]" << __LINE__ << "push_back");
                }
            }
            plan_.push_back(tmp_plan.back());
            ROS_INFO_STREAM("[path_planner]" << __LINE__ << " Plan size: " << plan_.size());
            // ROS_INFO_STREAM("[path planner]" << cost_total);
            return true;
        }
        double parentG = std::numeric_limits<double>::max();
        if (from_x_[(int)cur.x][(int)cur.y] != -1 && from_y_[(int)cur.x][(int)cur.y] != -1 && from_x_[(int)cur.x][(int)cur.y] != 0 && from_y_[(int)cur.x][(int)cur.y] != 0) {
            parentG = gScore[from_x_[(int)cur.x][(int)cur.y]][from_y_[(int)cur.x][(int)cur.y]];
        }
        for (int i = 0; i < 8; ++i) {
            int nx = cur.x + dir_[i][0];
            int ny = cur.y + dir_[i][1];
            // ROS_INFO_STREAM("[path_planner]" << __LINE__ << " nx ny: " << nx << " " << ny);
            // ROS_INFO_STREAM("[path_planner]" << __LINE__ << "nx ny cell_x cell_y: " << nx << " " << ny << " " << x_cell_sz_ << " " << y_cell_sz_);
            if (nx < 0 || nx >= x_cell_sz_ || ny < 0 || ny >= y_cell_sz_ || from_x_[nx][ny] != -1 || from_y_[nx][ny] != -1) continue;
            int px = from_x_[(int)cur.x][(int)cur.y];
            int py = from_y_[(int)cur.x][(int)cur.y];

            Node next_node(nx, ny);
            Node parent_node(px, py);
            Node parent_world;
            costmap_->mapToWorld(px, py, parent_world.x, parent_world.y);
            double next_wx, next_wy;
            costmap_->mapToWorld(nx, ny, next_wx, next_wy);
            Node next_node_world(next_wx, next_wy);
            double cost_val = costmap_->getCost(nx, ny);
            double dist = next_node.distTo(goal);
            // ROS_INFO_STREAM("[path_planner]" << "now " << cur.x << ' ' << cur.y << ' ' << nx << ' ' << ny << ' ' << px << ' ' << py);
            // if (cost_val > 50) cost_val = 0x3f3f3f3f;
            // if (cost_val != 0)
                // ROS_INFO_STREAM("[path_planner]" << __LINE__ << " cost_val: " << cost_val);
            // if (cost_val != costmap_2d::NO_INFORMATION && cost_val >= 250) cost_val = std::numeric_limits<double>::max();
            // else if (cost_val > 0) {
            //     cost_val *= 0.8;
            // }
            // else cost_val = 0;
            next_node.costmap_cost = cost_val;

            if (px != -1 && py != -1 && lineOfSight(px, py, nx, ny)) {
                double next_g_score = gScore[nx][ny];
                if (next_g_score == -1) next_g_score = cur.walked_dist + cur_world.distTo(next_node_world);
                else next_g_score = std::min(next_g_score, cur.walked_dist + cur_world.distTo(next_node_world));
                // ROS_INFO_STREAM("[path_planner]" << __LINE__ << "next gscore " << next_g_score);
                gScore[nx][ny] = next_g_score;
                if (gScore[px][py] + parent_world.distTo(next_node_world) < gScore[nx][ny]) {
                    // ROS_INFO_STREAM("[path_planner]" << __LINE__ << " from parent");
                    gScore[nx][ny] = gScore[px][py] + parent_node.distTo(next_node);
                    from_x_[nx][ny] = px;
                    from_y_[nx][ny] = py;
                    next_node.walked_dist = gScore[nx][ny];
                    next_node.heuristic = next_node.walked_dist + dist;
                    pq.push(next_node);
                }
                else {
                    next_node.walked_dist = gScore[nx][ny]; 
                    // if (gScore[nx][ny] == -1) gScore[nx][ny] = next_node.walked_dist;
                    // else gScore[nx][ny] = std::min(gScore[nx][ny], (int)next_node.walked_dist); 
                    next_node.heuristic = gScore[nx][ny] + dist;
                    from_x_[nx][ny] = cur.x;
                    from_y_[nx][ny] = cur.y; 
                    pq.push(next_node);

                }
            }
            else {
                // ROS_INFO_STREAM("[path_planner]" << __LINE__ << " not line of sight");
                next_node.walked_dist = cur.walked_dist + cur_world.distTo(next_node_world);
                if (gScore[nx][ny] == -1) gScore[nx][ny] = next_node.walked_dist;
                else gScore[nx][ny] = std::min(gScore[nx][ny], (int)next_node.walked_dist); 
                next_node.heuristic = next_node.walked_dist + dist;
                from_x_[nx][ny] = cur.x;
                from_y_[nx][ny] = cur.y; 
                pq.push(next_node);

            }
        }
    }
    // delete from;

    // for (int i = 0; i < 300; ++i) {
    //     for (int j = 0; j < 200; ++j) {
    //         ROS_INFO_STREAM("[path_planner]" << "gscore " << gScore[i][j]);
    //     }
    // }
    return false;
}

bool PathPlanner::lineOfSight(int x0, int y0, int x1, int y1) {
    // return false;
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);

    int sX = -1;
    int sY = -1;
    if (x0 < x1) sX = 1;
    if (y0 < y1) sY = 1;

    int e = dx + dy;
    int up_limit = 50;
    while (1) {
        // ROS_INFO_STREAM("[path_planner]" << __LINE__ << " x0 y0: " << x0 << " " << y0 << " " << x1 << " " << y1);
        if (x0 < 0 || x0 >= x_cell_sz_ || y0 < 0 || y0 >= y_cell_sz_ || costmap_->getCost(x0, y0) > 0) return false;
        if (x0 == x1 && y0 == y1) return true;

        int e2 = 2 * e;
        if (e2 >= dy) {
            if (x0 == x1) {
                return true;
            }
            e += dy;
            x0 += sX;
        }
        if (e2 <= dx) {
            if (y0 == y1) { 
                return true;
            }
            e += dx;
            y0 += sY;
        }
    }
    return false;
}

};
