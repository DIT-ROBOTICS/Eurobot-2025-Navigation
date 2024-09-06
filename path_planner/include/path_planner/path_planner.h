/** include the libraries you need in your planner here */
/** for path path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <eigen3/Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/GetPlan.h>
#include <queue>
#include <utility>
#include <algorithm>
#include <limits>
#include <tf/tf.h>
#include <costmap_2d/cost_values.h>

using std::string;

#ifndef PATH_PLANNER_CPP
#define PATH_PLANNER_CPP

struct Node {
    double x;
    double y;
    double heuristic;
    double walked_dist;
    double costmap_cost;
    Node(double _x = 0, double _y = 0): x(_x), y(_y), heuristic(0), walked_dist(0), costmap_cost(0) {}
    bool operator<(const Node& rhs) const {
        if (fabs(costmap_cost-rhs.costmap_cost) > 0) return costmap_cost > rhs.costmap_cost;
        else return heuristic > rhs.heuristic;
    }
    double distTo(Node& rhs) {
        return sqrt(pow((x - rhs.x), 2) + pow((y - rhs.y), 2));
    }
};


namespace path_planner {

class PathPlanner : public nav_core::BaseGlobalPlanner {
public:

    PathPlanner();
    PathPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan
                );
    // double PathPlanner::footprintCost(double x_i, double y_i, double theta_i);
    bool AStarSearch(Node start, Node goal);
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
    void clearRobotCell(int mx, int my);
    void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
    bool check_pose_valid(int, int);

// protected:
    ros::Publisher plan_pub_;
    ros::ServiceServer make_plan_srv_;
    std::string frame_id_;
private:
    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    base_local_planner::WorldModel* world_model_;

    unsigned int x_cell_sz_, y_cell_sz_;
    int from_x_[305][205];
    int from_y_[305][205];
    // std::map<std::pair<int, int>, int> from_x_;
    // std::map<std::pair<int, int>, int> from_y_;
    std::vector<geometry_msgs::PoseStamped> plan_;
    int dir_[45][2] = {
        {1, 1}, {1, -1}, {-1, -1}, {-1, 1}, {1, 0}, {0, -1}, {-1, 0}, {0, 1}, // 45
        {2, 1}, {1, 2}, {-1, 2}, {-2, 1}, {-2, -1}, {-1, -2}, {1, -2}, {2, -1}, // 26.5
        {4, 1}, {1, 4}, {-1, 4}, {-4, 1}, {-4, -1}, {-1, -4}, {1, -4}, {4, -1}, // 14
        {3, 1}, {1, 3}, {-1, 3}, {-3, 1}, {-1, -3}, {-3, -1}, {1, -3}, {3, -1}, // 18
        {3, 2}, {2, 3}, {-2, 3}, {-3, 2}, {-3, -2}, {-2, -3}, {2, -3}, {3, -2} // 34
    };
    int last_dir_idx_[305][205];
    boost::mutex mutex_;
    bool lineOfSight(int x0, int y0, int x1, int y1);
    int gScore[305][205];
    int robot_radius_;
};
};
#endif