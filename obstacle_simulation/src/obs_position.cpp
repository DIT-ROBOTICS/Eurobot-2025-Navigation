#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "obstacle_simulation/obs_position.h"
#include "obstacle_detector/CircleObstacle.h"
#include "obstacle_detector/Obstacles.h"
#include "std_msgs/Bool.h"
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


bool pot_status;
double generateGaussianNoise(double mu, double sigma)
{
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0*3.14159265358979323846;

    static double z0, z1;
    static bool generate;
    generate = !generate;

    if (!generate)
       return z1 * sigma + mu;

    double u1, u2;
    do
    {
      u1 = rand() * (1.0 / RAND_MAX);
      u2 = rand() * (1.0 / RAND_MAX);
    }
    while ( u1 <= epsilon );

    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}

void obstaclePub(ros::Publisher pub, std::string frame, std::vector<std::vector<double>> obstacle_pos){
    int obstacle_num = obstacle_pos.size();
    for(int i = 0; i < obstacle_num; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame; 
        pose.pose.position.x = obstacle_pos[i][0];
        pose.pose.position.y = obstacle_pos[i][1];
        pub.publish(pose);
    }
}

void obstaclePubArray(ros::Publisher pub, std::string frame, obstacle_detector::Obstacles obstacle_pos){
    int obstacle_num = obstacle_pos.circles.size();
    obstacle_pos.header.frame_id = frame;
    obstacle_pos.header.stamp = ros::Time::now();
    // poses.header.frame_id = frame;
    // poses.header.stamp = ros::Time::now();

    // for(int i = 0; i < obstacle_num; i++)
    // {     
    //     geometry_msgs::Pose p;
    //     p.position.x = obstacle_pos[i][0];
    //     p.position.y = obstacle_pos[i][1];
    //     poses.poses.push_back(p);
    // }
    pub.publish(obstacle_pos);
}

/**
 *  @brief: pubish all types of obstacles 
 *  @param: std::vector<std::vector<double>> obstacles : all the obstacles information from YAML file
 */ 
void obstaclePub(ros::Publisher pub, std::vector<Obstacle> obstacles)
{
    geometry_msgs::PoseArray poses;
    for(int i=0; i<obstacles.size(); i++)
    {
        poses.header.frame_id = obstacles[0].from;
        poses.header.stamp = ros::Time::now();
        switch(obstacles[i].motion_type){
            case Motion_type::STATIC:
                for(int j = 0; j<obstacles[0].position.size(); j++)
                {
                    geometry_msgs::Pose p;
                    p.position.x = generateGaussianNoise(obstacles[i].position[j][0], obstacles[i].stdev);
                    p.position.y = generateGaussianNoise(obstacles[i].position[j][1], obstacles[i].stdev);
                    poses.poses.push_back(p);
                }
                break;
            case Motion_type::RECIPROCATION:
                break;
        }
    }
    pub.publish(poses);
}


/** @brief Parse a vector of vector of floats from a string.
 * @param input
 * @param error_return
 * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...] */
std::vector<std::vector<double>> parseVVF(const std::string & input, std::string & error_return)
{
    std::vector<std::vector<double>> result;

    std::stringstream input_ss(input);
    int depth = 0;
    std::vector<double> current_vector;
    while (!!input_ss && !input_ss.eof()) {
        switch (input_ss.peek()) {
            case EOF:
                break;
            case '[':
                depth++;
                if (depth > 2) {
                    error_return = "Array depth greater than 2";
                    return result;
                }
                input_ss.get();
                current_vector.clear();
                break;
            case ']':
                depth--;
                if (depth < 0) {
                    error_return = "More close ] than open [";
                    return result;
                }
                input_ss.get();
                if (depth == 1) {
                    result.push_back(current_vector);
                }
                break;
            case ',':
            case ' ':
            case '\t':
                input_ss.get();
                break;
                default:  // All other characters should be part of the numbers.
                if (depth != 2) {
                    std::stringstream err_ss;
                    err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
                    error_return = err_ss.str();
                    return result;
                }
                float value;
                input_ss >> value;
                if (!!input_ss) {
                    current_vector.push_back(value);
                }
                break;
        }
    }
    if (depth != 0) {
        error_return = "Unterminated vector string.";
    } else {
        error_return = "";
    }
    return result;
}

void PotControlCB(const std_msgs::Bool::ConstPtr& msg) {
    pot_status = msg->data; 
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Config conf;
    conf.bools.clear();
    bool_param.name = "enabled";
    if (pot_status) {
        bool_param.value = true;
    }
    else {
        bool_param.value = false;
    }
    conf.bools.push_back(bool_param);
    srv_req.config = conf;
    // ros::service::call("move_base/global_costmap/inflation_layer", srv_req, srv_resp);
    ros::service::call("move_base/global_costmap/inflation_layer/set_parameters", srv_req, srv_resp);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "obstacle_position");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher obs_pub_PoseStamped = nh.advertise<geometry_msgs::PoseStamped>("obstacle_position",1000);
    ros::Publisher obs_pub_PoseArray = nh.advertise<obstacle_detector::Obstacles>("pot",1000);
    ros::Subscriber pot_ctl_sub = nh.subscribe("pot_ctl", 1000, PotControlCB);
    // ros::Publisher obs_pub_all_PoseArray = nh.advertise<geometry_msgs::PoseArray>("obstacle_all_position_array1",1000);
    pot_status = true;

    // read the YAML file
    int update_frequency;
    std::string frame;
    std::string RobotName; 
    nh.getParam("RobotName", RobotName);
    private_nh.param("update_frequency", update_frequency, 10);
    private_nh.param("frame", frame, std::string(RobotName + "/map"));
    ros::Rate loop_rate(update_frequency);

    // std::vector<Obstacle> obstacles;

    double obstacle_num = 0;

    // ROS_INFO_STREAM("[obst postion]" << obstacles.size());
    while(ros::ok())
    {
        obstacle_detector::Obstacles obstacles;
        std::vector<std::vector<double>> obs_pos;
        if (private_nh.hasParam("obstacles"))
        {
            if (pot_status) {
                XmlRpc::XmlRpcValue my_list;
                private_nh.getParam("obstacles", my_list);
                // ROS_INFO_STREAM("[obstacle simulation]" << my_list.size());
                for (int32_t i = 0; i < my_list.size(); ++i)
                {
                    std::string pos, error_return;
                    pos = static_cast<std::string>(my_list[i]["position"]);
                    obs_pos = parseVVF(pos, error_return);
                    // obs.stdev = static_cast<double>(my_list[i]["stdev"]);
                    if(error_return!= "")
                        ROS_ERROR("[obs_position] obstacle_simulation: error_return %s", error_return.c_str());
                    else {
                        obstacle_detector::CircleObstacle cir_obs;
                        cir_obs.center.x = obs_pos[0][0];
                        cir_obs.center.y = obs_pos[0][1];
                        cir_obs.radius = 0.125;
                        cir_obs.true_radius = 0.125 + 0.3;
                        obstacles.circles.push_back(cir_obs);
                    }

                }
            }
            else {
                obstacles.circles.clear();
            }
        }
        else {
            ROS_INFO("No sources!!!");
        }


        // obstaclePub(obs_pub_PoseStamped, frame, obstacle_pos);
        obstaclePubArray(obs_pub_PoseArray, frame, obstacles);
        // obstaclePub(obs_pub_PoseArray, obstacles);
        ros::spinOnce();
        loop_rate.sleep();
    }

}