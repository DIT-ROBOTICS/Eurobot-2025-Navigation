#include "navigation_main/navigation_main.h"
int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_main");

    std::string node_name_ = ros::this_node::getName();

    do {
        try {
            ROS_INFO_STREAM("[" << node_name_ << "] : Initializing ...");

            ros::NodeHandle nh_global("");
            ros::NodeHandle nh_local("~");

            Navigation_Main navigation_main_(&nh_global, &nh_local, node_name_);
            ros::Rate rate(navigation_main_.GetUpdateFrequency());
            while (ros::ok()) {
                ros::spinOnce();

                navigation_main_.Loop();

                rate.sleep();
            }
        } catch (const char* s) {
            ROS_FATAL_STREAM("[" << node_name_ << "] : " << s);
            ROS_FATAL_STREAM("[" << node_name_ << "] : Resuming ...");
        } catch (...) {
            ROS_FATAL_STREAM("[" << node_name_ << "] : Unexpected error occurred.");
            ROS_FATAL_STREAM("[" << node_name_ << "] : Resuming ...");
        }
    } while (ros::ok());

    return EXIT_SUCCESS;
}