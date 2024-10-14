#include <ros/ros.h>
#include <ros/time.h>
#include <std_srvs/Empty.h>



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "fake_rival_reset");
    ros::NodeHandle nh;
    ros::ServiceClient reset_client = nh.serviceClient<std_srvs::Empty>("/rival/reset");
    ros::Rate loop_rate(50);

    std_srvs::Empty reset;
    double last = 0;

    while(ros::ok())
    {
        double now = ros::Time::now().toSec();
        double spain = 10;
        

        if((now-last) >= spain){
            if(reset_client.call(reset)){
                ROS_INFO("Reset operation succeeded.");
                // std::cout<<now-last<<"\n";
            } else {
                ROS_WARN("Failed to call reset service.");
            }
            last = now;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}