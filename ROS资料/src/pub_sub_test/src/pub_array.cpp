#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include <sstream>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("chatter", 1000);
    //ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("chatter", 1000);

    ros::Rate loop_rate(10);
    std::vector<double> testArray = {1,2,3,4,5};
    while (ros::ok())
    {
        std_msgs::Float64MultiArray msg;
        msg.data = testArray;
        ROS_INFO("I have published array data");
        chatter_pub.publish(msg);
        ros::spinOnce;
        loop_rate.sleep();
    }
    

}