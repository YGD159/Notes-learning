#include "ros/ros.h"
#include "std_msgs/String.h"


void chatterCallback(const std_msgs::String::ConstPtr&  msg)//msg --接收信息的指针
//回调函数  
//Int 8 类型(const std_msgs::Int8::ConstPtr&  msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    //初始化节点
    ros::Subscriber sub =n.subscribe("chatter", 1000, chatterCallback);
    //订阅chatter话题
    ros::spin();
    return 0;
}

