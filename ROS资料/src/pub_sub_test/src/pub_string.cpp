#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc,argv, "talker");
    ros::NodeHandle n;
    //初始化节点
    ros::Publisher chatter_pub=n.advertise<std_msgs::String>("chatter",1000);
    //发布话题"chatter"
    ros::Rate loop_rate(10); //睡眠频率，配合loop_rate.sleep()使用
    int count = 0;
    while (ros::ok)
    {
        std_msgs::String msg;
     //定义了std_msgs::String的对象msg       String/Int8
        std::stringstream ss;
        ss << "hello world"<<count;//写入ss 字符串流
        msg.data = ss.str();//赋值msg

        ROS_INFO("%s",msg.data.c_str());
        chatter_pub.publish(msg); //pusblish()
        ros::spinOnce();
        loop_rate.sleep();
        ++count;

    }
    return 0;    
}
