#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include<cmath>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker" );
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("chatter", 10);
    ros:: Rate loop_rate(10);

    double positionX, positionY, postionZ;
    double orienntationX, orienntationY, orienntationZ, orienntationW;

    double fixedOrientation = 0.1;
    orienntationX = fixedOrientation;
    orienntationY = fixedOrientation;
    orienntationZ = fixedOrientation;
    orienntationW = sqrt(1.0 - 3.0*fixedOrientation*fixedOrientation);

    double count = 0.0;
    while ( (ros::ok))
    {
        positionX = count;
        positionY = count;
        postionZ  =  count;

        geometry_msgs::PoseStamped msg;
        ros::Time currentTime = ros::Time::now();
        msg.header.stamp = currentTime;

        msg.pose.position.x = positionX;
        msg.pose.position.y = positionY;
        msg.pose.position.z = positionY;

        msg.pose.orientation.x = orienntationX;
        msg.pose.orientation.y = orienntationY;
        msg.pose.orientation.z = orienntationZ;
        msg.pose.orientation.w = orienntationW;

        ROS_INFO("we publish the robat's position and orientation");
        ROS_INFO("the position(x,y,z) is  %f , %f , %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        ROS_INFO("the orientation(x,y,z,w) is %f, %f, %f", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
        ROS_INFO("the time we get the pose is %f", msg.header.stamp.sec + 1e-9*msg.header.stamp.nsec);
        std::cout<<"\n \n"<< std::endl;

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

        ++count;
    }
    return 0 ;
}