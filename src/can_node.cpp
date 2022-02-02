
#include "ros/ros.h"
#include "std_msgs/Bool.h" 

//500 here is the receiving CAN ID


void chatter1(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data ? "true" : "false");
    if (msg->data)
    {
    system("cansend can0 500#01");
    }
    else
    {
    system("cansend can0 500#00");
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/object", 1000, chatter1);
    ros::spin();

  return 0;
}

