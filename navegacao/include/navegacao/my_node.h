#ifndef MY_NODE_H
#define MY_NODE_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class MyNode
{
public:
    MyNode();
    ~MyNode();
    void move();

private:
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::Publisher pub;
    ros::Subscriber sub;
    bool obstacle;
};

#endif // MY_NODE_H
