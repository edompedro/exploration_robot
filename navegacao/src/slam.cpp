#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include<unistd.h>

class ObstacleAvoidanceNode {
    
public:
    ObstacleAvoidanceNode() : nh_("~"), obstacleDetected_(false){
        
        scanSub_ = nh_.subscribe("/scan", 1, &ObstacleAvoidanceNode::scanCallback, this);
        cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    void spin() {
        ros::Rate rate(10);  

        while (ros::ok()) {
            ros::spinOnce();

            if (obstacleDetected_) {
                ROS_INFO("Obstáculo detectado. Girando...");
                rotate();
            } else {
                ROS_INFO("Nenhum obstáculo detectado. Movendo-se em linha reta...");
                moveForward();
            }
            rate.sleep();
        }
    }

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

        int counter = 0;
        for (float range : msg->ranges) {
            if (range < 0.4) {
                obstacleDetected_ = true;
                return;
            }
            if ((counter%2) != 0){obstacleDirection = 0;} else{obstacleDirection = 1;}
            counter++;
        }
        obstacleDetected_ = false;
    }

    void rotate() {
        geometry_msgs::Twist twist;

        if(obstacleDirection = 0){
            twist.angular.z = 1.0;  
            cmdVelPub_.publish(twist);
        }else{
            twist.angular.z = -1.0;
            cmdVelPub_.publish(twist);
        }
    }

    void moveForward() {
        geometry_msgs::Twist twist;        
        twist.linear.x = 0.2; 
        cmdVelPub_.publish(twist);
    }

    ros::NodeHandle nh_;
    ros::Subscriber scanSub_;
    ros::Publisher cmdVelPub_;
    bool obstacleDetected_;
    int obstacleDirection;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance_node");
    
    ObstacleAvoidanceNode node;
    node.spin();

    return 0;
}
