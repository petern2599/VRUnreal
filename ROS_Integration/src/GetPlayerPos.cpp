#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>


//Declaring variables
float x_pos;
float y_pos;
float z_pos;

//Init Function to intialize values
void init_vals()
{
    x_pos = 0.0;
    y_pos = 0.0;
    z_pos = 0.0;
}

//Callback Function
void chatterCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    x_pos = msg->x;
    y_pos = msg->y;
    z_pos = msg->z;

    std::cout << "x: " << x_pos << "  y: "<< y_pos << "  z: " << z_pos << std::endl;
}

//Main Script
int main(int argc, char **argv)
{
    //Initialize node
    ros::init(argc, argv, "unreal_pos_listener"); 
    //Defining node handle for ROS node
    ros::NodeHandle n;
    //Executing function to intialize values
    init_vals();
    //Subscribing to topic
    ros::Subscriber sub = n.subscribe<geometry_msgs::Vector3>("player_position", 10, &chatterCallback);
    //Allow Subscriber to continously get data by looping script
    ros::spin();
    return 0;
}
