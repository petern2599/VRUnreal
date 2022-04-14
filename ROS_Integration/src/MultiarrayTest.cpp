#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"

class MultiArray
{
    private:
        std_msgs::Float32MultiArray msg;
        ros::Publisher array_pub;
        //Declaring node handle for ROS
        ros::NodeHandle n;
    
    public:
        MultiArray()
        {
            
            //Defining rate
            ros::Rate rate(20.0);
            
            msg.data.push_back(1.0);
            
            array_pub.publish(msg);
            ros::spinOnce();
            rate.sleep();
            
        }

};  

int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "MultiArray");
    //Define object class
    MultiArray test;
    return 0;
}