#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>

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
            array_pub = n.advertise<std_msgs::Float32MultiArray>("/multiarray_test", 10);
            //Defining rate
            ros::Rate rate(20.0);
            
            std::vector<double> vecX {1.0, 2.0, 3.0};
            std::vector<double> vecY {4.0, 5.0, 6.0};
            std_msgs::MultiArrayDimension layoutX;
            layoutX.size = vecX.size();
            layoutX.stride = 1;
            layoutX.label = "X";
            msg.layout.dim.push_back(layoutX);

            std_msgs::MultiArrayDimension layoutY;
            layoutY.size = vecY.size();
            layoutY.stride = 1;
            layoutY.label = "Y";
            msg.layout.dim.push_back(layoutY);
            
            for(int x{0}; x < vecX.size(); x++){
                msg.data.push_back(vecX.at(x));
            }
            for(int y{0}; y < vecY.size(); y++){
                msg.data.push_back(vecY.at(y));
            }
            while(ros::ok){
                array_pub.publish(msg);
                ros::spinOnce();
                rate.sleep();
            }
            
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