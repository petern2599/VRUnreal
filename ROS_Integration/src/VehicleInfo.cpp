#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/BatteryState.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <string>


class VehicleInfo
{
    private:
        float battery_percentage, offset_x, offset_y, x_pos, y_pos, z_pos;
        std_msgs::Float32 battery_percentage_F32;
        geometry_msgs::Vector3 drone_offset;
        std::string drone_select;
        ros::Subscriber battery_sub, droneSelection_sub;
        ros::Publisher battery_percentage_pub, drone_offset_pub;
        //Declaring node handle for ROS
        ros::NodeHandle n;
    
    public:
        VehicleInfo()
        {
            

            init_vals();
            droneSelection_sub = n.subscribe<std_msgs::String>("/drone_selected", 10, &VehicleInfo::droneSelectionCallback, this);
            drone_offset_pub = n.advertise<geometry_msgs::Vector3>(drone_select + "/drone_offset", 10);
            battery_percentage_pub = n.advertise<std_msgs::Float32>(drone_select + "/battery_percentage", 10);
            //Defining rate
            ros::Rate rate(20.0);
            //Initialize values
            
            while(ros::ok)
            {
                battery_percentage_F32.data = battery_percentage;
                battery_percentage_pub.publish(battery_percentage_F32);
                drone_offset_pub.publish(drone_offset);
                ros::spinOnce();
                rate.sleep();
            }
            
        }

    
    //Init Function to intialize values
    void init_vals()
    {
        battery_percentage = 0.0;
        drone_offset.x = -3.0;
        drone_offset.y = 3.0;
        drone_offset.z = 0.0;
        drone_select="/uav0";
    };

    void droneSelectionCallback(const std_msgs::String::ConstPtr& msg)
    {
        drone_select = msg->data;
        //std::cout << drone_select << std::endl;
        if(drone_select == "/uav0")
        { 
            drone_offset.x = -3.0;
            drone_offset.y = 3.0;
            drone_offset.z = 0.0;
        }
        else if(drone_select == "/uav1")  
        {
            drone_offset.x = 0.0;
            drone_offset.y = 0.0;
            drone_offset.z = 0.0;
        }
        battery_sub = n.subscribe<sensor_msgs::BatteryState>(drone_select + "/mavros/battery", 10, &VehicleInfo::batteryCallback, this);
        drone_offset_pub = n.advertise<geometry_msgs::Vector3>(drone_select + "/drone_offset", 10);
        battery_percentage_pub = n.advertise<std_msgs::Float32>(drone_select + "/battery_percentage", 10);

    };
    //Battery Percentage callback function
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
    {
        battery_percentage = msg->percentage;
    };

};

int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "Vehicle_Info");
    //Define object class
    VehicleInfo vehicle_info;
    return 0;
}