#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/BatteryState.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <string>


class VehicleInfo
{
    private:
        float battery_percentage0, battery_percentage1, battery_percentage2, offset_x, offset_y, x_pos, y_pos, z_pos;
        std_msgs::Float32 battery_percentage0_F32, battery_percentage1_F32, battery_percentage2_F32;
        geometry_msgs::Vector3 drone_offset0, drone_offset1, drone_offset2;
        std::string drone_select;
        ros::Subscriber battery0_sub, battery1_sub, battery2_sub;
        ros::Publisher battery_percentage0_pub, battery_percentage1_pub, battery_percentage2_pub, drone_offset0_pub, drone_offset1_pub, drone_offset2_pub;
        //Declaring node handle for ROS
        // ros::NodeHandle n;
    
    public:
        VehicleInfo(ros::NodeHandle *n, std::string uav, float x_offset, float y_offset, float z_offset)
        {
            

            init_vals();
            
            drone_offset0_pub = n->advertise<geometry_msgs::Vector3>(uav + "/drone_offset", 10);
            // drone_offset1_pub = n.advertise<geometry_msgs::Vector3>("/uav1/drone_offset", 10);
            // drone_offset2_pub = n.advertise<geometry_msgs::Vector3>("/uav2/drone_offset", 10);
            battery_percentage0_pub = n->advertise<std_msgs::Float32>(uav + "/battery_percentage", 10);
            // battery_percentage1_pub = n.advertise<std_msgs::Float32>("/uav1/battery_percentage", 10);
            // battery_percentage2_pub = n.advertise<std_msgs::Float32>("/uav2/battery_percentage", 10);

            battery0_sub = n->subscribe<sensor_msgs::BatteryState>(uav + "/mavros/battery", 10, &VehicleInfo::batteryCallback0, this);
            // battery1_sub = n.subscribe<sensor_msgs::BatteryState>("/uav1/mavros/battery", 10, &VehicleInfo::batteryCallback1, this);
            // battery2_sub = n.subscribe<sensor_msgs::BatteryState>("/uav2/mavros/battery", 10, &VehicleInfo::batteryCallback2, this);

            
            //Defining rate
            ros::Rate rate(20.0);
            //Initialize values
            
            drone_offset0.x = x_offset;
            drone_offset0.y = y_offset;
            drone_offset0.z = z_offset;
        
        
            // drone_offset1.x = 0.0;
            // drone_offset1.y = -3.0;
            // drone_offset1.z = 0.0;

            // drone_offset2.x = 3.0;
            // drone_offset2.y = -3.0;
            // drone_offset2.z = 0.0;
        
            // while(ros::ok)
            // {
            //     battery_percentage0_F32.data = battery_percentage0;
            //     // battery_percentage1_F32.data = battery_percentage1;
            //     // battery_percentage2_F32.data = battery_percentage2;

            //     battery_percentage0_pub.publish(battery_percentage0_F32);
            //     // battery_percentage1_pub.publish(battery_percentage1_F32);
            //     // battery_percentage2_pub.publish(battery_percentage2_F32);

            //     drone_offset0_pub.publish(drone_offset0);
            //     // drone_offset1_pub.publish(drone_offset1);
            //     // drone_offset2_pub.publish(drone_offset2);
            //     ros::spinOnce();
            //     rate.sleep();
            // }
            
        }

    
    //Init Function to intialize values
    void init_vals()
    {
        battery_percentage0 = 0.0;
        // battery_percentage1 = 0.0;
        // battery_percentage2 = 0.0;
    };

    
    //Battery Percentage callback function
    void batteryCallback0(const sensor_msgs::BatteryState::ConstPtr& msg)
    {
        battery_percentage0 = msg->percentage;
    };

    //Battery Percentage callback function
    // void batteryCallback1(const sensor_msgs::BatteryState::ConstPtr& msg)
    // {
    //     battery_percentage1 = msg->percentage;
    // };

    // //Battery Percentage callback function
    // void batteryCallback2(const sensor_msgs::BatteryState::ConstPtr& msg)
    // {
    //     battery_percentage2 = msg->percentage;
    // };

    void publishVehicleInfo()
    {
        ros::Rate rate(20.0);
        battery_percentage0_F32.data = battery_percentage0;
        // battery_percentage1_F32.data = battery_percentage1;
        // battery_percentage2_F32.data = battery_percentage2;

        battery_percentage0_pub.publish(battery_percentage0_F32);
        // battery_percentage1_pub.publish(battery_percentage1_F32);
        // battery_percentage2_pub.publish(battery_percentage2_F32);

        drone_offset0_pub.publish(drone_offset0);
        // drone_offset1_pub.publish(drone_offset1);
        // drone_offset2_pub.publish(drone_offset2);
        ros::spinOnce();
        rate.sleep();
    }
};

int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "Vehicle_Info");

    //Defining node handle
    ros::NodeHandle n;
    //Define object class
    VehicleInfo vehicle_info0(&n, "uav0", -3.0, -3.0, 0.0);
    VehicleInfo vehicle_info1(&n, "uav1", 0.0, -3.0, 0.0);
    VehicleInfo vehicle_info2(&n, "uav2", 3.0, -3.0, 0.0);

    ros::Rate rate(20.0);
    while(ros::ok())
    {   
        vehicle_info0.publishVehicleInfo();
        vehicle_info1.publishVehicleInfo();
        vehicle_info2.publishVehicleInfo();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}