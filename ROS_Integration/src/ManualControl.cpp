#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "std_msgs/String.h"
#include <string>


class ManualController
{
    private:
        float left_x, left_y, left_z, right_x, right_y, right_z;
        std::string enable_ManualTrigger {"Manual Enabled"}, disable_ManualTrigger {"Manual Disabled"};
        std::string manual_Trigger; 
        mavros_msgs::State current_state;
        geometry_msgs::TwistStamped twist;
        ros::Subscriber state_sub, left_controller_sub, right_controller_sub, manual_sub;
        ros::Publisher local_vel_pub;
        //Declaring node handle for ROS
        ros::NodeHandle n;
    
    public:
        ManualController()
        {
            //Subscribe to get player location
            left_controller_sub = n.subscribe<geometry_msgs::Vector3>("/left_controller",10, &ManualController::left_controller_cb, this);
            right_controller_sub = n.subscribe<geometry_msgs::Vector3>("/right_controller",10, &ManualController::right_controller_cb, this);
            manual_sub = n.subscribe<std_msgs::String>("/uav0/manual_enable",10, &ManualController::manual_cb, this);
            
            state_sub = n.subscribe<mavros_msgs::State>("/uav0/mavros/state", 10, &ManualController::state_cb, this);
            local_vel_pub = n.advertise<geometry_msgs::TwistStamped>("/uav0/mavros/setpoint_velocity/cmd_vel", 10);
            //Defining rate
            ros::Rate rate(20.0);

            while(ros::ok)
            {
                if(manual_Trigger == enable_ManualTrigger){
                    //Defining request variables and switch coords between UE4 and MAVROS
                    twist.twist.linear.x = left_y;
                    twist.twist.linear.y = left_x;
                    twist.twist.linear.z = right_x;

                    twist.twist.angular.z = right_y;
                    
                    //std::cout << "Hello" << "\n";
                    

                    //When call is recieved, indicate that its working with response
                    //and set the that the FOM has been triggered already and response is recieved
                    if (current_state.mode == "OFFBOARD")
                    {
                        local_vel_pub.publish(twist);
                    }
                        
                    ros::spinOnce();
                    rate.sleep();
                }
                ros::spinOnce();
                rate.sleep();
            }
        }

    //Callback function to get state
    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state = *msg;
    };

    void left_controller_cb(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        left_x = msg->x;
        left_y = msg->y;
        left_z = msg->z;
    }
    void right_controller_cb(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        right_x = msg->x;
        right_y = msg->y;
        right_z = msg->z;
    }
    void manual_cb(const std_msgs::String::ConstPtr& msg)
    {
        manual_Trigger = msg->data;
        //std::cout << manual_Trigger << "\n";
    };
    

    
};

int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "Manual_Controller");
    //Define object class
    ManualController manualController;
    return 0;
}