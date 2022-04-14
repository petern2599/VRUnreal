#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include <string>


class LP
{
    private:
        float x_pos, y_pos, z_pos, offset_x, offset_y, offset_z;
        bool isLPTriggered;
        mavros_msgs::State current_state;
        geometry_msgs::PoseStamped pose;
        geometry_msgs::Vector3 drone_offset;
        std::string trigger_LP, trigger_enable, trigger_disable, drone_select;
        ros::Subscriber subTriggerLP, subLPPos, state_sub, droneSelection_sub, drone_offset_sub;
        ros::Publisher local_pos_pub;
        //Declaring node handle for ROS
        ros::NodeHandle n;
    
    public:
        LP()
        {
            droneSelection_sub = n.subscribe<std_msgs::String>("/drone_selected", 10, &LP::droneSelectionCallback, this);

            //Defining rate
            ros::Rate rate(20.0);
            //Initialize values
            init_vals();

            while(ros::ok)
            {
                //if the trigger for FOM is "Enabled" and FOM has not been triggered yet, request service
                if (trigger_LP == trigger_enable)
                {
                    //Defining request variables and switch coords between UE4 and MAVROS
                    pose.pose.position.x = y_pos - offset_x;
                    pose.pose.position.y = x_pos - offset_y;
                    pose.pose.position.z = z_pos;

                    //When call is recieved, indicate that its working with response
                    //and set the that the FOM has been triggered already and response is recieved
                    if (current_state.mode == "OFFBOARD")
                    {
                        local_pos_pub.publish(pose);
                        //ROS_INFO("Flying To Desired Target");  
                        isLPTriggered == true;
                        
                    }
                    
                }
                //When the FOM trigger is set to "Disable", indicate that it is disabled
                // and that FOM has not been triggered
                else if (trigger_LP == trigger_disable)
                {
                    //ROS_INFO("Disabling Flying To Target");
                    isLPTriggered = false;
                }

                
                ros::spinOnce();
                rate.sleep();
            }
        }
    
    //Init Function to intialize values
    void init_vals()
    {
        x_pos = 0.0;
        y_pos = 0.0;
        z_pos = 0.0;
        drone_offset.x = 0.0;
        drone_offset.y = 0.0;
        drone_offset.z = 0.0;
        trigger_enable = "LP Enabled";
        trigger_disable = "LP Disabled";
        drone_select = "uav0";
    };

    void droneSelectionCallback(const std_msgs::String::ConstPtr& msg)
    {
        drone_select = msg->data;
        //Subscribe to get string message to enable FOM
        subTriggerLP = n.subscribe<std_msgs::String>(drone_select + "/lp_enable", 10, &LP::triggerLPCallback, this);
        //Subscribe to get player location
        subLPPos = n.subscribe<geometry_msgs::Vector3>("/landingpad_position", 10, &LP::landpadPosCallback, this);
        state_sub = n.subscribe<mavros_msgs::State>(drone_select + "/mavros/state", 10, &LP::state_cb, this);
        local_pos_pub = n.advertise<geometry_msgs::PoseStamped>(drone_select + "/mavros/setpoint_position/local", 10);
        drone_offset_sub = n.subscribe<geometry_msgs::Vector3>(drone_select + "/drone_offset", 10, &LP::droneOffsetCallback, this);
    };

    //FOM trigger callback function
    void triggerLPCallback(const std_msgs::String::ConstPtr& msg)
    {
        trigger_LP = msg->data;
    };

    
    void landpadPosCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        x_pos = msg->x;
        y_pos = msg->y;
        z_pos = msg->z;
    };

    //Callback function to get state
    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state = *msg;
    };

    void droneOffsetCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        offset_x = msg->x;
        offset_y = msg->y;
        offset_z = msg->z;
    };

};

int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "Landing_Pad");
    //Define object class
    LP landingpad;
    return 0;
}