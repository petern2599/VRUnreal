/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "std_msgs/String.h"
#include <string>

class TOL
{
    private:
        mavros_msgs::State current_state;
        std::string takeoff_Trigger, enable_TakeOffTrigger, disable_TakeOffTrigger, land_Trigger, enable_LandTrigger, disable_LandTrigger, drone_select;
        std_msgs::String enable_Armed, disable_Armed;
        //Defining node handle
        ros::NodeHandle n;
        ros::Subscriber state_sub, takeoff_sub, land_sub, localPos_sub, droneSelection_sub;
        ros::Publisher local_pos_pub, armed_pub;
        ros::ServiceClient arming_client, landing_client;
        geometry_msgs::PoseStamped pose;
        //Defining arm command
        mavros_msgs::CommandBool arm_cmd;
        //Defining land command
        mavros_msgs::CommandTOL land_cmd;
        

    public:
        TOL()
        {
            init_vals();
            droneSelection_sub = n.subscribe<std_msgs::String>("/drone_selected", 10, &TOL::droneSelectionCallback, this);

            //the setpoint publishing rate MUST be faster than 2Hz
            ros::Rate rate(20.0);

            // wait for FCU connection
            while(ros::ok() && !current_state.connected)
            {
                ros::spinOnce();
                rate.sleep();
            }

            
            //Defining pose to fly towards
            // pose.pose.position.x = 0;
            // pose.pose.position.y = 0;
            pose.pose.position.z = 2;

            //send a few setpoints before starting
            for(int i = 100; ros::ok() && i > 0; --i)
            {
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }   

            arm_cmd.request.value = true;

            land_cmd.request.yaw = 0;
            land_cmd.request.latitude = 0;
            land_cmd.request.longitude = 0;
            land_cmd.request.altitude = 0;
            land_cmd.request.min_pitch = 0;

            //Reference time of last request
            ros::Time last_request = ros::Time::now();    
            while(ros::ok())
            {   
                if (takeoff_Trigger == enable_TakeOffTrigger)
                {
                    if (land_Trigger == enable_LandTrigger)
                    {
                        if(landing_client.call(land_cmd) && land_cmd.response.success)
                        {
                            armed_pub.publish(disable_Armed);
                            ros::spinOnce();
                            rate.sleep();
                        }
                    }
                    else
                    {
                        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                        {
                            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                            {
                                armed_pub.publish(enable_Armed);
                                ros::spinOnce();
                                rate.sleep();
                            }

                            last_request = ros::Time::now();
                        }
                        
                        //Publish pose to fly towards
                        local_pos_pub.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                    }
                    
                }

                ros::spinOnce();
                rate.sleep();
            }
            
                
        }
    

    void init_vals()
    {
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        enable_TakeOffTrigger = "TakeOff Enabled";
        disable_TakeOffTrigger = "TakeOff Disabled";
        enable_LandTrigger = "Land Enabled";
        disable_LandTrigger = "Land Disabled";
        enable_Armed.data = "Armed";
        disable_Armed.data = "Not Armed";
        drone_select = "uav0";

    };

    void droneSelectionCallback(const std_msgs::String::ConstPtr& msg)
    {
        drone_select = msg->data;
        //std::cout << drone_select << std::endl;
        //std::cout << drone_select + "/offboard_enable" << std::endl;
        //Declaring subscribers, publishers, and service clients
        state_sub = n.subscribe<mavros_msgs::State>(drone_select + "/mavros/state", 10, &TOL::state_cb, this);
        takeoff_sub = n.subscribe<std_msgs::String>(drone_select + "/takeoff_enable", 10, &TOL::takeoff_cb, this);
        localPos_sub = n.subscribe<geometry_msgs::PoseStamped>(drone_select + "/mavros/local_position/pose", 10, &TOL::localPos_cb, this);
        land_sub = n.subscribe<std_msgs::String>(drone_select + "/land_enable", 10, &TOL::landing_cb, this);
        local_pos_pub = n.advertise<geometry_msgs::PoseStamped>(drone_select + "/mavros/setpoint_position/local", 10);
        armed_pub = n.advertise<std_msgs::String>(drone_select + "/armed_status", 10);
        arming_client = n.serviceClient<mavros_msgs::CommandBool>(drone_select + "/mavros/cmd/arming");
        landing_client = n.serviceClient<mavros_msgs::CommandTOL>(drone_select + "/mavros/cmd/land");
    };
    //Callback function to get state
    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state = *msg;
    };

    void takeoff_cb(const std_msgs::String::ConstPtr& msg)
    {
        takeoff_Trigger = msg->data;
        //std::cout << offboard_Trigger << "\n";
    };

    void landing_cb(const std_msgs::String::ConstPtr& msg)
    {
        land_Trigger = msg->data;
        //std::cout << land_Trigger << "\n";
    };

    void localPos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose.pose.position.x = msg->pose.position.x;
        pose.pose.position.y = msg->pose.position.y;
    };
};

int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "takeoff_and_land_node");
    //Define object class
    TOL takeoffandland;
    return 0;
}

