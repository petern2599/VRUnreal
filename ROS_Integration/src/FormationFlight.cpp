#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include <string>


class FF
{
    private:
        float leader_xPos, leader_yPos, leader_zPos, leader_xOffset, leader_yOffset, leader_zOffset,follower_xPos, follower_yPos, follower_zPos, follower_xOffset, follower_yOffset, follower_zOffset;
        int distance_x, distance_y, distance_z;
        bool isFFTriggered;
        mavros_msgs::State leader_state, follower_state;
        geometry_msgs::PoseStamped followerPose;
        std::string trigger_FF, trigger_enable, trigger_disable, leader_select, follower_select;
        ros::Subscriber leaderSelection_sub, followerSelection_sub, leaderPos_sub, leaderOffset_sub, followerPos_sub, followerOffset_sub, leaderState_sub, followerState_sub, distanceFF_sub, triggerFF_sub ;
        ros::Publisher followerLocalPos_pub;
        //Declaring node handle for ROS
        ros::NodeHandle n;
    
    public:
        FF()
        {
            leaderSelection_sub = n.subscribe<std_msgs::String>("/leader_selected", 10, &FF::leaderSelectionCallback, this);
            followerSelection_sub = n.subscribe<std_msgs::String>("/follower_selected", 10, &FF::followerSelectionCallback, this);
            distanceFF_sub = n.subscribe<geometry_msgs::Vector3>("/formation_distance", 10, &FF::distanceFFCallback, this);
            triggerFF_sub = n.subscribe<std_msgs::String>("/formation_enable", 10, &FF::triggerFFCallback, this);
            //Defining rate
            ros::Rate rate(20.0);
            //Initialize values
            init_vals();

            while(ros::ok)
            {
                //if the trigger for FOM is "Enabled" and FOM has not been triggered yet, request service
                if (trigger_FF == trigger_enable)
                {
                    //Defining request variables and switch coords between UE4 and MAVROS
                    followerPose.pose.position.x = (leader_xPos + leader_xOffset + distance_x) - follower_xOffset;
                    followerPose.pose.position.y = (leader_yPos + leader_yOffset + distance_y) - follower_yOffset;
                    followerPose.pose.position.z = (leader_zPos + leader_zOffset + distance_z) - follower_zOffset;


                    //When call is recieved, indicate that its working with response
                    //and set the that the FOM has been triggered already and response is recieved
                    if (follower_state.mode == "OFFBOARD")
                    {
                        followerLocalPos_pub.publish(followerPose);
                        //ROS_INFO("Flying To Desired Target");  
                        isFFTriggered == true;
                        ros::spinOnce();
                        rate.sleep();
                    }
                    
                }
                //When the FOM trigger is set to "Disable", indicate that it is disabled
                // and that FOM has not been triggered
                else if (trigger_FF == trigger_disable)
                {
                    //ROS_INFO("Disabling Flying To Target");
                    isFFTriggered = false;
                    ros::spinOnce();
                    rate.sleep();
                }

                
                ros::spinOnce();
                rate.sleep();
            }
        }
    
    //Init Function to intialize values
    void init_vals()
    {
        leader_xPos = 0.0;
        leader_yPos = 0.0;
        leader_zPos = 0.0;
        leader_xOffset = 0.0;
        leader_yOffset = 0.0;
        leader_zOffset = 0.0;
        follower_xOffset = 0.0;
        follower_yOffset = 0.0;
        follower_zOffset = 0.0;
        
        trigger_enable = "FF Enabled";
        trigger_disable = "FF Disabled";
        leader_select = "uav0";
        follower_select = "uav1";
    };

    void leaderSelectionCallback(const std_msgs::String::ConstPtr& msg)
    {
        leader_select = msg->data;
        
        leaderPos_sub = n.subscribe<geometry_msgs::PoseStamped>(leader_select + "/mavros/local_position/pose", 10, &FF::leaderPosCallback, this);
        leaderOffset_sub = n.subscribe<geometry_msgs::Vector3>(leader_select + "/drone_offset", 10, &FF::leaderOffsetCallback, this);
        leaderState_sub = n.subscribe<mavros_msgs::State>(leader_select + "/mavros/state", 10, &FF::leaderStateCallback, this);
    };

    void followerSelectionCallback(const std_msgs::String::ConstPtr& msg)
    {
        follower_select = msg->data;

        followerPos_sub = n.subscribe<geometry_msgs::PoseStamped>(follower_select + "/mavros/local_position/pose", 10, &FF::followerPosCallback, this);
        followerOffset_sub = n.subscribe<geometry_msgs::Vector3>(follower_select + "/drone_offset", 10, &FF::followerOffsetCallback, this);
        followerState_sub = n.subscribe<mavros_msgs::State>(follower_select + "/mavros/state", 10, &FF::followerStateCallback, this);
        followerLocalPos_pub = n.advertise<geometry_msgs::PoseStamped>(follower_select + "/mavros/setpoint_position/local", 10);
    };

    void distanceFFCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        distance_x = msg->x;
        distance_y = msg->y;
        distance_z = msg->z;
    };

    //FOM trigger callback function
    void triggerFFCallback(const std_msgs::String::ConstPtr& msg)
    {
        trigger_FF = msg->data;
    };

    //Callback function to get state
    void leaderStateCallback(const mavros_msgs::State::ConstPtr& msg)
    {
        leader_state = *msg;
    };

    void followerStateCallback(const mavros_msgs::State::ConstPtr& msg)
    {
        follower_state = *msg;
    };

    void leaderPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        leader_xPos = msg->pose.position.x;
        leader_yPos = msg->pose.position.y;
        leader_zPos = msg->pose.position.z;
    };

    void leaderOffsetCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        leader_xOffset = msg->x;
        leader_yOffset = msg->y;
        leader_zOffset = msg->z;
    };

    void followerPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        follower_xPos = msg->pose.position.x;
        follower_yPos = msg->pose.position.y;
        follower_zPos = msg->pose.position.z;
    };

    void followerOffsetCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        follower_xOffset = msg->x;
        follower_yOffset = msg->y;
        follower_zOffset = msg->z;
    };

};

int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "Formation_Flight");
    //Define object class
    FF formationFlight;
    return 0;
}