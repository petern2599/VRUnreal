#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include <string>


class RingWaypoints
{
    private:
        float x_pos, y_pos, z_pos, offset_x, offset_y, offset_z;
        mavros_msgs::State current_state;
        geometry_msgs::PoseStamped pose;
        std::string trigger_ring, trigger_enable, trigger_disable;
        ros::Subscriber uav0RingWaypoint_sub, state_sub, drone_offset_sub,subTriggerRing;
        ros::Publisher local_pos_pub;
        //Declaring node handle for ROS
        ros::NodeHandle n;
    
    public:
        RingWaypoints()
        {
            //Initialize values
            init_vals();
            uav0RingWaypoint_sub = n.subscribe<geometry_msgs::Vector3>("uav0/ring_target", 10, &RingWaypoints::ringTargetCallback, this);
            state_sub = n.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, &RingWaypoints::state_cb, this);
            drone_offset_sub = n.subscribe<geometry_msgs::Vector3>("uav0/drone_offset", 10, &RingWaypoints::droneOffsetCallback, this);
            subTriggerRing = n.subscribe<std_msgs::String>("uav0/ring_enable", 10, &RingWaypoints::triggerRingCallback, this);
            local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("uav0/mavros/setpoint_position/local", 10);
            //Defining rate
            ros::Rate rate(20.0);

            while(ros::ok)
            {
                
                if(trigger_ring == trigger_enable)
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
                        
                        
                    }
                }
                else if(trigger_ring == trigger_disable)
                {
                    
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
        offset_x = 0.0;
        offset_y = 0.0;
        offset_z = 0.0;

        trigger_enable = "Ring Enabled";
        trigger_disable = "Ring Disabled";
        
    };

    
    

    //Player Position Callback Function
    void ringTargetCallback(const geometry_msgs::Vector3::ConstPtr& msg)
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
    void triggerRingCallback(const std_msgs::String::ConstPtr& msg)
    {
        trigger_ring = msg->data;
    };

};

int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "Ring_Waypoints");
    //Define object class
    RingWaypoints ringWaypoints;
    return 0;
}