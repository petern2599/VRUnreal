#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "std_msgs/String.h"
#include <string>


class LF
{
    private:
        float x_pos, y_pos, z_pos, offset_x, offset_y, offset_z, drone_x, drone_y, drone_z;
        bool isLFTriggered;
        mavros_msgs::State current_state;
        geometry_msgs::TwistStamped twist;
        geometry_msgs::Vector3 drone_offset;
        ros::Subscriber subTargetPos, state_sub, drone_offset_sub, drone_pos_sub;
        ros::Publisher local_vel_pub;
        //Declaring node handle for ROS
        ros::NodeHandle n;
    
    public:
        LF()
        {
            //Subscribe to get player location
            subTargetPos = n.subscribe<geometry_msgs::Vector3>("/line_centroid", 10, &LF::targetPosCallback, this);
            state_sub = n.subscribe<mavros_msgs::State>("/uav0/mavros/state", 10, &LF::state_cb, this);
            local_vel_pub = n.advertise<geometry_msgs::TwistStamped>("/uav0/mavros/setpoint_velocity/cmd_vel", 10);
            drone_offset_sub = n.subscribe<geometry_msgs::Vector3>("/uav0/drone_offset", 10, &LF::droneOffsetCallback, this);
            drone_pos_sub = n.subscribe<geometry_msgs::PoseStamped>("/uav0/mavros/local_position/pose", 10, &LF::dronePosCallback, this);
            //Defining rate
            ros::Rate rate(20.0);
            //Initialize values
            init_vals();

            while(ros::ok)
            {
                
                //Defining request variables and switch coords between UE4 and MAVROS
                if(x_pos != 0.0 || y_pos != 0.0)
                {
                    twist.twist.linear.x = -(y_pos/30);
                    twist.twist.linear.y = -(x_pos/30);
                    twist.twist.linear.z = 0;
                }
                else
                {
                    twist.twist.linear.x = 0;
                    twist.twist.linear.y = 0;
                    twist.twist.linear.z = 0;
                }
                

                //When call is recieved, indicate that its working with response
                //and set the that the FOM has been triggered already and response is recieved
                if (current_state.mode == "OFFBOARD")
                {
                    local_vel_pub.publish(twist);
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
        drone_x = 0.0;
        drone_y = 0.0;
        drone_z = 0.0;
        offset_x = 0.0;
        offset_y = 0.0;
        offset_z = 0.0;
        
    };

    //Player Position Callback Function
    void targetPosCallback(const geometry_msgs::Vector3::ConstPtr& msg)
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

    void dronePosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        drone_x = msg->pose.position.x;
        drone_y = msg->pose.position.y;
        drone_z = msg->pose.position.z;
    };
};

int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "Line_Follower");
    //Define object class
    LF linefollower;
    return 0;
}