#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include <cmath>


class FTT
{
    private:
        std_msgs::Float32MultiArray target_pos;
        float offset_x, offset_y, offset_z, drone_xPos, drone_yPos, drone_zPos;
        std::vector<float> vec_x, vec_y, vec_z;
        bool isFTTTriggered;
        mavros_msgs::State current_state;
        geometry_msgs::PoseStamped pose;
        geometry_msgs::Vector3 drone_offset;
        std::string trigger_FTT, trigger_enable, trigger_disable, drone_select;
        ros::Subscriber subTriggerFTT, subTargetPos, state_sub, droneSelection_sub, drone_offset_sub, dronePos_sub;
        ros::Publisher local_pos_pub;
        //Declaring node handle for ROS
        ros::NodeHandle n;
    
    public:
        FTT()
        {
            droneSelection_sub = n.subscribe<std_msgs::String>("/drone_selected", 10, &FTT::droneSelectionCallback, this);

            //Defining rate
            ros::Rate rate(20.0);
            //Initialize values
            init_vals();
            
            bool vector_set {false};
            int index {0};
            bool final_wp_reached {false};
            while(ros::ok)
            {
                //if the trigger for FOM is "Enabled" and FOM has not been triggered yet, request service
                if (trigger_FTT == trigger_enable)
                {
                    if(vector_set == false){
                        if(vec_x.size() == 0){
                            for(int i{0}; i < target_pos.layout.dim[0].size;i++){
                                vec_x.push_back(target_pos.data.at(i));
                            }
                            for(int j{target_pos.layout.dim[0].size}; j < (target_pos.layout.dim[0].size + target_pos.layout.dim[1].size);j++){
                                vec_y.push_back(target_pos.data.at(j));
                            }
                            for(int k{target_pos.layout.dim[0].size + target_pos.layout.dim[1].size}; k < (target_pos.layout.dim[0].size + target_pos.layout.dim[1].size + target_pos.layout.dim[2].size); k++){
                                vec_z.push_back(target_pos.data.at(k));
                            }
                        }
                        vector_set = true;
                    }
                    
                    
                    while(index < vec_x.size()){
                        if(current_state.mode == "OFFBOARD"){
                             //Defining request variables and switch coords between UE4 and MAVROS
                            if(final_wp_reached == false && trigger_FTT != trigger_disable){
                                if(abs((drone_xPos + offset_x) - vec_y[index]) <= 5 && abs((drone_yPos + offset_y) - vec_x[index]) <= 5 && abs(drone_zPos - vec_z[index]) <= 5){
                                    index++;
                                    
                                }
                                pose.pose.position.x = vec_y[index] - offset_x;
                                pose.pose.position.y = vec_x[index] - offset_y;
                                pose.pose.position.z = vec_z[index];
                                local_pos_pub.publish(pose);
                                    
                                
                                //ROS_INFO("Flying To Desired Target");  
                                isFTTTriggered == true;
                            }
                            else{
                                break;
                            }
                            
                        }
                        ros::spinOnce();
                        rate.sleep();
                    }
                    final_wp_reached = true;
                }
                //When the FOM trigger is set to "Disable", indicate that it is disabled
                // and that FOM has not been triggered
                else if (trigger_FTT == trigger_disable)
                {
                    //ROS_INFO("Disabling Flying To Target");
                    isFTTTriggered = false;
                    vec_x.clear();
                    vec_y.clear();
                    vec_z.clear();
                    vector_set = false;
                    index = 0;
                    final_wp_reached = false;
                }

                
                ros::spinOnce();
                rate.sleep();
            }
        }
    
    //Init Function to intialize values
    void init_vals()
    {
        drone_xPos = 0.0;
        drone_yPos = 0.0;
        drone_zPos = 0.0;
        drone_offset.x = 0.0;
        drone_offset.y = 0.0;
        drone_offset.z = 0.0;
        trigger_enable = "FTT Enabled";
        trigger_disable = "FTT Disabled";
        drone_select = "uav0";
    };

    void droneSelectionCallback(const std_msgs::String::ConstPtr& msg)
    {
        drone_select = msg->data;
        //Subscribe to get string message to enable FOM
        subTriggerFTT = n.subscribe<std_msgs::String>(drone_select + "/ftt_enable", 10, &FTT::triggerFTTCallback, this);
        //Subscribe to get player location
        subTargetPos = n.subscribe<std_msgs::Float32MultiArray>(drone_select + "/target_position", 10, &FTT::targetPosCallback, this);
        state_sub = n.subscribe<mavros_msgs::State>(drone_select + "/mavros/state", 10, &FTT::state_cb, this);
        local_pos_pub = n.advertise<geometry_msgs::PoseStamped>(drone_select + "/mavros/setpoint_position/local", 10);
        drone_offset_sub = n.subscribe<geometry_msgs::Vector3>(drone_select + "/drone_offset", 10, &FTT::droneOffsetCallback, this);
        dronePos_sub = n.subscribe<geometry_msgs::PoseStamped>(drone_select + "/mavros/local_position/pose", 10, &FTT::dronePosCallback, this);
    };

    //FOM trigger callback function
    void triggerFTTCallback(const std_msgs::String::ConstPtr& msg)
    {
        trigger_FTT = msg->data;
    };

    //Player Position Callback Function
    void targetPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        target_pos = *msg;
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
        drone_xPos = msg->pose.position.x;
        drone_yPos = msg->pose.position.y;
        drone_zPos = msg->pose.position.z;
    };

    float calc_distance(float x1, float y1, float z1, float x2, float y2, float z2){
        return sqrt(pow((x2 - x1),2) + pow((y2 - y1),2) + pow((z2 - z1),2));
    }

};

int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "Fly_To_Target");
    //Define object class
    FTT flyToTarget;
    return 0;
}