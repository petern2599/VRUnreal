#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include <cmath>


class Flight_History
{
    private:
        float offset_x, offset_y, offset_z, drone_xPos, drone_yPos, drone_zPos;
        std::vector<float> vec_x, vec_y, vec_z;
        std_msgs::Float32MultiArray msg;

        ros::Subscriber drone_offset_sub, dronePos_sub;
        ros::Publisher drone_flight_history_pub;
        
    public:
        Flight_History(ros::NodeHandle *n, std::string uav){
            drone_offset_sub = n->subscribe<geometry_msgs::Vector3>(uav + "/drone_offset", 10, &Flight_History::droneOffsetCallback, this);
            dronePos_sub = n->subscribe<geometry_msgs::PoseStamped>(uav + "/mavros/local_position/pose", 10, &Flight_History::dronePosCallback, this);
            drone_flight_history_pub = n->advertise<std_msgs::Float32MultiArray>(uav +"/flight_history", 10);
            //Defining rate
            ros::Rate rate(20.0);
            //Initialize values
            init_vals();
            


        }

        void init_vals()
        {
            drone_xPos = 0.0;
            drone_yPos = 0.0;
            drone_zPos = 0.0;
            offset_x = 0.0;
            offset_y = 0.0;
            offset_z = 0.0;
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
        
        void update_array_message_layout(std_msgs::Float32MultiArray &msg){
            std_msgs::MultiArrayDimension layoutX;
            layoutX.size = vec_x.size();
            layoutX.stride = 1;
            layoutX.label = "X";
            msg.layout.dim.push_back(layoutX);

            std_msgs::MultiArrayDimension layoutY;
            layoutY.size = vec_y.size();
            layoutY.stride = 1;
            layoutY.label = "Y";
            msg.layout.dim.push_back(layoutY);

            std_msgs::MultiArrayDimension layoutZ;
            layoutZ.size = vec_z.size();
            layoutZ.stride = 1;
            layoutZ.label = "Z";
            msg.layout.dim.push_back(layoutZ);
        }
        
        void clear_array_message(std_msgs::Float32MultiArray &msg){
            msg.layout.dim.clear();
            msg.data.clear();
        }

        void update_flight_history(std::vector<float> &vec_x, std::vector<float> &vec_y, std::vector<float> &vec_z){
            vec_x.push_back(drone_yPos + offset_x);
            vec_y.push_back(drone_xPos + offset_y);
            vec_z.push_back(drone_zPos);
        }

        void update_array_message_data(std_msgs::Float32MultiArray &msg){
            for(int x{0}; x < vec_x.size(); x++){
                msg.data.push_back(vec_x.at(x));
            }
            for(int y{0}; y < vec_y.size(); y++){
                msg.data.push_back(vec_y.at(y));
            }
            for(int z{0}; z < vec_z.size(); z++){
                msg.data.push_back(vec_z.at(z));
            }
            
        }

        void publish_flight_history(){
            drone_flight_history_pub.publish(msg);
        }

        void main_process(){
            update_flight_history(vec_x, vec_y, vec_z);
            update_array_message_layout(msg);
            update_array_message_data(msg);
            publish_flight_history();
            clear_array_message(msg);
        }

};


int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "Fly_History");
    ros::NodeHandle n;
    //Define object class
    Flight_History flight_history0(&n, "uav0");
    Flight_History flight_history1(&n, "uav1");
    Flight_History flight_history2(&n, "uav2");

    ros::Rate rate(20.0);
    while(ros::ok){
        flight_history0.main_process();
        flight_history1.main_process();
        flight_history2.main_process();
        ros::Duration(10.0).sleep();
        ros::spinOnce();
        rate.sleep();

    }
    
    return 0;
}