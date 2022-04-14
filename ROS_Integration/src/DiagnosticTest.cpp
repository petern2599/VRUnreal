#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include <vector>
#include <string>

class DiagnosticTest
{
    private:
        diagnostic_msgs::DiagnosticArray diag_msg;
        ros::Subscriber diagnostic_sub;
        std::string message;
        
        //Declaring node handle for ROS
        ros::NodeHandle n;
    
    public:
        DiagnosticTest()
        {
            diagnostic_sub = n.subscribe<diagnostic_msgs::DiagnosticArray>("diagnostics", 10, &DiagnosticTest::diagnosticCallback, this);;
            //Defining rate
            ros::Rate rate(20.0);
            
            while(ros::ok){
                std::cout << diag_msg.status[0].message << std::endl;
                ros::spinOnce();
                rate.sleep();
            }

            ros::spinOnce();
            rate.sleep(); 
                
            
            
        }

        void diagnosticCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg){
            diag_msg = *msg;
            message = diag_msg.status[0].values[0].value;
        }

};  

int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "DiagnosticTest");
    //Define object class
    DiagnosticTest test;
    return 0;
}