#include "ros/ros.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>

class GetAirsimImages
{
    private:
        sensor_msgs::Image msg;
        std::string str;
    public:
        GetAirsimImages()
        {
            using namespace msr::airlib;

            typedef ImageCaptureBase::ImageRequest ImageRequest;
            typedef ImageCaptureBase::ImageResponse ImageResponse;
            typedef ImageCaptureBase::ImageType ImageType;

            // for car use
            // CarRpcLibClient client;
            MultirotorRpcLibClient client;

            // get right, left and depth images. First two as png, second as float16.
            std::vector<ImageRequest> request = { 
                //uncompressed RGB array bytes
                ImageRequest("1", ImageType::Scene, false, false),       
                
            };

            while(ros::ok)
            {
                const std::vector<ImageResponse>& responses = client.simGetImages(request);

                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "frameId";
                msg.encoding = "rgba8";
                msg.height = 240;
                msg.width = 320;
                msg.data = ;
                msg.is_bigendian = 0;
                msg.step = msg.width*1;
            }
            
        }
    

};

int main(int argc, char **argv)
{
    //Intializing node
    ros::init(argc, argv, "Airsim_Images");
    //Define object class
    GetAirsimImages airsim_images;
    return 0;
}