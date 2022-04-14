#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <geometry_msgs/Vector3.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher centroid_pos_pub;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/airsim_node/Drone1/MyCamera1/Scene", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    centroid_pos_pub = nh_.advertise<geometry_msgs::Vector3>("/line_centroid", 10);;

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat src = cv_ptr->image;
    cv::Mat imageHSV;
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      cv::cvtColor(src, imageHSV, cv::COLOR_BGR2HSV);

      cv::Vec3b bgrPixel(0,200,0);
      cv::Mat3b bgr(bgrPixel);
      cv::Mat3b hsv;
      cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);

      cv::Vec3b hsvPixel(hsv.at<cv::Vec3b>(0,0));

      int threshold=40;

      cv::Scalar minBGR = cv::Scalar(bgrPixel.val[0]-threshold, bgrPixel.val[1]-threshold, bgrPixel.val[2]-threshold);
      cv::Scalar maxBGR = cv::Scalar(bgrPixel.val[0]+threshold, bgrPixel.val[1]+threshold, bgrPixel.val[2]+threshold);

      cv::Mat maskBGR, resultBGR;
      cv::inRange(src,minBGR,maxBGR,maskBGR);
      cv::bitwise_and(src,src,resultBGR,maskBGR);

      cv::Scalar minHSV = cv::Scalar(hsvPixel.val[0] - threshold, hsvPixel.val[1] - threshold, hsvPixel.val[2] - threshold);
      cv::Scalar maxHSV = cv::Scalar(hsvPixel.val[0]+threshold, hsvPixel.val[1]+threshold, hsvPixel.val[2]+threshold);

      cv::Mat maskHSV, resultHSV;
      cv::inRange(imageHSV,minHSV,maxHSV,maskHSV);
      cv::bitwise_and(imageHSV,imageHSV,resultHSV,maskHSV);

      cv::imshow("Result BGR", resultBGR);
      cv::imshow("Result HSV", resultHSV);
      
      cv::Mat thr, gray;
      cv::cvtColor(resultBGR,gray,cv::COLOR_BGR2GRAY);

      cv::threshold(gray,thr,100,255,cv::THRESH_BINARY);
      
      cv::Moments m = cv::moments(thr,true);
      float cx = m.m10/(m.m00+1e-5);
      float cy = m.m01/(m.m00+1e-5);
      cv::Point p(cx,cy);

      std::cout << cv::Mat(p) << std::endl;

      cv::circle(resultBGR,p,5,cv::Scalar(128,0,0),-1);
      cv::imshow("Image with Center",resultBGR);

      float width = src.cols;
      float height = src.rows;
      
      float error_x;
      float error_y;
      float error_z;

      if(cx == 0.0 || cy == 0.0)
      {
        error_x = 0.0;
        error_y = 0.0;
        error_z = 0.0;
      }
      else
      {
        error_x = cx-width/2;
        error_y = cy-height/2;
        error_z = 0.0;
      }

      geometry_msgs::Vector3 vector;

      vector.x = error_x;
      vector.y = error_y;
      vector.z = error_z;

      centroid_pos_pub.publish(vector);
      
    

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}