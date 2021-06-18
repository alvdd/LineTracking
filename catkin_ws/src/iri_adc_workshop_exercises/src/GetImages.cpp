#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
class My_node
{ 
public:
  My_node()
  {
      this->n = ros::NodeHandle("~");
      //Topics subscribed
      this->sub=n.subscribe("/camera/rgb/image_raw",1000,&My_node::data_Callback,this);    
      cv::namedWindow("Image window");
      
  }
  void data_Callback(const sensor_msgs::ImageConstPtr& msg)//diferencia con ::Image::ConstPtr ???
  {
    cv_bridge::CvImagePtr cv_ptr;
    ROS_INFO("brrrrrrrrrrr");

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
     cv::imshow("Image window", cv_ptr->image);
     cv::waitKey(3);
     

      

  }
    
private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  sensor_msgs::ImageConstPtr imageIn_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "GetImages");
  My_node hola;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ROS_INFO("Estoy funcionando");
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
