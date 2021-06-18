#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

class OdomLoop
{
  public:
    OdomLoop()
    {
      this->n = ros::NodeHandle("~");
      this->odom_sub  = n.subscribe("odom", 1, &OdomLoop::odom_callback, this);
      this->twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      
      //EXAMPLE: fill variables with parameters, or default value
      this->n.param<double>("speed", this->speed, 0.5);
      this->n.param<double>("steering", this->steering, 0.5);
    }
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      //EXAMPLE: subscriber topic. This function is called on each new message received
      this->input_odom_msg = *msg;
      ROS_INFO("odom_loop: received first odom x,y=%f,%f", this->input_odom_msg.pose.pose.position.x, this->input_odom_msg.pose.pose.position.y);
    }
    
    void loop()
    {
      //EXAMPLE: Publish topic
      double my_speed= 0.0;
      double my_steering=0.0;
      this->twist_msg.linear.x = my_speed;
      this->twist_msg.angular.z = my_steering;
      this->twist_pub.publish(this->twist_msg);
      ROS_INFO("publish_twist: linear.x, angular.z = =%f,%f", my_speed, my_steering);
      
      //EXAPLE: ROS Time
      static bool first=true;
      if(first)
      {
        first=false;
        this->time=ros::Time::now();
        ROS_INFO("Getting first time");
      }
      ros::Duration d = ros::Time::now()-this->time;
      ROS_INFO("Elapsed time %f seconds", d.toSec());

    }

  private:
    ros::NodeHandle n;
    
    ros::Subscriber odom_sub;
    nav_msgs::Odometry input_odom_msg;
    
    ros::Publisher  twist_pub;
    geometry_msgs::Twist twist_msg;

    double speed;
    double steering;
    ros::Time time;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_loop");
  OdomLoop my_object;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    my_object.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
