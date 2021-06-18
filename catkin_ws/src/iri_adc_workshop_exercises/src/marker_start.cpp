#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include <tf/transform_datatypes.h>

class MarkerStart
{
  public:
    MarkerStart()
    {
      this->n = ros::NodeHandle("~");

      this->odom_sub  = n.subscribe("odom", 1, &MarkerStart::odom_callback, this);
      
      this->markers_sub = n.subscribe("markers", 1, &MarkerStart::markers_callback, this);
      
      this->twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      
      this->n.param<double>("speed", this->speed, 0.5);
      this->n.param<double>("steering", this->steering, 0.5);
      this->n.param<int>("start_marker_id", this->start_marker_id, 0);
    }
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      this->input_odom_msg = *msg;
    }
    
    void markers_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
    {
      this->markers=msg->markers;
      
      for(unsigned int i=0;i<this->markers.size();i++)
      {
        if(this->markers[i].id==this->start_marker_id)
        {
          ROS_INFO("marker_start: found marker id=%d, at x,y,z=%f,%f,%f", this->markers[i].id, this->markers[i].pose.pose.position.x,this->markers[i].pose.pose.position.y, this->markers[i].pose.pose.position.z );
          break;
        }
      }

    }
    
    void loop()
    {
      
      
      
    }

  private:
    ros::NodeHandle n;
    
    ros::Subscriber odom_sub;
    nav_msgs::Odometry input_odom_msg;
    
    ros::Subscriber markers_sub;
    std::vector<ar_track_alvar_msgs::AlvarMarker> markers;
    
    ros::Publisher  twist_pub;
    geometry_msgs::Twist twist_msg;
    
    double speed;
    double steering;
    ros::Time time;
    int start_marker_id;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_start");
  MarkerStart my_object;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    my_object.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}