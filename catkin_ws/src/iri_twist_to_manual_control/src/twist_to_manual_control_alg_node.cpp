#include "twist_to_manual_control_alg_node.h"

TwistToManualControlAlgNode::TwistToManualControlAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<TwistToManualControlAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->steering_publisher_ = this->public_node_handle_.advertise<std_msgs::UInt8>("steering", 1);
  this->speed_publisher_ = this->public_node_handle_.advertise<std_msgs::Int16>("speed", 1);
  
  // [init subscribers]
  this->cmd_vel_subscriber_ = this->public_node_handle_.subscribe("cmd_vel", 1, &TwistToManualControlAlgNode::cmd_vel_callback, this);
  pthread_mutex_init(&this->cmd_vel_mutex_,NULL);

  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

TwistToManualControlAlgNode::~TwistToManualControlAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->cmd_vel_mutex_);
}

void TwistToManualControlAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  // Initialize the topic message structure

  // Initialize the topic message structure
  //this->steering_UInt8_msg_.data = my_var;

  // Initialize the topic message structure
  //this->speed_Int16_msg_.data = my_var;

  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]

  // Uncomment the following line to publish the topic message
  //this->steering_publisher_.publish(this->steering_UInt8_msg_);

  // Uncomment the following line to publish the topic message
  //this->speed_publisher_.publish(this->speed_Int16_msg_);

}

/*  [subscriber callbacks] */
void TwistToManualControlAlgNode::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double min_in, max_in, zero_in;
  int min_out, max_out, zero_out;

  //STEERING
  int steering;
  double angular;

  if(this->config_.invert_steering)
    angular = - msg->angular.z;
  else
    angular = msg->angular.z;
  
  min_in=this->config_.angular_sat_min;
  max_in=this->config_.angular_sat_max;
  zero_in=0.0;

  angular = this->clip(angular, min_in, max_in);

  zero_out = this->config_.steering_zero;

  if(angular>0)
  {
    min_in  = zero_in;
    min_out = zero_out;
    max_out = this->config_.steering_max;

  }
  else //if(angular<=0)
  {
    max_in  = zero_in;
    min_out = this->config_.steering_min;
    max_out = zero_out;
  }
  
  steering = (((angular  - min_in) * (max_out - min_out)) / (max_in - min_in)) + min_out;
  
  this->steering_UInt8_msg_.data = steering;
  
  //SPEED
  int speed;
  double linear;

  if(this->config_.invert_speed)
    linear = - msg->linear.x;
  else
    linear = msg->linear.x;
  
  min_in=this->config_.linear_sat_min;
  max_in=this->config_.linear_sat_max;
  zero_in=0.0;

  linear = this->clip(linear, min_in, max_in);

  zero_out = this->config_.speed_zero;

  if(linear>0)
  {
    min_in  = zero_in;
    min_out = zero_out;
    max_out = this->config_.speed_max;

  }
  else //if(linear<=0)
  {
    max_in  = zero_in;
    min_out = this->config_.speed_min;
    max_out = zero_out;
  }
  
  speed = (((linear  - min_in) * (max_out - min_out)) / (max_in - min_in)) + min_out;
  
  this->speed_Int16_msg_.data = speed;

  //PUBLISH
  ROS_INFO("Steering = %d \r\n",steering_UInt8_msg_.data);
  ROS_INFO("Speed = %d \r\n",speed_Int16_msg_.data);
  this->steering_publisher_.publish(this->steering_UInt8_msg_);
  this->speed_publisher_.publish(this->speed_Int16_msg_);
}

double TwistToManualControlAlgNode::clip(double n, double lower, double upper)
{
  return std::max(lower, std::min(n, upper));
}

void TwistToManualControlAlgNode::cmd_vel_mutex_enter(void)
{
  pthread_mutex_lock(&this->cmd_vel_mutex_);
}

void TwistToManualControlAlgNode::cmd_vel_mutex_exit(void)
{
  pthread_mutex_unlock(&this->cmd_vel_mutex_);
}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void TwistToManualControlAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void TwistToManualControlAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<TwistToManualControlAlgNode>(argc, argv, "twist_to_manual_control_alg_node");
}
