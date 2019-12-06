#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>


int status_atual;
double sim_vel_linear, sim_vel_angular;
double sim_l_scale = 1.0, sim_a_scale = 1.0;

class TeleopJoy
{
public:
  TeleopJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void simCallback(const geometry_msgs::Twist::ConstPtr& twist);
  void fuzCallback(const std_msgs::Int16::ConstPtr& fuz);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  bool linear_pressed, angular_pressed;
  std::string topic_name_;
  ros::Publisher vel_pub_;
  ros::Publisher status_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber sim_sub_;
  ros::Subscriber fuz_sub_;

};


TeleopJoy::TeleopJoy():
  linear_(1),
  angular_(0)
{

  nh_.param("/comandar_joy_node/axis_linear", linear_, 1);
  nh_.param("/comandar_joy_node/axis_angular", angular_, 0);
  nh_.param("/comandar_joy_node/scale_angular", a_scale_, 0.4);
  nh_.param("/comandar_joy_node/scale_linear", l_scale_, 0.2);
  nh_.param<std::string>("/comandar_joy_node/topic_name", topic_name_, "/air1/cmd_vel");

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_name_, 1);

  status_pub_ = nh_.advertise<std_msgs::Int16>("autonomy_level", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);
  sim_sub_ = nh_.subscribe<geometry_msgs::Twist>("sim/cmd_vel", 10, &TeleopJoy::simCallback, this);
  fuz_sub_ = nh_.subscribe<std_msgs::Int16>("fuzzy/autonomy_level", 10, &TeleopJoy::fuzCallback, this);

}

void TeleopJoy::fuzCallback(const std_msgs::Int16::ConstPtr& fuz)
{

  status_atual = fuz->data;

}

void TeleopJoy::simCallback(const geometry_msgs::Twist::ConstPtr& twist)
{

  sim_vel_linear = twist->linear.x;
  sim_vel_angular = twist->angular.z;

}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  // MODO DE AUTONOMIA
  std_msgs::Int16 status;

  if (joy->buttons[0]) status_atual = 1; // manual
  if (joy->buttons[1]) status_atual = 2; // controle de velocidade linear
  if (joy->buttons[2]) status_atual = 3; // controle de direção
  if (joy->buttons[3]) status_atual = 4; // autônomo

  status.data = status_atual;
  status_pub_.publish(status);

  // CONTROLE PELO JOYSTICK
  geometry_msgs::Twist twist;
  
  if (joy->buttons[5] && !linear_pressed) 
  {
    l_scale_ += 0.05;
    //sim_l_scale += 0.05;
    linear_pressed = 1;
  }
  if (joy->buttons[4]&& !linear_pressed) 
  {
    l_scale_ -= 0.05;
    //sim_l_scale -= 0.05;
    linear_pressed = 1;
  }
  if ((!joy->buttons[4]) && (!joy->buttons[5])) linear_pressed = 0;

  if (joy->buttons[7] && !angular_pressed) 
  {
    a_scale_ += 0.05;
    //sim_a_scale += 0.05;
    angular_pressed = 1;
  }
  if (joy->buttons[6]&& !angular_pressed) 
  {
    a_scale_ -= 0.05;
    //sim_a_scale -= 0.05;
    angular_pressed = 1;
  }
  if ((!joy->buttons[6]) && (!joy->buttons[7])) angular_pressed = 0;


  // SELEÇÃO DE AUTONOMIA
  if (status_atual == 1)
  {
    twist.angular.z = -a_scale_*joy->axes[angular_];
    twist.linear.x = l_scale_*joy->axes[linear_];
  }
  
  if (status_atual == 2)
  {
    twist.angular.z = sim_a_scale*sim_vel_angular;
    twist.linear.x = l_scale_*joy->axes[linear_];
  }
  if (status_atual == 3 || status_atual == 4)
  {
    twist.angular.z = sim_a_scale*sim_vel_angular;
    twist.linear.x = sim_l_scale*sim_vel_linear;
  }

  vel_pub_.publish(twist);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  TeleopJoy teleop_joy;

  ros::spin();
}
