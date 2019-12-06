#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>


class TeleopJoy
{
public:
  TeleopJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void simCallback(const geometry_msgs::Twist::ConstPtr& twist);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  bool linear_pressed, angular_pressed, autonomy_pressed_on, autonomy_pressed_off;
  std::string topic_name_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopJoy::TeleopJoy():
  linear_(1),
  angular_(0)
{

  nh_.param("/comandar_joy_node/axis_linear", linear_, 1);
  nh_.param("/comandar_joy_node/axis_angular", angular_, 2);
  nh_.param("/comandar_joy_node/scale_angular", a_scale_, 0.05);
  nh_.param("/comandar_joy_node/scale_linear", l_scale_, 0.05);
  nh_.param<std::string>("/comandar_joy_node/topic_name", topic_name_, "joy/cmd_vel");

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_name_, 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);

}


void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  // CONTROLE PELO JOYSTICK
  geometry_msgs::Twist twist;
  
  if (joy->buttons[0] && !linear_pressed)
  {
    l_scale_ += 0.05;
    //sim_l_scale += 0.05;
    linear_pressed = 1;
  }
  if (joy->buttons[1]&& !linear_pressed) 
  {
    l_scale_ -= 0.05;
    //sim_l_scale -= 0.05;
    linear_pressed = 1;
  }
  if ((!joy->buttons[1]) && (!joy->buttons[0])) linear_pressed = 0;

  if (joy->buttons[4] && !angular_pressed) 
  {
    a_scale_ += 0.05;
    //sim_a_scale += 0.05;
    angular_pressed = 1;
  }
  if (joy->buttons[3]&& !angular_pressed) 
  {
    a_scale_ -= 0.05;
    //sim_a_scale -= 0.05;
    angular_pressed = 1;
  }
  if ((!joy->buttons[3]) && (!joy->buttons[4])) angular_pressed = 0;


  // MODO AUTOMATICO - CONTROLE ATRAVÉS DO SEGUIMENTO DE SOLDA
  if (joy->buttons[2] && !autonomy_pressed_on)
  {
      system("rosservice call mux/select air1/cmd_vel");
      autonomy_pressed_on = 1;
      autonomy_pressed_off = 0;
  }

  // MODO MANUAL - CONTROLE ATRAVÉS DO JOYSTICK
  if (!joy->buttons[2] && !autonomy_pressed_off)
  {
      system("rosservice call mux/select joy/cmd_vel");
      autonomy_pressed_off = 1;
      autonomy_pressed_on = 0;
  }

  // PUBLICANDO
  twist.angular.z = -a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(twist);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  TeleopJoy teleop_joy;

  ros::spin();
}
