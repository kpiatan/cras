#define USE_USBCON // necessary for arduino due
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <ros/time.h>

// input buttons
#define auxLU 22 // auxiliar left up -- K15
#define auxLD 23 // auxiliar left down -- K16
#define auxMU 24 // auxiliar mid up -- K14
#define auxMD 25 // auxiliar mid down -- K13
#define auxRU 26 // auxiliar right up -- K12

// input axes
#define leftL 28 // left stick slightly left -- K6
#define leftR 29 // left stick slightly right -- K5
#define leftLR 30 // left stick full left/right -- K7
#define leftU 31 // left stick slightly up -- K8
#define leftD 32 // left stick slightly down -- K9
#define leftUD 33 // left stick full up/down -- K10
#define rightL 34 // right stick slightly left -- K4
#define rightR 35 // right stick slightly right -- K3
#define rightLR 36 // right stick full left/right -- K11


ros::NodeHandle  nh;

sensor_msgs::Joy joy_msg;
float temp_axes[3]; // necessary when dealing with std vectors in arduino
long int temp_buttons[5];
ros::Publisher pub("joy", &joy_msg);
int seq = 0;

void setup()
{ 
  Serial.begin(9600);
  nh.getHardware()->setBaud(9600);
  
  pinMode(auxLU, INPUT_PULLUP);
  pinMode(auxLD, INPUT_PULLUP);
  pinMode(auxMU, INPUT_PULLUP);
  pinMode(auxMD, INPUT_PULLUP);
  pinMode(auxRU, INPUT_PULLUP);

  pinMode(leftL, INPUT_PULLUP);
  pinMode(leftR, INPUT_PULLUP);
  pinMode(leftLR, INPUT_PULLUP);
  pinMode(leftU, INPUT_PULLUP);
  pinMode(leftD, INPUT_PULLUP);
  pinMode(leftUD, INPUT_PULLUP);
  pinMode(rightL, INPUT_PULLUP);
  pinMode(rightR, INPUT_PULLUP);
  pinMode(rightLR, INPUT_PULLUP);

  joy_msg.axes_length = 3; // necessary when working with arduino
  joy_msg.buttons_length = 5;  
  
  nh.initNode();
  //joy_msg.header.frame_id = "null";
  nh.advertise(pub);
}

void loop()
{
  seq++;
  joy_msg.header.seq = seq;
  joy_msg.header.stamp = nh.now();

  // left stick, horizontal
  if (digitalRead(leftL) == LOW && digitalRead(leftLR) == LOW) temp_axes[0] = -1.0;
  if (digitalRead(leftL) == LOW && digitalRead(leftLR) == HIGH) temp_axes[0] = -0.5;
  if (digitalRead(leftR) == LOW && digitalRead(leftLR) == LOW) temp_axes[0] = 1.0;
  if (digitalRead(leftR) == LOW && digitalRead(leftLR) == HIGH) temp_axes[0] = 0.5;
  if (digitalRead(leftL) == HIGH && digitalRead(leftR) == HIGH) temp_axes[0] = 0.0;

  // left stick, vertical
  if (digitalRead(leftU) == LOW && digitalRead(leftUD) == LOW) temp_axes[1] = 1.0;
  if (digitalRead(leftU) == LOW && digitalRead(leftUD) == HIGH) temp_axes[1] = 0.5;
  if (digitalRead(leftD) == LOW && digitalRead(leftUD) == LOW) temp_axes[1] = -1.0;
  if (digitalRead(leftD) == LOW && digitalRead(leftUD) == HIGH) temp_axes[1] = -0.5;
  if (digitalRead(leftU) == HIGH && digitalRead(leftD) == HIGH) temp_axes[1] = 0.0;

  // right stick, horizontal
  if (digitalRead(rightL) == LOW && digitalRead(rightLR) == LOW) temp_axes[2] = -1.0;
  if (digitalRead(rightL) == LOW && digitalRead(rightLR) == HIGH) temp_axes[2] = -0.5;
  if (digitalRead(rightR) == LOW && digitalRead(rightLR) == LOW) temp_axes[2] = 1.0;
  if (digitalRead(rightR) == LOW && digitalRead(rightLR) == HIGH) temp_axes[2] = 0.5;
  if (digitalRead(rightL) == HIGH && digitalRead(rightR) == HIGH) temp_axes[2] = 0.0;
  
  joy_msg.axes = temp_axes;

  if (digitalRead(auxLU) == LOW) temp_buttons[0] = 1;
  else temp_buttons[0] = 0;
  if (digitalRead(auxLD) == LOW) temp_buttons[1] = 1;
  else temp_buttons[1] = 0;
  if (digitalRead(auxMU) == LOW) temp_buttons[2] = 1;
  else temp_buttons[2] = 0;
  if (digitalRead(auxMD) == LOW) temp_buttons[3] = 1;
  else temp_buttons[3] = 0;
  if (digitalRead(auxRU) == LOW) temp_buttons[4] = 1;
  else temp_buttons[4] = 0;
  joy_msg.buttons = temp_buttons;
  
  pub.publish(&joy_msg);
  nh.spinOnce();
  delay(250);
}
