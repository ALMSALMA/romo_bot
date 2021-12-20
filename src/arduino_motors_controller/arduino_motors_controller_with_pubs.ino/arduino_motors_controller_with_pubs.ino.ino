#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <AFMotor.h>

void stop();
float mapPwm(float x, float out_min, float out_max);
void onTwist(const geometry_msgs::Twist &msg);

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);

std_msgs::Float64 left;
std_msgs::Float64 right;

std_msgs::Float64 left_pwm;
std_msgs::Float64 right_pwm;

std_msgs::Float64 map_res;

std_msgs::String text;

ros::Publisher text_pub("text", &text);
ros::Publisher left_pub("left", &left);
ros::Publisher right_pub("right", &right);
ros::Publisher left_pwm_pub("left_pwm", &left_pwm);
ros::Publisher right_pwm_pub("right_pwm", &right_pwm);

ros::Publisher map_res_pub("map_res", &map_res);

AF_DCMotor motor3(3);
AF_DCMotor motor4(4);


void setup()

{
 
  stop();
  nh.initNode();
  nh.advertise(text_pub);
  nh.subscribe(sub);
  nh.advertise(left_pub);
  nh.advertise(right_pub);
  nh.advertise(left_pwm_pub);
  nh.advertise(right_pwm_pub);
  nh.advertise(map_res_pub);
}

void loop()
{
  nh.spinOnce();
}

void stop()
{
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void onTwist(const geometry_msgs::Twist &msg)
{

  // Cap values at [-1 .. 1]
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
//  left.data = (2 * x - z * 0.13) / (2 * 0.03);
//  right.data = (2 * x + z * 0.13) / (2 * 0.03);
  left.data = (x - z) / 2;
  right.data = (x + z) / 2;

  right_pub.publish(&right);
  left_pub.publish(&left);
  

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
//  left_pwm.data = mapPwm(fabs(left.data), 0, 255);
//  right_pwm.data = mapPwm(fabs(right.data), 0, 255);

  if(left.data == 0 && right.data == 0){
    left_pwm.data = mapPwm(0, 0, 255);
    right_pwm.data = mapPwm(0, 0, 255);
    
    motor3.setSpeed(left_pwm.data);
    motor4.setSpeed(right_pwm.data);
    
    motor3.run(left_pwm.data);
    motor4.run(right_pwm.data);

    right_pwm_pub.publish(&right_pwm);
    left_pwm_pub.publish(&left_pwm);
    
    text.data = "1-1 entered";
    text_pub.publish(&text);
    
  }else{
    left_pwm.data = mapPwm(fabs(left.data), 0, 255);
    right_pwm.data = mapPwm(fabs(right.data), 0, 255);
    motor3.setSpeed(left_pwm.data);
    motor4.setSpeed(right_pwm.data);   
    
    right_pwm_pub.publish(&right_pwm);
    left_pwm_pub.publish(&left_pwm);
  
    text.data = "1-2 entered";
    text_pub.publish(&text);
  }

  if (left.data>0){
    motor3.run(FORWARD);
       
    text.data = "2-1 entered";
    text_pub.publish(&text);
    
  }else if (left.data<0){
    motor3.run(BACKWARD);
       
    text.data = "2-2 entered";
    text_pub.publish(&text);}
    
  if (right.data>0){
    motor4.run(FORWARD);
       
    text.data = "3-1 entered";
    text_pub.publish(&text);
    
  }else if (right.data<0){
    motor4.run(BACKWARD);
       
    text.data = "3-2 entered";
    text_pub.publish(&text);}
  
}

float mapPwm(float x, float out_min, float out_max)
{ 
     text.data = "map function entered";
     text_pub.publish(&text);
     
     float result = x * (out_max - out_min) + out_min + 100;
     if (x == 0){
        result = 0;
        text.data = "map-1 entered";
        text_pub.publish(&text);
     }else if (result > 255){
        result = 255;
        text.data = "map-2 entered";
        text_pub.publish(&text);
     }else if (result < 150){
        result = 150;
        text.data = "map-3 entered";
        text_pub.publish(&text);
     }
     map_res.data = result;
     map_res_pub.publish(&map_res);
    
  return result;
}
