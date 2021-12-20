#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <AFMotor.h>

void stop();
float mapPwm(float x, float out_min, float out_max);
void onTwist(const geometry_msgs::Twist &msg);

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);

AF_DCMotor motor3(3);
AF_DCMotor motor4(4);


void setup()

{
  stop();
  nh.initNode();
  nh.subscribe(sub);
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

  float left = (x - z) / 2;
  float right = (x + z) / 2;
  

  if(left == 0 && right == 0){
  
    uint16_t left_pwm = mapPwm(0, 0, 255);
    uint16_t right_pwm = mapPwm(0, 0, 255);
    
    motor3.setSpeed(left_pwm);
    motor4.setSpeed(right_pwm);
    
    motor3.run(left_pwm);
    motor4.run(right_pwm);
    
  }else{
    
    uint16_t left_pwm = mapPwm(fabs(left), 0, 255);
    uint16_t right_pwm = mapPwm(fabs(right), 0, 255);
    
    motor3.setSpeed(left_pwm);
    motor4.setSpeed(right_pwm);   
  }

  if (left>0)
    motor3.run(FORWARD);
  else if (left<0)
    motor3.run(BACKWARD);
 
  if (right>0)
    motor4.run(FORWARD);
  else if (right<0)
    motor4.run(BACKWARD);
}      

float mapPwm(float x, float out_min, float out_max){
       
     float result = x * (out_max - out_min) + out_min + 100;
     
     if (x == 0)
        result = 0;
        
     else if (result > 255)
        result = 255;
        
     else if (result < 150)
        result = 150;
    
  return result;
}
