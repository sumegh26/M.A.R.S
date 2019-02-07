
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#define EN_L 5                   //break   PE_4
#define IN1_L 3                //motor1 dir   PB_0
#define IN2_L 23               //motor1 pwm    PD_0
 
#define EN_R 25                //break         PD_2
#define IN1_R 4              //motor2 dir      PB_1
#define IN2_R 24              //motor2 pwm     PD_1
 
 
double wheel_r=0;
double wheel_l=0;
double wheel_radius=0.05;
double wheel_sep=0.50;
double ang_speed=0;
double lin_speed=0;

int lowSpeed = 200;
int highSpeed = 50;

ros::NodeHandle nh;

void cmdVelCb( const geometry_msgs::Twist& twist){                      //callback function
 ang_speed=twist.angular.z;
 lin_speed=twist.linear.x;
 wheel_r = (lin_speed/wheel_radius)+((ang_speed*wheel_sep)/(2.0*wheel_radius));
 wheel_l = (lin_speed/wheel_radius)-((ang_speed*wheel_sep)/(2.0*wheel_radius));
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCb );
char hello[30] = "subscribed to teleop";
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);
std_msgs::String str_msg;

void setup()
{
 pinMode(28, OUTPUT);           //PE_2
 pinMode(8, OUTPUT);            //PA_5
  Motors_init();
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  digitalWrite(28, HIGH);
   digitalWrite(8, HIGH);
    str_msg.data = hello;
    MotorL(wheel_l*10);
    MotorR(wheel_r*10);
    nh.spinOnce();
    str_msg.data = hello;
    delay(1);
}


void Motors_init(){                 //defining pins and setting all motors to halt state
 pinMode(EN_L, OUTPUT);
 pinMode(EN_R, OUTPUT);
 pinMode(IN1_L, OUTPUT);
 pinMode(IN2_L, OUTPUT);
 pinMode(IN1_R, OUTPUT);
 pinMode(IN2_R, OUTPUT);
 digitalWrite(EN_L, LOW);
 digitalWrite(EN_R, LOW);
 digitalWrite(IN1_L, LOW);
 digitalWrite(IN2_L, LOW);
 digitalWrite(IN1_R, LOW);
 digitalWrite(IN2_R, LOW);
}

void MotorL(int Pulse_Width1){              //motor1 clockwise
 if (Pulse_Width1 > 0){
 
    // analogWrite(EN_L, Pulse_Width1);
 
     analogWrite(IN1_L, Pulse_Width1);      //pwm
 
     digitalWrite(IN2_L, HIGH);              //dir pin
 
 }
 
 if (Pulse_Width1 < 0){                      //motor1 anti
 
     Pulse_Width1=abs(Pulse_Width1);
 
     //analogWrite(EN_L, Pulse_Width1);
 
     analogWrite(IN1_L, Pulse_Width1);
 
     digitalWrite(IN2_L, LOW);
 
 }
 
 if (Pulse_Width1 == 0){                        //motor1 stop
 
     //analogWrite(EN_L, Pulse_Width1);
 
     analogWrite(IN1_L, Pulse_Width1);
 
     digitalWrite(EN_L, HIGH);                   //breaking pin
 
 }
 
}


void MotorR(int Pulse_Width2){
 
 
 if (Pulse_Width2 > 0){
 
     //analogWrite(EN_R, Pulse_Width2);
 
     analogWrite(IN1_R, Pulse_Width2);
 
     digitalWrite(IN2_R, LOW);
 
 }
 
 if (Pulse_Width2 < 0){
 
     Pulse_Width2=abs(Pulse_Width2);
 
     //analogWrite(EN_R, Pulse_Width2);
 
     analogWrite(IN1_R, Pulse_Width2);
 
     digitalWrite(IN2_R, HIGH);
 
 }
 
 if (Pulse_Width2 == 0){
 
     //analogWrite(EN_R, Pulse_Width2);
 
     analogWrite(IN1_R, Pulse_Width2);
 
     digitalWrite(EN_R, HIGH);
 
 }
 
 }
