  /*
  *
  * Project Name:   M.A.R.S
  * Author List:   Sumegh Pramod Thale 
  *     
  * Filename:     TELEOP_CONTROLLER
  * Functions:    cmdVelCb(),setup(),loop().
  * Global Variables: none. 
  *     
  *
  */
#include<ros.h>
#include<std_msgs/String.h>
#include<geometry_msgs/Twist.h>

//Break pins
#define B_LF 22              
#define B_LR 23
#define B_RF 24
#define B_RR 25

//Direction pins
#define D_LF 26 
#define D_LR 27
#define D_RF 28
#define D_RR 29

//PWM pins
#define P_LF 2
#define P_LR 3
#define P_RF 4
#define P_RR 5

void cmdVelCb( const geometry_msgs::Twist& twist)                          // Variable Name: ang_speed  lin_speed
{ //callback function
  float ang_speed;
  float lin_speed;
  
  ang_speed=twist.angular.z;
  lin_speed=twist.linear.x;

//IF-ELSE CONDITIONS FOR TELEOP

if(ang_speed == 0 && lin_speed == 0)     
{    
  //No Input,thus keep the bot stationary by making break pin High.                                  
  digitalWrite(B_LF,HIGH);
  digitalWrite(B_LR,HIGH);
  digitalWrite(B_RF,HIGH);
  digitalWrite(B_RR,HIGH);

  //Making PWM pins Zero.
  analogWrite(P_LF,0);
  analogWrite(P_LR,0);
  analogWrite(P_RF,0);
  analogWrite(P_RR,0);

  }


else if(ang_speed == 0 && lin_speed != 0)             //conditions if linear velocity is given
{
  if(lin_speed > 0)
{ 
  //Making Direction pins High.
  digitalWrite(D_LF,HIGH);
  digitalWrite(D_LR,HIGH);
  digitalWrite(D_RF,HIGH);
  digitalWrite(D_RR,HIGH);

  //Making Breaking pins Low.
  digitalWrite(B_LF,LOW);
  digitalWrite(B_LR,LOW);
  digitalWrite(B_RF,LOW);
  digitalWrite(B_RR,LOW);

  //Giving value to PWM pins.
  analogWrite(P_LF,100);
  analogWrite(P_LR,100);
  analogWrite(P_RF,100);
  analogWrite(P_RR,100);
  
   }
else if(lin_speed < 0) 
{
  //Making Direction pins Low.
  digitalWrite(D_LF,LOW);
  digitalWrite(D_LR,LOW);
  digitalWrite(D_RF,LOW);
  digitalWrite(D_RR,LOW);

  //Making Breaking pins Low.
  digitalWrite(B_LF,LOW);
  digitalWrite(B_LR,LOW);
  digitalWrite(B_RF,LOW);
  digitalWrite(B_RR,LOW);

  //Giving value to PWM pins.
  analogWrite(P_LF,100);
  analogWrite(P_LR,100);
  analogWrite(P_RF,100);
  analogWrite(P_RR,100);
  
  }

else
{
  //IF above conditions are not satisfied stoping the bot by breaking pins 
  digitalWrite(B_LF,HIGH);
  digitalWrite(B_LR,HIGH);
  digitalWrite(B_RF,HIGH);
  digitalWrite(B_RR,HIGH);
  
  }    
 }


else if(ang_speed != 0 && lin_speed == 0)                         //conditions if angular velocity is given
{
  if(ang_speed > 0)                                               // condition when velocity is positive
{                                                                 
  //Making Breaking pins Low.
  digitalWrite(B_LF,LOW);
  digitalWrite(B_LR,LOW);
  digitalWrite(B_RF,LOW);
  digitalWrite(B_RR,LOW);

  //Left wheels high,right wheel loww
  digitalWrite(D_LF,HIGH);
  digitalWrite(D_LR,HIGH);
  digitalWrite(D_RF,LOW);
  digitalWrite(D_RR,LOW);

  //Giving value to PWM pins.
  analogWrite(P_LF,50);
  analogWrite(P_LR,50);
  analogWrite(P_RF,50);
  analogWrite(P_RR,50);
    }
 else if(ang_speed < 0)
{
  //Making Breaking pins Low.
  digitalWrite(B_LF,LOW);
  digitalWrite(B_LR,LOW);
  digitalWrite(B_RF,LOW);
  digitalWrite(B_RR,LOW);

  //Left wheels Low,right wheels High
  digitalWrite(D_LF,HIGH);
  digitalWrite(D_LR,HIGH);
  digitalWrite(D_RF,LOW);
  digitalWrite(D_RR,LOW);

  //Giving value to PWM pins.
  analogWrite(P_LF,50);
  analogWrite(P_LR,50);
  analogWrite(P_RF,50);
  analogWrite(P_RR,50);

 }
 else
{
  //IF above conditions are not satisfied stoping the bot by breaking pins 
  digitalWrite(B_LF,HIGH);
  digitalWrite(B_LR,HIGH);
  digitalWrite(B_RF,HIGH);
  digitalWrite(B_RR,HIGH);
 
  }
 }
}


ros::NodeHandle nh;                                               // Instantiating the node handle
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&cmdVelCb);  //Subscribing to cmd_vel and calling callback function


void setup() {                                                   // Variable Name: Description of the variable and the range of expected values of the variable. 
   
 nh.getHardware()-> setBaud(9600);                               //setting the baud rate of Arduino.
 
 //Assigning pinmodes
 pinMode(P_LF, OUTPUT);                                          //PWM PIN
 pinMode(P_LR, OUTPUT);                                          //PWM PIN
 pinMode(P_RF, OUTPUT);                                          //PWM PIN
 pinMode(P_RR, OUTPUT);                                          //PWM PIN
 pinMode(D_LF, OUTPUT);                                          //Direction PIN
 pinMode(D_LR, OUTPUT);                                          //Direction PIN
 pinMode(D_RF, OUTPUT);                                          //Direction PIN
 pinMode(D_RR, OUTPUT);                                          //Direction PIN
 pinMode(B_LF, OUTPUT);                                          //Breaking PIN
 pinMode(B_LR, OUTPUT);                                          //Breaking PIN
 pinMode(B_RF, OUTPUT);                                          //Breaking PIN
 pinMode(B_RR, OUTPUT);                                          //Breaking PIN

nh.initNode();
nh.subscribe(sub);                                               //Subscribing to sub.

}

void loop() {
  
nh.spinOnce();                                                   //ros::spinOnce() will handle passing messages to the subscriber callback. 

}
