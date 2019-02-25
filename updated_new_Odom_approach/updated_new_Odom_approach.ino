  /*
  *
  * Project Name:   M.A.R.S
  * Author List:   Sumegh Pramod Thale 
  *     
  * Filename:     TELEOP_CONTROLLER
  * Functions:    cmdVelCb(),setup(),loop(),motor_1_distance(),motor_2_distance(),motor_3_distance(),motor_4_distance()
  * Global Variables: motor_1;motor_2;motor_3;motor_4;motor,
  * Macros:       B_LF,B_LR,B_RF,B_RR,D_LF,D_LR,D_RF,D_RR,P_LF,P_LR,P_RF,P_RR.   
  *
  */
#include<ros.h>
#include<std_msgs/String.h>
#include<dec_description/odom_data.h>
#include<geometry_msgs/Twist.h>
#include <ros/time.h>
#include<tf/tf.h>
#include<tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

//Direction pins
#define D_LF 26 
#define D_LR 27
#define D_RF 28
#define D_RR 29

//PWM pins
#define P_LF 4
#define P_LR 5
#define P_RF 6
#define P_RR 7


//pseudo breaking by making ground pin high
#define brk 40

//Global variables for encoder ticks
float motor_1;
float motor_2;
float motor_3;
float motor_4;
float motor;
float wheel_base = 0.50;
float theta;
float distance;
float left_encoder;
float right_encoder;

float  X_position;
float  Y_position;
float counter1_hold,counter2_hold,counter3_hold,counter4_hold;


void cmdVelCb( const geometry_msgs::Twist& twist)                          // Variable Name: ang_speed  lin_speed
{ //callback function
  float ang_speed;
  float lin_speed;
  
  ang_speed=twist.angular.z;
  lin_speed=twist.linear.x;

//IF-ELSE CONDITIONS FOR TELEOP

if(ang_speed == 0 && lin_speed == 0)     
{    
  //No Input,thus keep the bot stationary by making ground pin High.                                  
  digitalWrite(brk,HIGH);
  
  //Making Direction pins High.
  digitalWrite(D_LF,HIGH);
  digitalWrite(D_LR,HIGH);
  digitalWrite(D_RF,HIGH);
  digitalWrite(D_RR,HIGH);

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
  //making gnd low for operation
  digitalWrite(brk,LOW);

  //Making Direction pins High.
  digitalWrite(D_LF,HIGH);
  digitalWrite(D_LR,HIGH);
  digitalWrite(D_RF,HIGH);
  digitalWrite(D_RR,HIGH);


  //Giving value to PWM pins.
  analogWrite(P_LF,100);
  analogWrite(P_LR,100);
  analogWrite(P_RF,100);
  analogWrite(P_RR,100);
  
   }
else if(lin_speed < 0) 
{
   //making gnd low for operation
  digitalWrite(brk,LOW);

  //Making Direction pins Low.
  digitalWrite(D_LF,LOW);
  digitalWrite(D_LR,LOW);
  digitalWrite(D_RF,LOW);
  digitalWrite(D_RR,LOW);


  //Giving value to PWM pins.
  analogWrite(P_LF,100);
  analogWrite(P_LR,100);
  analogWrite(P_RF,100);
  analogWrite(P_RR,100);
  
  }

else
{
   //No Input,thus keep the bot stationary by making ground pin High.                                  
  digitalWrite(brk,HIGH);

  //IF above conditions are not satisfied stoping the bot by Making PWM pins Zero.
  
  //Making Direction pins High.
  digitalWrite(D_LF,HIGH);
  digitalWrite(D_LR,HIGH);
  digitalWrite(D_RF,HIGH);
  digitalWrite(D_RR,HIGH);
  
  analogWrite(P_LF,0);
  analogWrite(P_LR,0);
  analogWrite(P_RF,0);
  analogWrite(P_RR,0);

  }    
 }


else if(ang_speed != 0 && lin_speed == 0)                         //conditions if angular velocity is given
{
  if(ang_speed > 0)                                               // condition when velocity is positive
{   
                                                              
  //making gnd low for operation
  digitalWrite(brk,LOW);

  //Left wheels high,right wheel loww
  digitalWrite(D_LF,LOW);
  digitalWrite(D_LR,LOW);
  digitalWrite(D_RF,HIGH);
  digitalWrite(D_RR,HIGH);

  //Giving value to PWM pins.
  analogWrite(P_LF,50);
  analogWrite(P_LR,50);
  analogWrite(P_RF,50);
  analogWrite(P_RR,50);
    }
 else if(ang_speed < 0)
{

  //making gnd low for operation
  digitalWrite(brk,LOW);

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
   //No Input,thus keep the bot stationary by making ground pin High.                                  
  digitalWrite(brk,HIGH); 

  //IF above conditions are not satisfied stoping the bot by Making PWM pins Zero.
  
  analogWrite(P_LF,0);
  analogWrite(P_LR,0);
  analogWrite(P_RF,0);
  analogWrite(P_RR,0);

  //Making Direction pins High.
  digitalWrite(D_LF,HIGH);
  digitalWrite(D_LR,HIGH);
  digitalWrite(D_RF,HIGH);
  digitalWrite(D_RR,HIGH);
 
  }
 }
}

// Encoder Data calculation for motor_1 (Left front ).
float pulse_counter1=0;
//ISR - Rising edge

void motor_1_distance()
{
  switch (digitalRead(50))
  {
   
  case HIGH:  //Serial.print("clockwise");
              pulse_counter1--;
              break;
            
  case LOW:   pulse_counter1++;
              break;
  
  default:    break;
  }
}

// Encoder Data calculation for motor_2 (right front).
float pulse_counter2=0;
//ISR - Rising edge

void motor_2_distance()
{
  switch (digitalRead(51))
  {
   
  case HIGH:  //Serial.print("clockwise");
              pulse_counter2++;
              break;
            
  case LOW:   pulse_counter2--;
              break;
  
  default:    break;
  }
}

// Encoder Data calculation for motor_3 (left back).
float pulse_counter3=0;
//ISR - Rising edge

void motor_3_distance()
{
  switch (digitalRead(52))
  {
   
  case HIGH:  //Serial.print("clockwise");
              pulse_counter3++;
              break;
            
  case LOW:   pulse_counter3--;
              break;
  
  default:    break;
  }
}

// Encoder Data calculation for motor_4.
float pulse_counter4=0;
//ISR - Rising edge

void motor_4_distance()
{
  switch (digitalRead(53))
  {
   
  case HIGH:  //Serial.print("clockwise");
              pulse_counter4++;
              break;
            
  case LOW:   pulse_counter4--;
              break;
  
  default:    break;
  }
}

ros::NodeHandle nh;                                               // Instantiating the node handle
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&cmdVelCb);  //Subscribing to cmd_vel and calling callback function
geometry_msgs::TransformStamped  t;
tf::TransformBroadcaster broadcaster;

dec_description::odom_data odom_vl;
ros::Publisher p("/vl",&odom_vl);

//char base_link[] = "/base_link";
//char odom[] = "/odom";


void setup() {                                                   // Variable Name: Description of the variable and the range of expected values of the variable. 
   //Serial.begin(9600);
 nh.getHardware()-> setBaud(115200);                               //setting the baud rate of Arduino.
 //Assigning pinmodes
 pinMode(P_LF, OUTPUT);                                          //PWM PIN
 pinMode(P_LR, OUTPUT);                                          //PWM PIN
 pinMode(P_RF, OUTPUT);                                          //PWM PIN
 pinMode(P_RR, OUTPUT);                                          //PWM PIN
 pinMode(D_LF, OUTPUT);                                          //Direction PIN
 pinMode(D_LR, OUTPUT);                                          //Direction PIN
 pinMode(D_RF, OUTPUT);                                          //Direction PIN
 pinMode(D_RR, OUTPUT);                                          //Direction PIN
 //pinMode(B_LF, OUTPUT);                                          //Breaking PIN
 //pinMode(B_LR, OUTPUT);                                          //Breaking PIN
 //pinMode(B_RF, OUTPUT);                                          //Breaking PIN
 //pinMode(B_RR, OUTPUT);                                          //Breaking PIN
 pinMode(brk, OUTPUT);                                          //Breaking PIN

 pinMode(50,INPUT);                                              //Encoder 1 Channel B input pulse-
 pinMode(2,INPUT_PULLUP);                                        //Attaching interrupt from encoder 1 channel A
 attachInterrupt(digitalPinToInterrupt(2),motor_1_distance,RISING);

 pinMode(51,INPUT);                                              //Encoder 2 Channel B input pulse
 pinMode(3,INPUT_PULLUP);                                        //Attaching interrupt from encoder 2 channel A
 attachInterrupt(digitalPinToInterrupt(3),motor_2_distance,RISING);

 pinMode(52,INPUT);                                              //Encoder 3 Channel B input pulse
 pinMode(18,INPUT_PULLUP);                                        //Attaching interrupt from encoder 3 channel A
 attachInterrupt(digitalPinToInterrupt(18),motor_3_distance,RISING);

 pinMode(53,INPUT);                                              //Encoder 4 Channel B input pulse
 pinMode(19,INPUT_PULLUP);                                        //Attaching interrupt from encoder 4 channel A
 attachInterrupt(digitalPinToInterrupt(19),motor_4_distance,RISING);

nh.initNode();
nh.subscribe(sub);                                               //Subscribing to sub.
broadcaster.init(nh);
nh.advertise(p);

}

void loop() {
  
                                                   //ros::spinOnce() will handle passing messages to the subscriber callback. 
 //Encoder distance from pulses-
 /*motor_1 = pulse_counter1*0.00327;
 motor_2 = pulse_counter2*0.00327;
 motor_3 = pulse_counter3*0.00327;
 motor_4 = pulse_counter4*0.00327;
*/
counter1_hold=pulse_counter1;
counter2_hold=pulse_counter2;
counter3_hold=pulse_counter3;
counter4_hold=pulse_counter4;

left_encoder=(counter1_hold+counter3_hold)/2;             
right_encoder=(counter2_hold-counter4_hold)/2;  

//left_encoder=counter1_hold;
//right_encoder=counter2_hold;
//
//distance = (left_encoder + right_encoder) / 2.0;
//theta = (left_encoder - right_encoder) / wheel_base;
//
//X_position = distance * sin(theta)*0.1;
//Y_position = distance * cos(theta)*0.1;


double x = 0.0;
double y = 0.0;
double th = 0.0;

//double vx = X_position;
//double vy = Y_position;
//double vth = theta;

//Serial.print(left_encoder);
//Serial.print(X_position);
//Serial.println(pulse_counter1);

odom_vl.vx = pulse_counter1;
odom_vl.vy = right_encoder;
odom_vl.vth = theta;
odom_vl.header.seq = 0;
odom_vl.header.stamp = nh.now();

p.publish(&odom_vl);
nh.spinOnce();

}
