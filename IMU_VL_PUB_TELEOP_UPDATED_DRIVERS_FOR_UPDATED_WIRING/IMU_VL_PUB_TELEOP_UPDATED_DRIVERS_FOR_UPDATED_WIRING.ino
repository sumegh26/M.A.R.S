#include<ros.h>
#include<std_msgs/String.h>
#include<dec_description/odom_data.h>
#include<geometry_msgs/Twist.h>
#include <ros/time.h>
#include<tf/tf.h>
#include<tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <sensor_msgs/Imu.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

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
#define brk_LF 8
#define brk_LR 24
#define brk_RF 23
#define brk_RR 30


//Global variables for encoder ticks
float motor_1;
float motor_2;
float motor_3;
float motor_4;
float motor;
float wheel_base = 0.80;
float theta;
float distance;
float left_encoder;
float right_encoder;

float  X_position;
float  Y_position;
float counter1_hold,counter2_hold,counter3_hold,counter4_hold;

MPU6050 mpu;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
dec_description::odom_data odom_vl;
ros::Publisher p("/vl",&odom_vl);

ros::Publisher imu_data("/imu_data", &imu_msg);
char frameid[] = "/base_link";
char child[] = "/imu_frame";

bool dmpReady = true;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = true;
void dmpDataReady() {
    mpuInterrupt = true;
}

void cmdVelCb( const geometry_msgs::Twist& twist)                          // Variable Name: ang_speed  lin_speed
{ //callback function
  float ang_speed;
  float lin_speed;
  
  ang_speed=twist.angular.z;
  lin_speed=twist.linear.x;

//IF-ELSE CONDITIONS FOR TELEOP

if(ang_speed == 0 && lin_speed == 0)     
{    
  //break
    //Making PWM pins Zero.
  analogWrite(P_LF,0);
  analogWrite(P_LR,0);
  analogWrite(P_RF,0);
  analogWrite(P_RR,0);
  
  digitalWrite(brk_LF,HIGH);
  digitalWrite(brk_LR,HIGH);
  digitalWrite(brk_RF,HIGH);
  digitalWrite(brk_RR,HIGH);

  
  //Making Direction pins High.
  digitalWrite(D_LF,HIGH);
  digitalWrite(D_LR,HIGH);
  digitalWrite(D_RF,HIGH);
  digitalWrite(D_RR,HIGH);



  }


else if(ang_speed == 0 && lin_speed != 0)             //conditions if linear velocity is given
{
  if(lin_speed > 0)
{ 
  //making gnd low for operation
  //digitalWrite(brk,LOW);
  digitalWrite(brk_LF,LOW);
  digitalWrite(brk_LR,LOW);
  digitalWrite(brk_RF,LOW);
  digitalWrite(brk_RR,LOW);

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
  //digitalWrite(brk,LOW);
     //digitalWrite(brk,LOW);
  digitalWrite(brk_LF,LOW);
  digitalWrite(brk_LR,LOW);
  digitalWrite(brk_RF,LOW);
  digitalWrite(brk_RR,LOW);


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
 // digitalWrite(brk,HIGH);
     //digitalWrite(brk,LOW);
  analogWrite(P_LF,0);
  analogWrite(P_LR,0);
  analogWrite(P_RF,0);
  analogWrite(P_RR,0);

     
    digitalWrite(brk_LF,HIGH);
  digitalWrite(brk_LR,HIGH);
  digitalWrite(brk_RF,HIGH);
  digitalWrite(brk_RR,HIGH);


  //IF above conditions are not satisfied stoping the bot by Making PWM pins Zero.
  
  //Making Direction pins High.
  digitalWrite(D_LF,HIGH);
  digitalWrite(D_LR,HIGH);
  digitalWrite(D_RF,HIGH);
  digitalWrite(D_RR,HIGH);
  

  }    
 }


else if(ang_speed != 0 && lin_speed == 0)                         //conditions if angular velocity is given
{
  if(ang_speed > 0)                                               // condition when velocity is positive
{   
                                                              
  //making gnd low for operation
 // digitalWrite(brk,LOW);
     //digitalWrite(brk,LOW);
  digitalWrite(brk_LF,LOW);
  digitalWrite(brk_LR,LOW);
  digitalWrite(brk_RF,LOW);
  digitalWrite(brk_RR,LOW);


  //Left wheels high,right wheel loww
  digitalWrite(D_LF,LOW);
  digitalWrite(D_LR,LOW);
  digitalWrite(D_RF,HIGH);
  digitalWrite(D_RR,HIGH);

  //Giving value to PWM pins.
  analogWrite(P_LF,180);
  analogWrite(P_LR,180);
  analogWrite(P_RF,200);
  analogWrite(P_RR,200);
    }
 else if(ang_speed < 0)
{

  //making gnd low for operation
  //digitalWrite(brk,LOW);
     //digitalWrite(brk,LOW);
    digitalWrite(brk_LF,LOW);
  digitalWrite(brk_LR,LOW);
  digitalWrite(brk_RF,LOW);
  digitalWrite(brk_RR,LOW);


  //Left wheels Low,right wheels High
  digitalWrite(D_LF,HIGH);
  digitalWrite(D_LR,HIGH);
  digitalWrite(D_RF,LOW);
  digitalWrite(D_RR,LOW);

  //Giving value to PWM pins.
  analogWrite(P_LF,200);
  analogWrite(P_LR,200);
  analogWrite(P_RF,180);
  analogWrite(P_RR,180);

 }
 else
{
   //No Input,thus keep the bot stationary by making ground pin High.                                  
  //digitalWrite(brk,HIGH);
    analogWrite(P_LF,0);
  analogWrite(P_LR,0);
  analogWrite(P_RF,0);
  analogWrite(P_RR,0);
   
     //digitalWrite(brk,LOW);
  digitalWrite(brk_LF,HIGH);
  digitalWrite(brk_LR,HIGH);
  digitalWrite(brk_RF,HIGH);
  digitalWrite(brk_RR,HIGH);


  //IF above conditions are not satisfied stoping the bot by Making PWM pins Zero.
  


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

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&cmdVelCb);  //Subscribing to cmd_vel and calling callback function


void setup() {
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
// pinMode(brk, OUTPUT);                                          //Breaking PIN

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


//IMU config
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    


nh.initNode();
nh.subscribe(sub);                                               //Subscribing to sub.
broadcaster.init(nh);
nh.advertise(p);
nh.advertise(imu_data);
mpu.initialize();
devStatus = mpu.dmpInitialize();    
    if (devStatus == 0) {    
        mpu.setDMPEnabled(true);
       // attachInterrupt(20, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
                         }
}



void loop() {

   if (!dmpReady) return;
nh.spinOnce();
    
    while (!mpuInterrupt && fifoCount < packetSize) {}

    mpuInterrupt = true;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        mpu.resetFIFO();
    } 
    else if (mpuIntStatus & 0x01) 
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;            
            
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

            imu_msg.orientation.x = q.x;
            imu_msg.orientation.y = q.y;
            imu_msg.orientation.z = q.z;
            imu_msg.orientation.w = q.w;
            imu_msg.orientation_covariance[0] = -1;  /////////////change

            imu_msg.angular_velocity.x = ypr[0];
            imu_msg.angular_velocity.y = ypr[1];
            imu_msg.angular_velocity.z = ypr[2];  
            imu_msg.angular_velocity_covariance[0] = -1;   //////////////change

            imu_msg.linear_acceleration.x = aaReal.x * 1/16384. * 9.80665;
            imu_msg.linear_acceleration.y = aaReal.y * 1/16384. * 9.80665;
            imu_msg.linear_acceleration.z = aaReal.z * 1/16384. * 9.80665;
            imu_msg.linear_acceleration_covariance[0] = -1;

             //Assigning values to TF header and publish the transform
            imu_msg.header.seq = 0;
            imu_msg.header.stamp = nh.now();
            imu_msg.header.frame_id= frameid;
            
            

            //Assigning values to TF header and publish the transform
            t.header.frame_id = frameid;
            t.child_frame_id = child;
            t.transform.translation.x = 0.005; 
            t.transform.translation.z = 0.055; 

            t.transform.rotation.x = q.x;
            t.transform.rotation.y = q.y; 
            t.transform.rotation.z = q.z; 
            t.transform.rotation.w = q.w;  
            t.header.stamp = nh.now();
            broadcaster.sendTransform(t);
            

            imu_msg.header.seq++;
            imu_msg.header.frame_id=frameid;

            imu_data.publish(&imu_msg);
            nh.spinOnce();
   
            delay(200);     
     }

counter1_hold=pulse_counter1;
counter2_hold=pulse_counter2;
counter3_hold=pulse_counter3;
counter4_hold=pulse_counter4;

left_encoder=(counter1_hold+counter3_hold)/2;             
right_encoder=(counter2_hold-counter4_hold)/2; 

odom_vl.vx = pulse_counter3;
odom_vl.vy = pulse_counter4;
odom_vl.vth = theta;
//odom_vl.header.seq = 0;
//odom_vl.header.stamp = nh.now();

p.publish(&odom_vl);
nh.spinOnce();
}
