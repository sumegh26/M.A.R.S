#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#define BR_LF_1 2                   //break   PE_4
#define DIR_LF 3                //motor1 LEFT FRONT dir   PB_0   
#define PWM_LF 23               //motor1 pwm    PD_0
 
#define BR_RF_2 7                //break         PD_2
#define DIR_RF 4              //motor2  RIGHT FRONT   dir      PB_1
#define PWM_RF 24              //motor2 pwm     PD_1

#define BR_LB_3 27                //break         PD_2
#define DIR_LB 5              //motor3 LEFT BACK dir      PB_1
#define PWM_LB 25              //moto3 pwm     PD_1

#define BR_RB_4 8                //break         PD_2
#define DIR_RB 6              //motor4 RIGHT BACK dir      PB_1
#define PWM_RB 26              //motor4 pwm     PD_1

double wheel_rf=0;
double wheel_lf=0;
double wheel_rb=0;
double wheel_lb=0;

double wheel_radius=0.05;
double wheel_sep=0.50;
double ang_speed=0;
double lin_speed=0;
float width_robot = 79;
float wheel_circumference = 0.31415;
float diameter = 0.10;

//std_msgs::String str_msg;

  


void cmdVelCb( const geometry_msgs::Twist& twist)
{                      //callback function
 ang_speed=twist.angular.z;
 lin_speed=twist.linear.x;
// wheel_lf = (lin_speed/wheel_radius)-((ang_speed*wheel_sep)/(2.0*wheel_radius));
// wheel_rf = (lin_speed/wheel_radius)+((ang_speed*wheel_sep)/(2.0*wheel_radius));

// wheel_lb = (lin_speed/wheel_radius)-((ang_speed*wheel_sep)/(2.0*wheel_radius));
//wheel_rb = (lin_speed/wheel_radius)+((ang_speed*wheel_sep)/(2.0*wheel_radius));

double lin_speed;
double ang_speed;


  int pwm_LF = constrain(pwm_LF, 0, 255);
  int pwm_LB = constrain(pwm_LB, 0, 255);
  int pwm_RF = constrain(pwm_RF, 0, 255);
  int pwm_RB = constrain(pwm_RB, 0, 255);
  int pwm_max = 255;

  float v_LF ;
  float v_LB ;
  float v_RF ;
  float v_RB ;

  float w_LF ;
  float w_LB ;
  float w_RF ;
  float w_RB ;

  float rpm_LF = constrain(rpm_LF, 0, 255) ;
  float rpm_LB = constrain(rpm_LB, 0, 255) ;
  float rpm_RF = constrain(rpm_RF, 0, 255) ;
  float rpm_RB = constrain(rpm_RB, 0, 255) ;
  float rpm_max = 200;

  float wheel_circumference = 0.31415;
  float diameter = 0.10;

  
    if (lin_speed == 0 && ang_speed == 0) {

    //setting up the directions , setting it LOW though the default is LOW just for assurance
    digitalWrite(DIR_LF, LOW);
    digitalWrite(DIR_LB, LOW);
    digitalWrite(DIR_RF, LOW);
    digitalWrite(DIR_RB, LOW);
    //Breakpin high
    //setting up the pwm , setting it all to zero as bot will be stationary in this situation
    pwm_LF = 0;
    pwm_LB = 0;
    pwm_RF = 0;
    pwm_RB = 0;
    }
  else if (ang_speed == 0 && lin_speed != 0) {



    //Now in this condition there are two cases that the bot will move forward of backward

    //so two conditions

    //It moves in the backward direction , setting up the direction
    if (lin_speed < 0) {
      digitalWrite(DIR_LF, LOW);
      digitalWrite(DIR_LB, LOW);
      digitalWrite(DIR_RF, LOW);
      digitalWrite(DIR_RB, LOW);



      // setting up the desired speed from twist
      v_LF = lin_speed;
      v_RF = lin_speed;

      rpm_LF = rpm_LB = ((60 * v_LF) / (diameter * 3.142));
      rpm_RF = rpm_RB = ((60 * v_RF) / (diameter * 3.142));

      pwm_LF = pwm_LB = ((pwm_max / rpm_max) * rpm_LF);
      pwm_RF = pwm_RB = ((pwm_max / rpm_max) * rpm_RF);


    }
    //It moves in the forward direction
    else if (lin_speed > 0) {
      digitalWrite(DIR_LF, HIGH);
      digitalWrite(DIR_LB, HIGH);
      digitalWrite(DIR_RF, HIGH);
      digitalWrite(DIR_RB, HIGH);


      // setting up the pwm of the motors
      // setting up the desired speed from twist
      v_LF = lin_speed;
      v_RB = lin_speed;

      rpm_LF = rpm_LB = ((60 * v_LF) / (diameter * 3.142));
      rpm_RF = rpm_RB = ((60 * v_RF) / (diameter * 3.142));

      pwm_LF = pwm_LB = ((pwm_max / rpm_max) * rpm_LF);
      pwm_RF = pwm_RB = ((pwm_max / rpm_max) * rpm_RF);


    }

    //setting default conditions if the tests fails
    else {
      //setting the direction
      digitalWrite(DIR_LF, LOW);
      digitalWrite(DIR_LB, LOW);
      digitalWrite(DIR_RF, LOW);
      digitalWrite(DIR_RB, LOW);

      //setting up the pwm , setting it all to zero as bot will be stationary in this situation
      pwm_LF = 0;
      pwm_LB = 0;
      pwm_RF = 0;
      pwm_RB = 0;

    }
  }



  //For Spot turning

  else if (ang_speed != 0 && lin_speed == 0) {

    // if the angular z is positive that means it will rotate in left or anti clockwise direction
    if (ang_speed > 0) {

      //setting up the directions
      digitalWrite(DIR_LF, LOW);
      digitalWrite(DIR_LB, LOW);
      digitalWrite(DIR_RF, HIGH);
      digitalWrite(DIR_RB, HIGH);

      //setting up the pwm since its spot turning all pwms will be same
      // v= r x w where v is tangential linear velocity component
      v_LF = (ang_speed * width_robot) / 2 ;
      v_RF = (ang_speed * width_robot) / 2 ;

      rpm_LF = rpm_LB = ((60 * v_LF) / (diameter * 3.142));
      rpm_RF = rpm_RB = ((60 * v_RF) / (diameter * 3.142));


      pwm_LF = pwm_LB = ((pwm_max / rpm_max) * rpm_LF);
      pwm_RF = pwm_RB = ((pwm_max / rpm_max) * rpm_RF);

    }

    // if the angular z is negative it will rotate in right or clockwise direction
    else if (ang_speed < 0) {
      //setting up the directions
      digitalWrite(DIR_LF, HIGH);
      digitalWrite(DIR_LB, HIGH);
      digitalWrite(DIR_RF, LOW);
      digitalWrite(DIR_RB, LOW);

      //setting up the pwm since its spot turning all pwms will be same
      // v= r x w where v is tangential linear velocity component
      v_LF = (ang_speed * width_robot) / 2 ;
      v_RF = (ang_speed * width_robot) / 2 ;

      rpm_LF = rpm_LB = ((60 * v_LF) / (diameter * 3.142));
      rpm_RF = rpm_RB = ((60 * v_RF) / (diameter * 3.142));

      pwm_LF = pwm_LB = ((pwm_max / rpm_max) * rpm_LF);
      pwm_RF = pwm_RB = ((pwm_max / rpm_max) * rpm_RF);
    }
    //setting default conditions if the tests fails
    else {
      //setting the direction
      digitalWrite(DIR_LF, LOW);
      digitalWrite(DIR_LB, LOW);
      digitalWrite(DIR_RF, LOW);
      digitalWrite(DIR_RB, LOW);

      //setting up the pwm , setting it all to zero as bot will be stationary in this situation
      pwm_LF = 0;
      pwm_LB = 0;
      pwm_RF = 0;
      pwm_RB = 0;

    }

}
  analogWrite(PWM_LF, pwm_LF);
  analogWrite(PWM_LB, pwm_LB);
  analogWrite(PWM_RF, pwm_RF);
  analogWrite(PWM_LF, pwm_RB);
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCb);

void setup() {
  nh.getHardware()-> setBaud(9600);

  
  // put your setup code here, to run once:
 pinMode(PWM_LF, OUTPUT);           //PD_0        23
 pinMode(PWM_RF ,OUTPUT);           //PD_1        24
 pinMode(PWM_LB, OUTPUT);           //PD_2        25
 pinMode(PWM_RB, OUTPUT);           //PD_3        26
 pinMode(BR_LB_3,OUTPUT);           //PE_1        27
 pinMode(DIR_RF, OUTPUT);           //PB_1        4
 pinMode(DIR_LB, OUTPUT);           //PE_4        5
 pinMode(DIR_RB, OUTPUT);           //PE_5        6
 pinMode(BR_LF_1,OUTPUT);           //PB_5        2
 pinMode(BR_RF_2,OUTPUT);           //PE_1        7
 pinMode(DIR_LF, OUTPUT);           //PB_0        3
 pinMode(BR_RB_4,OUTPUT);           //PA_5        8

 
nh.initNode();
nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly: 
 
nh.spinOnce();



}
