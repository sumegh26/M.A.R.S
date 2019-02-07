

/*pin
 * 1=3.3v                       //21=VBUS      
 * 2=PB_5                         22=GND
 * 3=PB_0                         23=PD_0 SCL(3)        //no
 * 4=PB_1                         24=PD_1 SDA(3)
 * 5=PE_4   SCL(2)                25=PD_2
 * 6=PE_5   SDA(2)      //no      26=PD_3
 * 7=PB_4                         27=PE_1
 * 8=PA_5                         28=PE_2
 * 9=PA_6   SCL(1)                29=PE_3        
 * 10=PA_7  SDA(1)  //no          30=PF_1
 * 11=PA_2                        31=PF_4
 * 12=PA_3                        32=PD_7
 * 13=PA_4                        33=PD_6
 * 14=PB_6                        34=PC_7
 * 15=PB_7                        35=PC_6
 * 16=RESET                       36=PC5
 * 17=PF_0                        37=PC_4
 * 18=PE_0                        38=PB_3 SDA(0)    //not working for imu
 * 19=PB_2 SCL(0)                 39=PF_3
 * 20=GND                         40=PF_2
 * 
 * 
 */
 //USE PIN SCL1 AND SDA1
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
//#include <I2Cdev.h>

ros::NodeHandle  nh;
sensor_msgs::Imu imu_msg;
ros::Publisher pub("/imu/data_raw", &imu_msg);
char frame_id[] = "imu";





    #include "Wire.h" // This library allows you to communicate with I2C devices.
    const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
    int16_t accelerometer_x,y, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
    int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
    //int16_t quantx, quanty, quantz;
    //int16_t temperature; // variables for temperature data
   
    /*
    char tmp_str[7]; // temporary variable used in convert function
    char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
      sprintf(tmp_str, "%6d", i);
      return tmp_str;
                                          }
*/geometry_msgs::Quaternion q;/*
double gyroData;                            
double accData[6];
double quant[6];

geometry_msgs::Quaternion q[3];
geometry_msgs::Quaternion orientation;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 linear_acceleration;
*/
//Quaternion q;           // [w, x, y, z]         quaternion container
/*VectorInt16 aa; // [x, y, z] 
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; */
void setup(){  
    nh.getHardware()-> setBaud(115200);

  nh.initNode();
  nh.advertise(pub);

      //Serial.begin(9600);
      Wire.begin();
      Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
      Wire.write(0x6B); // PWR_MGMT_1 register
      Wire.write(0); // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
}

long publisher_timer;
void loop(){

  //if (millis() > publisher_timer) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 7*2=14 registers



      
      // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
      accelerometer_x =Wire.read()<<8|Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      accelerometer_y = /*Wire.read()<<8 |*/ Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
      accelerometer_z = /*Wire.read()<<8 |*/ Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
      //temperature = Wire.read()<<8 |*/ Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
      gyro_x = /*Wire.read()<<8 |*/ Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
      gyro_y = /*Wire.read()<<8 |*/ Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
      gyro_z = /*Wire.read()<<8 |*/ Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

//  Quaternion q.x=(Quaternion)accelerometer_x;

  imu_msg.header.seq = 0;
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id= frame_id;


  imu_msg.orientation.x = accelerometer_x;
  imu_msg.orientation.y =accelerometer_y;
  imu_msg.orientation.z = accelerometer_z;
  imu_msg.orientation.w = 0;
  imu_msg.orientation_covariance[0] = -1;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0;


  imu_msg.angular_velocity.x = accelerometer_x;
  imu_msg.angular_velocity.y = gyro_y;
  imu_msg.angular_velocity.z = gyro_z;
  imu_msg.angular_velocity_covariance[0] = -1;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0;


  imu_msg.linear_acceleration.x = accelerometer_x;
  imu_msg.linear_acceleration.y = gyro_y;
  imu_msg.linear_acceleration.z = gyro_z;
  imu_msg.linear_acceleration_covariance[0] = -1;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0;
  imu_msg.header.seq++;
  imu_msg.header.frame_id=frame_id;
//imu_msg.header.stamp = nh.now();

  // Doesn't work:
  pub.publish(&imu_msg);

 
 nh.spinOnce(); 
 // delay(500);
}
