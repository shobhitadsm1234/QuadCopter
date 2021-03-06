//NOTE: This code requires I2Cdev and MPU6050 libaries found here: https://github.com/jrowberg/i2cdevlib
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;  //Create new MPU6050 object called accelgyro

//Define variables to store gyroscope and accelerometer data
int16_t ax, ay, az;
int16_t gx, gy, gz;
  
void setup() {
  Wire.begin();
  Serial.begin(38400);
  
  //Initialize MPU6050 and check to see if it connected successfully
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

//Define variables
float angle_pitch, angle_roll, timeBeginPitch = millis(), timeBeginRoll = millis(), pitch, roll;

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //Get raw data values
  gx = (gx/131)-31;  //Divide gyroscope data by 131 to get degrees per second (-31 is an offset for this specific module)
  gy = gy/131;
  gz = gz/131;
  
  calc_xy_angles((float)ax/16384, (float)ay/16384, (float)az/16384);  //Calculate angles from accelerometer
  
  //Print out pitch and roll
  Serial.print(comp_filter_pitch(pitch, gy)); Serial.print("\t");
  Serial.println(comp_filter_roll(roll, gz));
  
  delay(20);
}


//Function to calculate angles based off of accelerometer
void calc_xy_angles(float accel_value_x, float accel_value_y, float accel_value_z){
  
  float x_val, y_val, z_val, result;
  float accel_center_x = .05, accel_center_y = -.02, accel_center_z = -.06;  //Offsets to zero acceleromter data
  unsigned long x2, y2, z2;
   
  //Find value after removing offset
  x_val = (float)accel_value_x-(float)accel_center_x;
  y_val = (float)accel_value_y-(float)accel_center_y;
  z_val = (float)accel_value_z-(float)accel_center_z;
  
  //Use trigonometry to calculate angles from accelerations
  roll = atan(y_val/sqrt(pow(x_val,2) + pow(z_val,2)));
  pitch = atan(z_val/sqrt(pow(y_val,2) + pow(x_val,2))); 
  
  //Convert angles from radians to degrees
  pitch = pitch * (180.0/PI);
  roll = roll * (180.0/PI) ;
}


//Functions to combine gyroscope and acceleromter data
float comp_filter_pitch(float accel, float gyro) {
  float dt = ((timeBeginPitch - millis())/1000) * -1; //Calculate time since last run
  timeBeginPitch = millis(); //Reset last run time
  
  angle_pitch = (0.98) * (angle_pitch + (gyro * dt)) + (.02) * accel; //Combine gyro and accelerometer data using complimentary filter
  
  return angle_pitch; //Return value
}

float comp_filter_roll(float accel, float gyro) {
  float dt = ((timeBeginRoll - millis())/1000); //Calculate time since last run
  timeBeginRoll = millis(); //Reset last run time
  
  angle_roll = (0.98) * (angle_roll + (gyro * dt)) + (.02) * accel; //Combine gyro and acceleromter data using complimntary filter
  
  return angle_roll; //Return value
}
