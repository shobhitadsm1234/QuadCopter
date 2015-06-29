#include "I2Cdev.h"
#include "MPU6050.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
  
void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup)(400, true);
  #endif
  
  
  Serial.begin(38400);
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  //accelgyro.setXGyroOffset(4061);
}

float angle_pitch;
float angle_roll;
float timeBeginPitch = millis();
float timeBeginRoll = millis();
float pitch;
float roll;

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gx = (gx/131)-31;
  gy = gy/131;
  gz = gz/131;
  /*Serial.print("a/g:\t");
  Serial.print((float)ax/16384); Serial.print("\t");
  Serial.print((float)ay/16384); Serial.print("\t");
  Serial.print((float)az/16384); Serial.print("\t");*/
  //Serial.print(gx); Serial.print("\t");
  //Serial.print(gy); Serial.print("\t");
  //Serial.print(gz); Serial.print("\t");
  //Serial.print(pitch); Serial.print("\t");
  //Serial.println(roll);
  calc_xy_angles((float)ax/16384, (float)ay/16384, (float)az/16384);
  Serial.print(comp_filter_roll(roll, gz)); Serial.print("\t");
  Serial.println(comp_filter_pitch(pitch, gy));
  delay(20);
}


void calc_xy_angles(float accel_value_x, float accel_value_y, float accel_value_z){
   // Using x y and z from accelerometer, calculate x and y angles
   float x_val, y_val, z_val, result;
   float accel_center_x = .05;
   float accel_center_y = -.02;
   float accel_center_z = -.06;
   unsigned long x2, y2, z2; //24 bit

   // Lets get the deviations from our baseline
   x_val = (float)accel_value_x-(float)accel_center_x;
   y_val = (float)accel_value_y-(float)accel_center_y;
   z_val = (float)accel_value_z-(float)accel_center_z;
   
   roll = atan(y_val/sqrt(pow(x_val,2) + pow(z_val,2)));
   pitch = atan(z_val/sqrt(pow(y_val,2) + pow(x_val,2))); 
   
   pitch = pitch * (180.0/PI);
   roll = roll * (180.0/PI) ;

   //Serial.print(pitch); Serial.print("\t");
   //Serial.println(roll);
}


float comp_filter_pitch(float accel, float gyro) {
  float dt = ((timeBeginPitch - millis())/1000) * -1;
  timeBeginPitch = millis();
  
  angle_pitch = (0.98) * (angle_pitch + (gyro * dt)) + (.02) * accel;
  
  return angle_pitch;
}

float comp_filter_roll(float accel, float gyro) {
  float dt = ((timeBeginRoll - millis())/1000);
  timeBeginRoll = millis();
  
  angle_roll = (0.98) * (angle_roll + (gyro * dt)) + (.02) * accel;
  
  return angle_roll;
}
