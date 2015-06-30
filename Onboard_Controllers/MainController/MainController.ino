//NOTE: This code requires I2Cdev and MPU6050 libaries found here: https://github.com/jrowberg/i2cdevlib
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;  //Create new MPU6050 object called accelgyro

//Define variables to store gyroscope and accelerometer data
int16_t ax, ay, az;
int16_t gx, gy, gz;

Servo ESC1, ESC2, ESC3, ESC4;
int checkSignal = 0;  //Variable used to check if connection with controller is still active

void setup() {
  Wire.begin();
  Serial.begin(9600);  //Start Serial connection
  
  //Initialize MPU6050 and check to see if it connected successfully
  accelgyro.initialize();
  
  //Attach electronic speed controlers to their servo objects.
  ESC1.attach(8);
  ESC2.attach(9);
  ESC3.attach(10);
  ESC4.attach(11);
  
  //Start configuring minimum and maximum throttle settings
  for (int i = 0; i < 25; i++) {
    ESC1.write(179);
    ESC2.write(179);
    ESC3.write(179);
    ESC4.write(179);
    delay(75);
  }
  
  for (int i = 0; i < 25; i++) {
    ESC1.write(0);
    ESC2.write(0);
    ESC3.write(0);
    ESC4.write(0);
    delay(75);
  }
  //End configuring minimum and maximum throttle settings
}

//Define variables for acceleromter/gyroscope
float angle_pitch, angle_roll, timeBeginPitch = millis(), timeBeginRoll = millis(), pitch, roll;

//Motor values from controller
int motor1Value = 0, motor2Value = 0, motor3Value = 0, motor4Value = 0;

void loop() {
  int motor1Value_s = 0, motor2Value_s = 0, motor3Value_s = 0, motor4Value_s = 0;
  if (Serial.available()) {
    char data[20];  //Stores data coming in over serial until it can be processed
    int i=0;  //Counter variable to go through new serial data with;
    
    delay(50); //Wait to receive all data
    
    while(Serial.available() && i<20) {  //Loop through all characters in serial data
      data[i++] = Serial.read();  //Add new data to "data" one character at a time
    }
    
    checkSignal = 0;  //Reset time since last signal was received
    data[i++] = '\0';  //remove

    if(i>0) {
      char motor[4][5];  //Create two dimensional array holding all 4 motor values
      int currentMotor = 0, consecNums = 0;
      
      //Set all of motor 4 to null in order to illiminate fluctuations du to it being end of data stream
      for (int b = 0; b < 4; b++) {
        motor[3][b] = '\0';
      }
    
      for(int a = 0; a <= i; a++) {  //Go through all data received
        
        if (data[a] == ',') {  //If a comma is found figure out what motor is currently being worked on and move to next
          motor[currentMotor][consecNums] = '\0';
          currentMotor++;
          consecNums = 0;
        } else if (data[a] == '\0' || data[a] == '\n') {  //If a null character is found end data parsing
          motor[4][consecNums] = '\0';
          break;
        } else {  //If anything else is found add it to the current motors value
          motor[currentMotor][consecNums] = data[a];
          consecNums++;
        }
      }
    
      //Convert character arrays to strings
      String holdMot1 = String(motor[0]); String holdMot2 = String(motor[1]);
      String holdMot3 = String(motor[2]); String holdMot4 = String(motor[3]);
      
      //Convert strings to integers and store in motor value variables
      motor1Value = holdMot1.toInt(); motor2Value = holdMot2.toInt();
      motor3Value = holdMot3.toInt(); motor4Value = holdMot4.toInt();
    }
  }
  
  //Start stabilization code
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //Get raw data values
  gx = (gx/131)-31;  //Divide gyroscope data by 131 to get degrees per second (-31 is an offset for this specific module)
  gy = gy/131;
  gz = gz/131;
  
  calc_xy_angles((float)ax/16384, (float)ay/16384, (float)az/16384);  //Calculate angles from accelerometer
  
  pitch = (comp_filter_pitch(pitch, gy)) + 2.9;
  roll = (comp_filter_roll(roll, gz)) + 1.41;
  
  //Start of PID controller
  float actual = pitch, desired = 0, intThreshold = 10, integral, lastActual, driveValue;
  float error = desired - actual;
  float P, I, D;
  float kP = 0.2, kI = 2, kD = 0.1; //Gain values
  float scaleFactor = 3;
  
  if (abs(error < intThreshold)) { //Stop integral windup
    integral += error;
  } else {
    integral = 0;
  }
  
  //Calculate P K and I
  P = error * kP;
  I = integral * kI;
  D = (lastActual - actual) * kD;
  driveValue = P + I + D;
  driveValue = driveValue * scaleFactor; //Used to get end value to desired range;
  
  //Add change to motors
  int change;
  
  if (driveValue > 0) {
    change = driveValue / 2;
    motor1Value_s = motor1Value + change;
    motor3Value_s = motor3Value - change;
  } else {
    change = (driveValue * -1) / 2;
    motor1Value_s = motor1Value - change;
    motor3Value_s = motor3Value + change;
  }
  
  //End of PID controller
  
  /*
  //Print out pitch, roll, and driveValue
  Serial.print(pitch); Serial.print("\t");
  Serial.print(roll); Serial.print("\t");
  Serial.println(driveValue);*/
  
  //Write motor values to motors
  ESC1.write(motor1Value_s); ESC2.write(motor2Value);
  ESC3.write(motor3Value_s); ESC4.write(motor4Value);
  
  Serial.print(motor1Value_s); Serial.print("\t");
  Serial.print(motor2Value); Serial.print("\t");
  Serial.print(motor3Value_s); Serial.print("\t");
  Serial.print(motor4Value); Serial.print("\t");
  Serial.println(change);
  
  delay(10);
  //End stabilization code
  
  //If last signal was over one minute ago then kill motors
  if(checkSignal >= 60000) {
    ESC1.write(0); ESC2.write(0); ESC3.write(0); ESC4.write(0);
  }
  
  delay(1);  //Delay 1 millisecond to wait for new data
  checkSignal++;  //Add 1 millsecond to time when data was last received
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
