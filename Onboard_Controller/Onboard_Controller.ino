#include <Servo.h>

Servo ESC1, ESC2, ESC3, ESC4;
int checkSignal = 0;  //Variable used to check if connection with controller is still active

void setup() {
  Serial.begin(9600);  //Start Serial connection
  
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

void loop() {

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
      
      //Convert strings to integers and write values to Electronic Speed Controllers
      ESC1.write(holdMot1.toInt()); ESC2.write(holdMot2.toInt());
      ESC3.write(holdMot3.toInt()); ESC4.write(holdMot4.toInt());
    }
  }
  
  //If last signal was over 500 milliseconds ago then kill motors
  if(checkSignal >= 500) {
    ESC1.write(0); ESC2.write(0); ESC3.write(0); ESC4.write(0);
  }
  
  delay(1);  //Delay 1 millisecond to wait for new data
  checkSignal++;  //Add 1 millsecond to time when data was last received
}
