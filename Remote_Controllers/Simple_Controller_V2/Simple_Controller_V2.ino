void setup() {
  //
  //Start heartbeat timer setup
  //
  noInterrupts();
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 31249;//(must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
  //
  //End heartbeat timer setup
  //
  
  Serial.begin(9600);
}

ISR(TIMER1_COMPA_vect) {
  Serial.print("H");
}

//Setup variables
int lastPotValue = 0;
int lastUDValue = 0;
int lastLRValue = 0;

void loop() {
  int potValue = analogRead(A0); //Read from potentiometer
  int UDPot = analogRead(A1) + 17;
  int LRPot = analogRead(A2) + 2;
  
  UDPot = map(UDPot, 0, 1023, -10, 10);
  LRPot = map(LRPot, 0, 1023, -10, 10);
  potValue = map(potValue, 0, 1023, 0, 149); //Map potentiometer value to acceptable motor values
  
  
  
  if (potValue > (lastPotValue + 1) || potValue < (lastPotValue-1) || UDPot != lastUDValue || LRPot != lastLRValue) { //Check to see if the value has changed since the last run of the loop
    lastPotValue = potValue;
    lastUDValue = UDPot;
    lastLRValue = LRPot;
    
    Serial.print(potValue); Serial.print(",");
    Serial.print(UDPot); Serial.print(",");
    Serial.print(LRPot); Serial.print("\n");
  
    /*char altStr[4];
    char UDStr[3];
    char LRStr[3];
    
    itoa(potValue, altStr, 10); //Convert integer to string for sending
    itoa(UDPot, UDStr, 10);
    itoa(LRPot, LRStr, 10);
    
    char toBeSent[40]; //Create variable to hold string to send
    toBeSent[0] = '\0';
    
    //Bulid string to be sent
    strcat(toBeSent, altStr);
    strcat(toBeSent, ",");
    strcat(toBeSent, UDStr);
    strcat(toBeSent, ",");
    strcat(toBeSent, LRStr);
    strcat(toBeSent, "\n");
    
    Serial.print(toBeSent);*/
  }
  
  delay(100);
}
