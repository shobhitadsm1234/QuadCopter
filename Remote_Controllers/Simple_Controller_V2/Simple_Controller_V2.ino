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
  Serial.println("H");
}

//Setup variables
int lastPotValue = 0;

void loop() {
  int potValue = analogRead(A0); //Read from potentiometer
  potValue = map(potValue, 0, 1023, 0, 149); //Map potentiometer value to acceptable motor values
  
  if (potValue > (lastPotValue + 1) || potValue < (lastPotValue-1)) { //Check to see if the value has changed since the last run of the loop
    lastPotValue = potValue;
    
    char str[4];
    
    itoa(potValue, str, 10); //Convert integer to string for sending
    
    char toBeSent[40]; //Create variable to hold string to send
    toBeSent[0] = '\0';
    
    //Bulid string to be sent
    strcat(toBeSent, str);
    strcat(toBeSent, ",");
    strcat(toBeSent, str);
    strcat(toBeSent, ",");
    strcat(toBeSent, str);
    strcat(toBeSent, ",");
    strcat(toBeSent, str);
    strcat(toBeSent, "\n");
    
    Serial.print(toBeSent);
  }
  
  delay(50);
}
