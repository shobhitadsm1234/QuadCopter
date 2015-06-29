void setup() {
  Serial.begin(9600);
}

char str1[4];
char str2[4];
char str3[4];
char str4[4];

void loop() {
  int mainPotValue = analogRead(A4);
  int pot1Value = analogRead(A3);
  int pot2Value = analogRead(A2);
  int pot3Value = analogRead(A1);
  int pot4Value = analogRead(A0);
  
  mainPotValue = map(mainPotValue, 0, 1023, 0, 179);
  pot1Value = map(pot1Value, 0, 1023, 0, 179);
  pot2Value = map(pot2Value, 0, 1023, 0, 179);
  pot3Value = map(pot3Value, 0, 1023, 0, 179);
  pot4Value = map(pot4Value, 0, 1023, 0, 179);
  
  int motor1;
  int motor2;
  int motor3;
  int motor4;
  
  if((mainPotValue - pot1Value) > 0) {
    motor1 = mainPotValue - pot1Value;
  } else {
    motor1 = 0;
  }
  
  if((mainPotValue - pot2Value) > 0) {
    motor2 = mainPotValue - pot2Value;
  } else {
    motor2 = 0;
  }
  
  if((mainPotValue - pot3Value) > 0) {
    motor3 = mainPotValue - pot3Value;
  } else {
    motor3 = 0;
  }
  
  if((mainPotValue - pot4Value) > 0) {
    motor4 = mainPotValue - pot4Value;
  } else {
    motor4 = 0;
  }
  
  itoa(motor1, str1, 10);
  itoa(motor2, str2, 10);
  itoa(motor3, str3, 10);
  itoa(motor4, str4, 10);
  
  char final[40];
  final[0] = '\0';
  
  strcat(final, str1);
  strcat(final, ",");
  strcat(final, str2);
  strcat(final, ",");
  strcat(final, str3);
  strcat(final, ",");
  strcat(final, str4);
  strcat(final, "\0");
  
  
  /*String string1 = String(str1);
  String string2 = String(str2);
  String string3 = String(str3);
  String string4 = String(str4);
  
  String final = string1 + "," + string2 + "," + string3 + "," + string4;
  */
  Serial.println(final);
  delay(75);
}
