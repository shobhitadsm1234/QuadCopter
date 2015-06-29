void setup() {
  Serial.begin(9600);
}

char str1[4];

void loop() {
  int potValue = analogRead(A0);
  potValue = map(potValue, 0, 1023, 0, 179);
  
  itoa(potValue, str1, 10);
  
  char final[40];
  final[0] = '\0';
  
  strcat(final, str1);
  strcat(final, ",");
  strcat(final, str1);
  strcat(final, ",");
  strcat(final, str1);
  strcat(final, ",");
  strcat(final, str1);
  //strcat(final, "\0");
  strcat(final, "\n");
  
  Serial.print(final);
  delay(75);
}
