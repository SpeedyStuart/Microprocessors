void setup() {
  // put your setup code here, to run once:
  pinMode(1, FUNCTION_3);
  pinMode(3, FUNCTION_3);
  pinMode(LED_BUILTIN, OUTPUT);  
  pinMode(1, OUTPUT);  
  pinMode(3, OUTPUT);  
}

void loop() {
  
  digitalWrite(1, LOW);
  digitalWrite(3, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000); 

  digitalWrite(1, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
}
