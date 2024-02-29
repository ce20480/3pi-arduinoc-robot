# define BUZZER_PIN1 A6
# define BUZZER_PIN2 6
# define BUZZER_PIN3 25

void buzzers_on (int delay_metric) {

  digitalWrite(BUZZER_PIN1, HIGH);
  delay(delay_metric);
  digitalWrite(BUZZER_PIN2, HIGH);
  delay(delay_metric);
  digitalWrite(BUZZER_PIN3, HIGH);// turn the LED on (HIGH is the voltage level)
  delay(delay_metric);                       // wait for a second
  digitalWrite(BUZZER_PIN1, LOW);
  delay(delay_metric);
  digitalWrite(BUZZER_PIN2, LOW);
  delay(delay_metric);
  digitalWrite(BUZZER_PIN3, LOW);// turn the LED off by making the voltage LOW
  delay(delay_metric);

}

void buzzer1_on (int delay_metric) {

  digitalWrite(BUZZER_PIN1, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(delay_metric);                       // wait for a second
  digitalWrite(BUZZER_PIN1, LOW); // turn the LED off by making the voltage LOW
  delay(delay_metric);

}

void setup() {
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(BUZZER_PIN1, OUTPUT);
  pinMode(BUZZER_PIN2, OUTPUT);
  pinMode(BUZZER_PIN3, OUTPUT);

}

void loop() {
  buzzers_on(1); 
}
