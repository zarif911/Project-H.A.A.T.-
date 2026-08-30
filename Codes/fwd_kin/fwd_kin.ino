const int cutoff = 10;   // noise threshold

void setup() {
  Serial.begin(9600);
}

void loop() {
  int raw = analogRead(A0);

  if (raw <= cutoff) {
    raw = 0;   // kill noise
  }

  Serial.println(raw);
  delay(10);
}
