#define TURBIDITY_PIN 1  // (ADC1_CH0 on GPIO 1)
const float MIN_NTU = 1.0;  // Minimum NTU value to be used instead of 0

void setup() {
  Serial.begin(115200); // Baud rate: 115200
}

void loop() {
  int sensorValue = analogRead(TURBIDITY_PIN); // Read the input on analog pin 1
  float voltage = sensorValue * (3.3 / 4095);  // Convert the analog reading to voltage
  
  // Calculate NTU using the polynomial equation from the graph
  float ntu = (-1120.4 * pow(voltage, 2)) + (5742.3 * voltage) - 4352.9;
  
  // Set NTU to a minimum value if it is below 0
  if (ntu < MIN_NTU) {
    ntu = MIN_NTU; // Set NTU to the minimum defined value
  }
  
  // Print out the voltage and NTU value
  Serial.print("Voltage: ");
  Serial.print(voltage, 2); // Print the voltage with 2 decimal places
  Serial.print(" V, NTU: ");
  Serial.println(ntu); // Print the NTU value, now guaranteed to be at least MIN_NTU
  
  delay(500); // Wait for 500 milliseconds
}

