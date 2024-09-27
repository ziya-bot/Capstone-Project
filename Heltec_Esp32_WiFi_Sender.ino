#include <WiFi.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// WiFi credentials
const char* ssid = "wifi name";
const char* password = "wifi password";

// Server IP address and port
const char* serverIP = "esp ip address";  // The ESP IP address
const int serverPort = port numb;             // The server port, usually 80 for HTTP

// Pin definitions
#define TURBIDITY_PIN 05  // Turbidity sensor connected to analog pin GPIO 05
#define PH_PIN 19         // pH sensor connected to analog pin GPIO 19
#define ONE_WIRE_BUS 20   // DS18B20 temperature sensor connected to GPIO 21

// Set up OneWire and DallasTemperature library
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Start up the DallasTemperature library
  sensors.begin();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to WiFi");
}

void loop() {
  // Variables to store sensor readings
  float turbidityVoltage = 0;
  float NTU = 0;
  float phVoltage = 0;
  float ph = 0;

  // Request temperature from DS18B20 sensor
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);  // Get temperature in Celsius

  // Read turbidity value from KEYSTUDIO Turbidity sensor 
  int turbidityValue = analogRead(TURBIDITY_PIN);
  if (turbidityValue < 4095) {  // Check if the pin is not floating
    turbidityVoltage = turbidityValue * (5.0 / 4095.0);  // Using 5.0V reference
    NTU = map(turbidityValue, 0, 4095, 3000, 0);  // Example linear conversion
    Serial.print("Turbidity Voltage: ");
    Serial.print(turbidityVoltage);
    Serial.print(" V, NTU: ");
    Serial.println(NTU);
  } else {
    Serial.println("Turbidity sensor not connected.");
  }

  // Read pH value from DFRobot Gravity: Analog pH meter V2 Sensor
  int phValue = analogRead(PH_PIN);
  if (phValue < 4095) {  // Check if the pin is not floating
    phVoltage = phValue * (5.0 / 4095.0);  // Using 5.0V reference
    ph = map(phValue, 0, 4095, 0, 14);  // Example mapping for illustrative purposes
    Serial.print("pH Voltage: ");
    Serial.print(phVoltage);
    Serial.print(" V, pH: ");
    Serial.println(ph);
  } else {
    Serial.println("pH sensor not connected.");
  }

  // Print temperature reading
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  // Send the sensor data to the server
  sendData(turbidityVoltage, NTU, phVoltage, ph, temperatureC);

  delay(10000);  // Wait for 4 seconds before the next reading
}

void sendData(float turbidityVoltage, float ntu, float phVoltage, float ph, float temperatureC) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String("http://") + serverIP + ":" + String(serverPort) + "/receive";
    Serial.println(url);
    
    http.begin(url);  // Specify destination
    http.addHeader("Content-Type", "text/plain");  // Set content type to plain text
    
    // Create POST data in plain text format
    String postData = "Temperature: " + String(temperatureC) + " °C, " +
                      "Turbidity Voltage: " + String(turbidityVoltage) + " V, NTU: " + String(ntu) + ", " +
                      "pH Voltage: " + String(phVoltage) + " V, pH: " + String(ph);
    
    // Send POST request with data
    int httpResponseCode = http.POST(postData);

    // Check for server response
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } else {
      Serial.print("Error sending POST: ");
      Serial.println(httpResponseCode);
    }

    // End HTTP connection
    http.end();
  } else {
    Serial.println("Error in WiFi connection");
  }
}
