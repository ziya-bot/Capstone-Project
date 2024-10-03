#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// LoRa settings
#define RF_FREQUENCY                                433000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 100       // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;
bool lora_idle = true;

// Sensor pin definitions
#define PH_PIN 2        // pH sensor connected to GPIO 2
#define TURBIDITY_PIN 1 // Turbidity sensor connected to GPIO 1
#define ONE_WIRE_BUS 20  // Data wire for DS18B20 temperature sensor connected to GPIO 21

// OneWire instance for temperature sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Variables for sensor readings
float turbidityVoltage = 0;
float NTU = 0;
float phVoltage = 0;
float ph = 0;
float temperatureC = 0;

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);

// Start time when the device starts (in milliseconds)
unsigned long startTime;

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    txNumber = 0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    // Start DallasTemperature library for DS18B20
    sensors.begin();

    // Get the start time (in milliseconds since the ESP32 was powered on)
    startTime = millis();
}

void loop() {
    if (lora_idle) {
        // Read turbidity sensor values
        int turbidityValue = analogRead(TURBIDITY_PIN);
        if (turbidityValue < 4095) {
            turbidityVoltage = turbidityValue * (5.0 / 4095.0); // Convert ADC to voltage
            NTU = map(turbidityValue, 0, 4095, 3000, 0);        // Convert voltage to NTU
        } else {
            Serial.println("Turbidity sensor not connected.");
        }

        // Read pH sensor values
        int phValue = analogRead(PH_PIN);
        if (phValue < 4095) {
            phVoltage = phValue * (5.0 / 4095.0); // Convert ADC to voltage
            ph = map(phValue, 0, 4095, 0, 14);    // Convert voltage to pH value
        } else {
            Serial.println("pH sensor not connected.");
        }

        // Read temperature from DS18B20 sensor
        sensors.requestTemperatures(); // Request temperature readings
        temperatureC = sensors.getTempCByIndex(0); // Get temperature in Celsius

        // Get the time since the device started (in seconds)
        unsigned long elapsedTime = (millis() - startTime) / 1000;

        // Format the data into txpacket including temperature, turbidity, pH, and timestamp
        snprintf(txpacket, sizeof(txpacket), 
                 "Temp: %.2f°C, Turbidity: %.2f NTU, pH: %.2f, Time: %lus", 
                 temperatureC, NTU, ph, elapsedTime);

        Serial.printf("\r\nSending packet \"%s\", length %d\r\n", txpacket, strlen(txpacket));

        // Send the packet via LoRa
        Radio.Send((uint8_t *)txpacket, strlen(txpacket));
        lora_idle = false;

        // Wait 10 seconds before sending the next packet
        delay(10000);
    }

    // Handle any LoRa events
    Radio.IrqProcess();
}

void OnTxDone(void) {
    Serial.println("TX done......");
    lora_idle = true;
}

void OnTxTimeout(void) {
    Radio.Sleep();
    Serial.println("TX Timeout......");
    lora_idle = true;
}
