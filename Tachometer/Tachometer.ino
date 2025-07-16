#include <Arduino.h>
#include <BluetoothSerial.h>

// Define pin for IR sensor
#define IR_SENSOR_PIN 15

// Bluetooth Serial object
BluetoothSerial SerialBT;

// Variables for tachometer
volatile int pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
const unsigned long debounceDelay = 10;  // Debounce delay in milliseconds

unsigned long previousMillis = 0;
const unsigned long interval = 1000;  // Interval for RPM calculation (1 second)

void IRAM_ATTR handleIRSensor() {
  unsigned long currentTime = millis();
  if ((currentTime - lastPulseTime) > debounceDelay) {
    pulseCount++;
    lastPulseTime = currentTime;
  }
}

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);

  // Start Bluetooth Serial
  SerialBT.begin("ESP32_Tachometer");  // Bluetooth device name

  // Set up IR sensor pin
  pinMode(IR_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), handleIRSensor, FALLING);

  Serial.println("Tachometer with IR sensor and Bluetooth initialized.");
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    // Calculate RPM
    unsigned long rpm = (pulseCount * 60) / 2;  // Assuming 2 pulses per revolution

    // Send RPM data over Bluetooth
    SerialBT.print("RPM: ");
    SerialBT.println(rpm);

    // Reset pulse count and timestamp
    pulseCount = 0;
    previousMillis = currentMillis;
  }
}
