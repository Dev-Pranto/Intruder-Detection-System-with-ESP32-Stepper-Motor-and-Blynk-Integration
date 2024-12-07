#define BLYNK_TEMPLATE_ID "TMPL6nU1rE_t9"
#define BLYNK_TEMPLATE_NAME "Intruder Detection System"
#define BLYNK_AUTH_TOKEN "uPieI1N4MOJIO_PxcSXsP6UeSSrRc3Zw"

#include <LiquidCrystal.h>       // Include the LiquidCrystal library
#include <DHT.h>                 // Include the DHT library
#include <WiFi.h>                // Include the WiFi library
#include <BlynkSimpleEsp32.h>    // Include the Blynk library for ESP32
#include <Stepper.h>             // Include the Stepper motor library

// IR Sensors and Buzzer
const int irSensor1 = 4;         // IR Sensor 1 connected to GPIO 4
const int irSensor2 = 13;        // IR Sensor 2 connected to GPIO 13
const int soundSensor = 34;      // KY-03 Sound Sensor analog pin connected to GPIO 34
const int buzzer = 15;           // Buzzer connected to GPIO 15

// Stepper Motor Settings
#define STEPS_PER_REV 2048       // Number of steps per revolution for the stepper motor

// Define fixed positions for the stepper motor
const int STEPS_TO_ZONE_1 = -STEPS_PER_REV / 8;  // 90 degrees counterclockwise
const int STEPS_TO_ZONE_2 = STEPS_PER_REV / 8;   // 90 degrees clockwise
const int STEPS_TO_DEFAULT = 0;  

Stepper stepperMotor(STEPS_PER_REV, 25, 26, 32, 33);  // IN1, IN2, IN3, IN4

// Variable to track the current position of the motor
int currentMotorPosition = STEPS_TO_DEFAULT;


// Relay Module
const int relayPin = 12;         // Relay connected to GPIO 12 (can use GPIO 14 or 35 instead)
bool relayState = false;         // To store the relay's ON/OFF state

// Threshold for sound intensity
const int soundThreshold = 500;  // Set a high threshold (0–4095 for ESP32 ADC)

// DHT11 Settings
#define DHTPIN 27       
#define DHTTYPE DHT11     
DHT dht(DHTPIN, DHTTYPE); 

// LCD Pins
LiquidCrystal lcd(23, 22, 21, 19, 18, 5);  // RS, E, D4, D5, D6, D7

// Wi-Fi Credentials
const char* ssid = "hamim";
const char* password = "hamim234";

// Blynk Auth Token
char auth[] = "uPieI1N4MOJIO_PxcSXsP6UeSSrRc3Zw";

// System State (1 = ON, 0 = OFF)
bool systemState = true;  // Initially ON

// Blynk virtual pin to toggle the system state
BLYNK_WRITE(V5) {
  systemState = param.asInt();  // Read the button state from the app
  if (systemState) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Armed");
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System OFF");
    noTone(buzzer);  // Turn off the buzzer
  }
}

// Blynk virtual pin to control the relay
BLYNK_WRITE(V7) {
  relayState = param.asInt();  // Read the relay state (ON/OFF) from the app
  digitalWrite(relayPin, relayState);
  if (relayState) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Relay ON");
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Relay OFF");
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(irSensor1, INPUT_PULLDOWN);
  pinMode(irSensor2, INPUT_PULLDOWN);
  pinMode(soundSensor, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(relayPin, OUTPUT);     // Set relay pin as output
  digitalWrite(relayPin, LOW);  // Ensure relay is initially OFF

  dht.begin();
  
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("System Armed");
  delay(2000);
  lcd.clear();

  WiFi.begin(ssid, password);
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connected");
  delay(2000);
  lcd.clear();

  // Start Blynk
  Blynk.begin(auth, ssid, password);

  stepperMotor.setSpeed(10);  // Set stepper motor speed (RPM)
}


void loop() {
  Blynk.run();  // Run Blynk process
  
  if (!Blynk.connected()) {
  Blynk.connect();  // Reconnect to the Blynk server
}

  if (!systemState) {
    // If the system is OFF, do nothing
    return;
  }

  int sensor1State = !digitalRead(irSensor1);
  int sensor2State = !digitalRead(irSensor2);
  int soundLevel = analogRead(soundSensor);  // Read sound intensity (0-4095)

  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
// Debug: Print sensor values to Serial Monitor
  Serial.println("----- Sensor Readings -----");
  Serial.print("IR Sensor 1 State: ");
  Serial.println(sensor1State);
  Serial.print("IR Sensor 2 State: ");
  Serial.println(sensor2State);
  Serial.print("Sound Level: ");
  Serial.println(soundLevel);
  Serial.print("Temperature (C): ");
  Serial.println(temp);
  Serial.print("Humidity (%): ");
  Serial.println(hum);
  Serial.println("---------------------------");

  int sensorState = (sensor1State << 1) | sensor2State;

    Blynk.virtualWrite(V1, temp);  // Send temperature
    Blynk.virtualWrite(V2, hum);   // Send humidity
    Blynk.virtualWrite(V3, sensor1State); // Send Zone 1 status
    Blynk.virtualWrite(V4, sensor2State); // Send Zone 2 status
    Blynk.virtualWrite(V6, soundLevel);       // Live sound level to V6

  if (soundLevel > soundThreshold) {
    // Trigger alarm for high sound level
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Loud Noise Det!");
    tone(buzzer, 1500);  // Activate buzzer
    delay(2000);         // Buzzer on for 2 seconds
    noTone(buzzer);
    return;              // Skip other checks during alert
  }

  // Handle motion detection
  if (sensor1State && !sensor2State) {
    // Motion detected in Zone 1
    if (currentMotorPosition != STEPS_TO_ZONE_1) {
      int stepsToMove = STEPS_TO_ZONE_1 - currentMotorPosition;
      stepperMotor.step(stepsToMove);
      currentMotorPosition = STEPS_TO_ZONE_1;
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Intruder Detected");
    lcd.setCursor(0, 1);
    lcd.print("Zone 1");
    tone(buzzer, 1000);  // Make the buzzer beep
  } else if (!sensor1State && sensor2State) {
    // Motion detected in Zone 2
    if (currentMotorPosition != STEPS_TO_ZONE_2) {
      int stepsToMove = STEPS_TO_ZONE_2 - currentMotorPosition;
      stepperMotor.step(stepsToMove);
      currentMotorPosition = STEPS_TO_ZONE_2;
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Intruder Detected");
    lcd.setCursor(0, 1);
    lcd.print("Zone 2");
    tone(buzzer, 1000);
  } else if (!sensor1State && !sensor2State) {
    // No motion detected
    if (currentMotorPosition != STEPS_TO_DEFAULT) {
      int stepsToMove = STEPS_TO_DEFAULT - currentMotorPosition;
      stepperMotor.step(stepsToMove);
      currentMotorPosition = STEPS_TO_DEFAULT;
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Armed");
    lcd.setCursor(0, 1);
    lcd.print("Sound OK");
    noTone(buzzer);  // Turn off the buzzer
  } else {
    // Motion detected in both zones
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Intruder Detected");
    lcd.setCursor(0, 1);
    lcd.print("Zones 1 & 2");
    tone(buzzer, 2000);  // Dual-tone beep
  }
  
  delay(1000);
}
