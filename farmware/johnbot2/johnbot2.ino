#include <WiFi.h>
#include <ArduinoOSCWiFi.h>

// -----------------------------------------------------------------------------
// Wi-Fi Configuration
// -----------------------------------------------------------------------------
const char* ssid = "acut404";
const char* password = "acut404pass";

// Robot network settings (static IP)
const IPAddress ip(192, 168, 50, 61);
const IPAddress gateway(192, 168, 50, 1);
const IPAddress subnet(255, 255, 255, 0);

// -----------------------------------------------------------------------------
// OSC Configuration
// -----------------------------------------------------------------------------
const char* hostIP = "192.168.50.126";   // PC/controller IP
const int outPort = 60011;              // Send sensor values → PC
const int inPort  = 61011;              // Receive motor/LED commands ← PC

// -----------------------------------------------------------------------------
// Pin Assignments
// -----------------------------------------------------------------------------
const int sensorPinR = A0;
const int sensorPinL = A1;

const int motorPinL = D2;   // PWM capable
const int motorPinR = D3;   // PWM capable

const int ledPinR = D8;
const int ledPinG = D9;
const int ledPinB = D10;

// -----------------------------------------------------------------------------
// Timers
// -----------------------------------------------------------------------------
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100;  // Send sensor values every 100 ms

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(sensorPinR, INPUT);
  pinMode(sensorPinL, INPUT);
  pinMode(motorPinL, OUTPUT);
  pinMode(motorPinR, OUTPUT);
  pinMode(ledPinR, OUTPUT);
  pinMode(ledPinG, OUTPUT);
  pinMode(ledPinB, OUTPUT);

  Serial.println("Starting Wi-Fi connection...");
  WiFi.begin(ssid, password);

  // Apply static IP settings
  if (!WiFi.config(ip, gateway, subnet)) {
    Serial.println("Failed to configure static IP!");
  }

  // Wi-Fi connection attempts
  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED && retryCount < 10) {
    delay(500);
    Serial.print(".");
    retryCount++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi connected successfully.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWi-Fi connection failed. Halting.");
    while (true) { delay(1000); }
  }

  // ---------------------------------------------------------------------------
  // OSC Subscriptions (motor + LED)
  // ---------------------------------------------------------------------------
  OscWiFi.subscribe(inPort, "/motor", [](const OscMessage& msg) {
    Serial.println("Received OSC message: /motor");

    if (msg.size() >= 2) {
      int motorValueL = msg.arg<int>(0);
      int motorValueR = msg.arg<int>(1);

      Serial.printf("Left motor: %d\n", motorValueL);
      Serial.printf("Right motor: %d\n", motorValueR);

      motorValueL = constrain(motorValueL, 0, 255);
      motorValueR = constrain(motorValueR, 0, 255);

      analogWrite(motorPinL, motorValueL);
      analogWrite(motorPinR, motorValueR);
    } else {
      Serial.println("Invalid /motor message: not enough arguments.");
    }
  });

  OscWiFi.subscribe(inPort, "/LED", [](const OscMessage& msg) {
    Serial.println("Received OSC message: /LED");

    if (msg.size() >= 3) {
      int LEDR = msg.arg<int>(0);
      int LEDG = msg.arg<int>(1);
      int LEDB = msg.arg<int>(2);

      Serial.printf("LED R: %d, G: %d, B: %d\n", LEDR, LEDG, LEDB);

      LEDR = constrain(LEDR, 0, 255);
      LEDG = constrain(LEDG, 0, 255);
      LEDB = constrain(LEDB, 0, 255);

      analogWrite(ledPinR, LEDR);
      analogWrite(ledPinG, LEDG);
      analogWrite(ledPinB, LEDB);

    } else {
      Serial.println("Invalid /LED message: not enough arguments.");
    }
  });
}

// -----------------------------------------------------------------------------
// Main Loop
// -----------------------------------------------------------------------------
void loop() {
  // Read sensors (0–1023) and normalize to 0–255
  int sensorValueR = map(analogRead(sensorPinR), 0, 1023, 0, 255);
  int sensorValueL = map(analogRead(sensorPinL), 0, 1023, 0, 255);

  // Send sensor values periodically (non-blocking)
  if (millis() - lastSendTime >= sendInterval) {
    OscWiFi.send(hostIP, outPort, "/sensor", sensorValueR, sensorValueL);
    lastSendTime = millis();
  }

  // Process incoming OSC messages
  OscWiFi.update();

  // Auto-reconnect Wi-Fi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected. Reconnecting...");
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWi-Fi reconnected.");
  }
}