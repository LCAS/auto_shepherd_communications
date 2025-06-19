#include <SPI.h>
#include <RadioLib.h>
#include <ArduinoJson.h>  

#define LORA_SCK     14
#define LORA_MISO    24
#define LORA_MOSI    15
#define LORA_SS      13
#define LORA_RST     23
#define LORA_DIO1    16
#define LORA_BUSY    18

#define LED_PIN      25  // Change to any digital pin if needed
#define TIMEOUT_MS   5000  // 20 seconds

SX1262 radio = new Module(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI1);

float throttleValue = 0.0;
float steeringValue = 0.0;
volatile bool operationDone = false;
bool transmitFlag = false;
unsigned long lastReceiveTime = 0;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag() {
  operationDone = true;
}
void processTwistMessage(const char *payload);



void setup() {
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // LED off initially

  SPI1.setRX(LORA_MISO);
  SPI1.setTX(LORA_MOSI);
  SPI1.setSCK(LORA_SCK);
  SPI1.begin(false);

  int state = radio.begin(868.0, 125.0, 6, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 17, 14, 0);
  if (state != RADIOLIB_ERR_NONE) while (true);

  radio.setDio1Action(setFlag);
  radio.startReceive();

  lastReceiveTime = millis();  // initialize timer
}

void loop() {
  if (operationDone) {
    operationDone = false;

    if (transmitFlag) {
      radio.startReceive();
      transmitFlag = false;
    } else {
      String msg;
      





      if (radio.readData(msg) == RADIOLIB_ERR_NONE) {
        Serial.println("Received: " + msg);
        lastReceiveTime = millis();  // reset timeout timer
        digitalWrite(LED_PIN, LOW);  // signal received, LED off
      }
      processTwistMessage(msg.c_str());
      radio.startTransmit("PONG");
      transmitFlag = true;
    }
  }

  // Check for 20-second timeout
  if (millis() - lastReceiveTime > TIMEOUT_MS) {
    digitalWrite(LED_PIN, HIGH);  // turn on LED
  }
}


// linear and angular velocities
void processTwistMessage(const char *payload) {
  StaticJsonDocument<200> doc;
  Serial.print("Raw payload: ");
  Serial.println(payload);

  DeserializationError error = deserializeJson(doc, payload);

  if (!error) {
    float linear_x = doc["linear"]["x"];
    float linear_y = doc["linear"]["y"];
    float linear_z = doc["linear"]["z"];
    float angular_x = doc["angular"]["x"];
    float angular_y = doc["angular"]["y"];
    float angular_z = doc["angular"]["z"];

    //  x component of linear and z component of angular velocity
    throttleValue = linear_x;  // Linear velocity (x-axis)
    steeringValue = angular_z; // Angular velocity (z-axis)

    // Print out the velocities
    Serial.print("Throttle (linear x): ");
    Serial.println(throttleValue);
    Serial.print("Steering (angular z): ");
    Serial.println(steeringValue);
  } else {
    Serial.println("Error parsing JSON");
  }
}
