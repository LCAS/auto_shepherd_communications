#include <SPI.h>
#include <RadioLib.h>

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

volatile bool operationDone = false;
bool transmitFlag = false;
unsigned long lastReceiveTime = 0;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag() {
  operationDone = true;
}

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

      radio.startTransmit("PONG");
      transmitFlag = true;
    }
  }

  // Check for 20-second timeout
  if (millis() - lastReceiveTime > TIMEOUT_MS) {
    digitalWrite(LED_PIN, HIGH);  // turn on LED
  }
}
