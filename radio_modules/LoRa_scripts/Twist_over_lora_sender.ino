#include <SPI.h>
#include <RadioLib.h>

#define INITIATING_NODE
#define LORA_SCK     14
#define LORA_MISO    24
#define LORA_MOSI    15
#define LORA_SS      13
#define LORA_RST     23
#define LORA_DIO1    16
#define LORA_BUSY    18
#define LORA_ANT_SW  17

#define LED_PIN 25  // Adjust to 18 for Tiny2040

SX1262 radio = new Module(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI1);

volatile bool operationDone = false;
bool transmitFlag = false;
int transmissionState = RADIOLIB_ERR_NONE;

unsigned long sendTime = 0;
unsigned long receiveTime = 0;
unsigned long lastResponseTime = 0;

const unsigned long packetTimeout = 2000;     // Max wait for response
const unsigned long noSignalTimeout = 20000;  // LED on if no signal for 20s

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag() {
  operationDone = true;
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  SPI1.setRX(LORA_MISO);
  SPI1.setTX(LORA_MOSI);
  SPI1.setSCK(LORA_SCK);
  SPI1.begin(false);

  int state = radio.begin(868.0, 125.0, 6, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 17, 14, 0);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Failed to start radio, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  radio.setDio1Action(setFlag);
  Serial.println(F("[SX1262] Sending first packet ..."));
  transmissionState = radio.startTransmit("{\"linear\":{\"x\":0.5,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":1.2}}");
  
  transmitFlag = true;




  sendTime = millis();
  lastResponseTime = millis();
}

void loop() {
  unsigned long now = millis();

  // Check for timeout while waiting for response
  if (!operationDone && !transmitFlag && now - sendTime > packetTimeout) {
    Serial.println(F("[SX1262] Timeout: no response, retrying..."));
    transmissionState = radio.startTransmit("{\"linear\":{\"x\":0.5,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":1.2}}");
    
    transmitFlag = true;
    sendTime = now;

  }

  // Check if no packet received in last 20s
  if (now - lastResponseTime > noSignalTimeout) {
    digitalWrite(LED_PIN, HIGH);  // No signal, LED on
  } else {
    digitalWrite(LED_PIN, LOW);   // Signal OK
  }

  if (operationDone) {
    operationDone = false;

    if (transmitFlag) {
      if (transmissionState == RADIOLIB_ERR_NONE) {
        Serial.println(F("Transmission finished."));
                sendTime = millis();
        // 
        float bandwidth = 125.0;         // kHz
        int spreadingFactor = 6;         // as set in radio.begin()
        int codingRate = 5;              // CR = 4/5 → use 5 in formula

        // Effective data rate in kbps (approximate LoRa calculation)
        float dataRate = (spreadingFactor * (4.0 / (4 + codingRate))) /
                         (pow(2, spreadingFactor) / bandwidth);

        Serial.print("Estimated Data Rate: ");
        Serial.print(dataRate, 4);
        Serial.println(" kbps");
        Serial.print((dataRate * 1000) / 8.0, 1);
        Serial.println(F(" bytes/sec)"));


      } else {
        Serial.print(F("Transmission failed, code "));
        Serial.println(transmissionState);
      }

      radio.startReceive();
      transmitFlag = false;
    } else {
      String str;
      int state = radio.readData(str);

      if (state == RADIOLIB_ERR_NONE) {
        receiveTime = millis();
        lastResponseTime = receiveTime;
            // print measured data rate

        Serial.println(F("[SX1262] Received packet!"));
        Serial.println("Elapsed: " + String(receiveTime - sendTime) + " ms | RSSI: " + String(radio.getRSSI()) + " dBm | SNR: " + String(radio.getSNR()) + " dB");

        delay(1000);
        Serial.println(F("[SX1262] Sending next packet ..."));
        transmissionState = radio.startTransmit("{\"linear\":{\"x\":0.5,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":1.2}}");

        
        transmitFlag = true;
           

        sendTime = millis();
      }
    }
  }
}

