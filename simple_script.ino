#include <SPI.h>
#include <RadioLib.h>

// SPI definition
#define LED_PIN      25   // Onboard LED at GP25
#define LORA_SCK     14   // GP10
#define LORA_MISO    24   // GP8
#define LORA_MOSI    15   // GP11
#define LORA_SS      13   // GP9
#define LORA_RST     23   // GP23
#define LORA_DIO1    16   // GP16
#define LORA_BUSY    18   // GP18
#define LORA_ANT_SW  17   // GP17

// Create RadioLib SX1262 instance
SX1262 radio = new Module(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI1);

// Hard-coded device ID
String deviceID = "myDevice001";

// Heartbeat timer (5-second interval)
unsigned long lastHeartbeat = 0;
String heartBeatCode = "#<3";

// Flag for received packet
volatile bool receivedFlag = false;
String last_msg = "none";

// ISR callback for when a LoRa packet arrives
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}

// 1. Check if data arrived from USB Serial and transmit
void checkSerialInput() {
  if (Serial.available() > 0) {
    // Read the message sent over Serial until newline
    String message = Serial.readStringUntil('\n');
    message.trim(); // remove any trailing newline or spaces

    if (message.length() > 0) {
      digitalWrite(LED_PIN, HIGH);
      Serial.print(">>| ");
      Serial.println(message);

      // We have to stop listening to transmit
      int standbyState = radio.standby();
      if (standbyState != RADIOLIB_ERR_NONE) {
        Serial.print("Failed to switch to standby, code ");
        Serial.println(standbyState);
      } else {
        // Transmit the message
        last_msg = message;
        int txState = radio.transmit(message);

        
        
// src/modules/LR11x0/LR11x0.cpp:179:int16_t LR11x0::transmit(const uint8_t* data, size_t len, uint8_t addr) {



        
        if (txState == RADIOLIB_ERR_NONE) {
          // Serial.println("Transmission ok.");
        } else {
          Serial.print("Transmission error: ");
          Serial.println(txState);
        }
        // Resume listening
        int rxState = radio.startReceive();
        if (rxState != RADIOLIB_ERR_NONE) {
          Serial.print("Failed to resume listening, code ");
          Serial.println(rxState);
        }
      }
      digitalWrite(LED_PIN, LOW);
    }
  }
}

// 2. Send a heartbeat every 5 seconds in the format heartBeatCode + deviceID
void sendHeartbeat() {
  if (millis() - lastHeartbeat >= 5000) {
    digitalWrite(LED_PIN, HIGH);
    // Serial.println("Sending heartbeat...");

    // Pause listening
    int standbyState = radio.standby();
    if (standbyState != RADIOLIB_ERR_NONE) {
      Serial.print("Failed to switch to standby, code ");
      Serial.println(standbyState);
    } else {
      // Construct the heartbeat message
      String heartbeatMsg = heartBeatCode + deviceID;
      // Transmit the heartbeat
      int txState = radio.transmit(heartbeatMsg);
      if (txState == RADIOLIB_ERR_NONE) {
        // Print only sender_id + "   <3"
        //Serial.println(">>>>>>>>>>>>>>");
        Serial.print(">3| ");
        Serial.println(heartbeatMsg);
      } else {
        Serial.print("Heartbeat error: ");
        Serial.println(txState);
      }

      // Resume listening
      int rxState = radio.startReceive();
      if (rxState != RADIOLIB_ERR_NONE) {
        Serial.print("ER| ");
        Serial.print("Failed to resume listening, code ");
        Serial.println(rxState);
      }
    }

    digitalWrite(LED_PIN, LOW);
    lastHeartbeat = millis();
  }
}

// 3. Check if a LoRa packet was received
void handleReceivedPacket() {
  if (receivedFlag) {
    // reset the flag
    receivedFlag = false;
    Serial.println("  |_________________");

    // Read received data
    String str = "";
    Serial.print("  | Before: ");
    Serial.println(str);
    int state = radio.readData(str);
    Serial.print("  | After: ");
    Serial.println(str);
    
    // Reanable radio listener
    int rxState = radio.startReceive();
    if (rxState != RADIOLIB_ERR_NONE) {
      Serial.print("ER| ");
      Serial.print("Failed to resume listening, code ");
      Serial.println(rxState);
    }

    Serial.print("In| ");
    Serial.println(str);


    if (state == RADIOLIB_ERR_NONE) {
      // If message is empty, skip processing
      if (str == "") { 
        Serial.print("--| ");
        Serial.println("empty");
      }
      // Check if this is the message we just sent
      else if (str == last_msg){
        if (str != "none") {
          Serial.print("--| ");
          Serial.println("loopback");
          last_msg = "none";
        }
      }
      // Check if this is a heartbeat packet
      else if (str.startsWith(heartBeatCode)) {
        // parse the sender ID (whatever follows heartBeatCode)
        String senderID = str.substring(strlen(heartBeatCode.c_str()));
        if (senderID != deviceID) {
          Serial.print("<3| ");
          Serial.println(senderID); //heartbeat
        }
      }
      else {
        Serial.print("<<| ");
        Serial.println(str); //regular message
      }

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.print("ER| ");
      Serial.println("CRC error!"); //malformed

    } else {
      Serial.print("ER| ");
      Serial.print("Read error, code "); //err
      Serial.println(state);
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(9600);
  while (!Serial);

  // SPI configuration
  SPI1.setRX(LORA_MISO);
  SPI1.setTX(LORA_MOSI);
  SPI1.setSCK(LORA_SCK);
  SPI1.begin(false);

  // Initialize SX1262, Change 433.0 to a supported frequency for your module
  Serial.print("[SX1262] Initializing ... ");
  int state = radio.begin(433.0, 125.0, 12, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 17, 14, 0);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("success!");
  } else {
    Serial.print("failed, code ");
    Serial.println(state);
    while (true) {
      delay(10);
    }
  }

  // Set the function that will be called when a new packet is received
  radio.setPacketReceivedAction(setFlag);

  // Start listening for LoRa packets
  Serial.print("[SX1262] Starting to listen ... ");
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("success!");
  } else {
    Serial.print("failed, code ");
    Serial.println(state);
    while (true) {
      delay(10);
    }
  }

  // Initialize heartbeat timer
  lastHeartbeat = millis();
  digitalWrite(LED_PIN, LOW);

  Serial.print("Device Name: ");
  Serial.println(deviceID);
}

void loop() {
  // 1. Check if data arrived from USB Serial
  checkSerialInput();
  // 2. Send a heartbeat every 5 seconds
  sendHeartbeat();
  // 3. Check if a LoRa packet was received
  handleReceivedPacket();
}
