#include <SPI.h>
#include <RadioLib.h>
#include <vector>  // We'll use a std::vector for the message buffer

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
String deviceID = "6A";

// Heartbeat timer (5-second interval)
unsigned long lastHeartbeat = 0;
String heartBeatCode = "#<3";

// Flag for received packet
volatile bool receivedFlag = false;
String last_msg = "none";

// Buffer of messages awaiting transmission
std::vector<String> txQueue;

// ISR callback for when a LoRa packet arrives
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}

// 1. Read from USB Serial and enqueue (do NOT transmit immediately)
void checkSerialInput() {
  if (Serial.available() > 0) {
    // Read the message sent over Serial until newline
    String incoming = Serial.readStringUntil('\n');
    incoming.trim(); // remove trailing newline/spaces

    if (incoming.length() > 0) {
      // Add message to queue
      txQueue.push_back(incoming);

      Serial.println("  |_________________");
      Serial.print(">>| ");
      Serial.println(incoming);
    }
  }
}

// 2. Transmit one message (if any) from the queue
void processNextMessage() {
  // Only transmit if there's at least 1 message in queue
  if (txQueue.empty()) {
    return;
  }

  // Pull out the next message (FIFO)
  String message = txQueue.front();

  // Indicate LED on to show we are transmitting
  digitalWrite(LED_PIN, HIGH);

  // Stop listening to transmit
  int standbyState = radio.standby();
  if (standbyState != RADIOLIB_ERR_NONE) {
    Serial.print("Failed to switch to standby, code ");
    Serial.println(standbyState);
    // We won't remove the message from queue, so it can retry next time
    digitalWrite(LED_PIN, LOW);
    return;
  }

  // Prepare for transmit
  last_msg = message;
  const char* cstr = message.c_str();
  size_t length = message.length();

  unsigned long startTime = micros();
  int txState = radio.transmit((const uint8_t*)cstr, length, 0);
  unsigned long endTime = micros();

  unsigned long elapsedMicros = endTime - startTime;
  float elapsedMs = elapsedMicros / 1000.0;

  if (txState == RADIOLIB_ERR_NONE) {
    Serial.print(" L| Transmission completed in ");
    Serial.print(elapsedMs);
    Serial.println(" ms");
    // Remove from the queue since it was sent successfully
    txQueue.erase(txQueue.begin());
  } else {
    Serial.print("Transmission error: ");
    Serial.println(txState);
    // We won't remove from queue, so next iteration can retry
  }

  // Resume listening
  int rxState = radio.startReceive();
  if (rxState != RADIOLIB_ERR_NONE) {
    Serial.print("Failed to resume listening, code ");
    Serial.println(rxState);
  }

  digitalWrite(LED_PIN, LOW);
}

// 3. Send a heartbeat every 5 seconds, but only if there is no heartbeat in the queue
void sendHeartbeat() {
  if (millis() - lastHeartbeat >= 5000) {
    // We'll enqueue the heartbeat just like a normal message
    // But first check if there's already one in the queue
    bool heartbeatInQueue = false;
    String heartbeatMsg = heartBeatCode + deviceID;

    for (const String &msg : txQueue) {
      // if the existing msg starts with #<3 or matches
      // might just compare it directly with heartbeatMsg
      if (msg == heartbeatMsg) {
        heartbeatInQueue = true;
        break;
      }
    }

    // Only enqueue if not found
    if (!heartbeatInQueue) {
      txQueue.push_back(heartbeatMsg);
    }

    lastHeartbeat = millis();
  }
}

// 4. Check if a LoRa packet was received
void handleReceivedPacket() {
  if (!receivedFlag) {
    return;
  }
  receivedFlag = false;

  // Read received data
  String str;
  int state = radio.readData(str);

  // Re-enable radio listener
  int rxState = radio.startReceive();
  if (rxState != RADIOLIB_ERR_NONE) {
    Serial.println("  |_________________");
    Serial.print("ER| ");
    Serial.print("Failed to resume listening, code ");
    Serial.println(rxState);
  }

  if (state == RADIOLIB_ERR_NONE) {
    if (str == "") {
      // Skip processing empty
      Serial.println("  |_________________");
      Serial.print("DE| ");
      Serial.println("empty");
    }
    else if (str == last_msg) {
      // Loopback detection
      if (str != "none") {
        Serial.println("  |_________________");
        Serial.print("DE| ");
        Serial.println("loopback");
        last_msg = "none";
      }
    }
    else if (str.startsWith(heartBeatCode)) {
      // It's a heartbeat
      String senderID = str.substring(strlen(heartBeatCode.c_str()));
      if (senderID != deviceID) {
        Serial.println("  |_________________");
        Serial.print("<3| ");
        Serial.println(senderID); // heartbeat from another device
      } else {
        Serial.println("  |_________________");
        Serial.print("DE| ");
        Serial.println(senderID); // own heartbeat
      }
    } else {
      // Normal (non-heartbeat) message
      Serial.println("  |_________________");
      Serial.print("<<| ");
      Serial.println(str);
    }
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("  |_________________");
    Serial.print("ER| ");
    Serial.println("CRC error!");
  } else {
    Serial.println("  |_________________");
    Serial.print("ER| Read error, code ");
    Serial.println(state);
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

  // Initialize SX1262 (change frequency if needed)
  Serial.print("[SX1262] Initializing ... ");
  int state = radio.begin(868.0, 125.0, 12, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 17, 14, 0);
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
  // 1. Enqueue any Serial input
  checkSerialInput();

  // 2. Possibly enqueue a heartbeat every 5 seconds
  sendHeartbeat();

  // 3. Process exactly one message from the queue (if available)
  processNextMessage();

  // 4. Check if a LoRa packet was received
  handleReceivedPacket();
}
