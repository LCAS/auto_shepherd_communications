#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define SERIAL Serial

// IMPORTANT: Set different device IDs for each ESP32
#define DEVICE_ID 1  // Change this to 2 for the second ESP32
// 1: breakout board one
// 2: loose one

// MAC addresses of your two ESP32 devices
// EC:E3:34:46:4F:CC
// 68:25:DD:EF:8C:B8

// Custom MAC addresses you want to use
#if DEVICE_ID == 1
  uint8_t customMAC[] = {0x68, 0x25, 0xDD, 0xEF, 0x8C, 0xB8};  // Device 1 MAC
  uint8_t targetMAC[] = {0xEC, 0xE3, 0x34, 0x46, 0x4F, 0xCC};  // Device 2 MAC
#else
  uint8_t customMAC[] = {0xEC, 0xE3, 0x34, 0x46, 0x4F, 0xCC};  // Device 2 MAC  
  uint8_t targetMAC[] = {0x68, 0x25, 0xDD, 0xEF, 0x8C, 0xB8};  // Device 1 MAC
#endif
esp_now_peer_info_t peerInfo;

// Message structure
struct MessageData {
  char message[50];
  int deviceId;
  int messageCount;
  unsigned long timestamp;
  bool isResponse;
};

MessageData sendData;
MessageData recvData;
MessageData oldReceived;

// Timing variables
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 3000; // Send every 3 seconds
int messageCounter = 0;

void setup() {
  SERIAL.begin(115200);
  delay(1000);
  
  SERIAL.println("ESP32 ESP-NOW Bidirectional Test");
  SERIAL.print("Device ID: ");
  SERIAL.println(DEVICE_ID);
  
  initControllerConnection();
  
  // Initialize send data
  sendData.deviceId = DEVICE_ID;
  sendData.messageCount = 0;
  sendData.isResponse = false;
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check if there's data available from USB serial
  if (SERIAL.available() > 0) {
    String serialInput = SERIAL.readStringUntil('\n');
    serialInput.trim(); // Remove any trailing whitespace/newlines
    
    if (serialInput.length() > 0) {
      // Prepare message data
      messageCounter++;
      sendData.messageCount = messageCounter;
      sendData.timestamp = currentTime;
      sendData.isResponse = false;
      
      // Copy the serial input to the message (limit to message array size)
      strncpy(sendData.message, serialInput.c_str(), sizeof(sendData.message) - 1);
      sendData.message[sizeof(sendData.message) - 1] = '\0'; // Ensure null termination
      
      SERIAL.print("[SERIAL->ESP-NOW] Sending: ");
      SERIAL.println(sendData.message);
      
      // Send via ESP-NOW
      esp_err_t result = esp_now_send(targetMAC, (uint8_t *) &sendData, sizeof(sendData));
      if (result != ESP_OK) {
        SERIAL.println("[ERROR] Failed to send serial data via ESP-NOW");
      }
    }
  }
  
  // Optional: Keep the periodic test messages from Device 1 (comment out if not needed)
  /*
  if (currentTime - lastSendTime >= sendInterval) {
    #if DEVICE_ID == 1
    sendTestMessage();
    #endif
    lastSendTime = currentTime;
  }
  */
  
  delay(10); // Small delay to prevent overwhelming the system
}

void initControllerConnection() {
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(1000);
  SERIAL.print("My MAC Address: ");
  SERIAL.println(WiFi.macAddress());

  // Set custom MAC address (temporary override)
  delay(500);
  esp_err_t ret = esp_wifi_set_mac(WIFI_IF_STA, customMAC);
  if (ret == ESP_OK) {
    SERIAL.println("Custom MAC set successfully");
  } else {
    SERIAL.println("Failed to set custom MAC");
    return;
  }
  
  delay(500);
  SERIAL.print("New MAC Address: ");
  SERIAL.println(WiFi.macAddress());

  delay(1000);
  SERIAL.print("Targeting MAC: ");
  for(int i = 0; i < 6; i++) {
    SERIAL.printf("%02X", targetMAC[i]);
    if(i < 5) SERIAL.print(":");
  }
  SERIAL.println();
  SERIAL.println();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    SERIAL.println("Error initializing ESP-NOW");
    return;
  }
  SERIAL.println("ESP-NOW initialized");

  // Register send callback
  esp_now_register_send_cb(OnDataSent);
  SERIAL.println("Send callback registered");

  // Register receive callback first
  esp_now_register_recv_cb(OnDataRecv);
  SERIAL.println("Receive callback registered");

  // Set up peer info
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, targetMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  
  // Add peer        
  esp_err_t addStatus = esp_now_add_peer(&peerInfo);
  if (addStatus != ESP_OK) {
    SERIAL.print("Failed to add peer: ");
    SERIAL.println(addStatus);
    return;
  }
  SERIAL.println("Peer added successfully");
  
  SERIAL.println("ESP-NOW setup complete - ready to communicate!");
  SERIAL.println("===================================");
}

void sendTestMessage() {
  messageCounter++;
  sendData.messageCount = messageCounter;
  sendData.timestamp = millis();
  sendData.isResponse = false;
  
  sprintf(sendData.message, "Hello from Device %d - Message #%d", DEVICE_ID, messageCounter);
  
  SERIAL.print("[SEND] ");
  SERIAL.println(sendData.message);
  
  esp_err_t result = esp_now_send(targetMAC, (uint8_t *) &sendData, sizeof(sendData));
  if (result != ESP_OK) {
    SERIAL.println("Error sending the data");
  }
}

void sendResponse(int originalDeviceId, int originalMessageCount) {
  sendData.messageCount = originalMessageCount;
  sendData.timestamp = millis();
  sendData.isResponse = true;
  
  sprintf(sendData.message, "ACK from Device %d to Device %d - Message #%d", 
          DEVICE_ID, originalDeviceId, originalMessageCount);
  
  SERIAL.print("[RESPONSE] ");
  SERIAL.println(sendData.message);
  
  esp_err_t result = esp_now_send(targetMAC, (uint8_t *) &sendData, sizeof(sendData));
  if (result != ESP_OK) {
    SERIAL.println("Error sending response");
  }
}

// Callback when data is received
void OnDataRecv(const esp_now_recv_info* recv_info, const uint8_t* incomingData, int len) {
  oldReceived = recvData;
  memcpy(&recvData, incomingData, sizeof(recvData));
  
  // Don't process our own messages
  if (recvData.deviceId == DEVICE_ID) {
    return;
  }
  
  SERIAL.print("[RECV] ");
  SERIAL.print(recvData.message);
  SERIAL.print(" (from Device ");
  SERIAL.print(recvData.deviceId);
  SERIAL.print(", ");
  SERIAL.print(len);
  SERIAL.println(" bytes)");
  
  // Print MAC address of sender
  SERIAL.print("From MAC: ");
  for(int i = 0; i < 6; i++) {
    SERIAL.printf("%02X", recv_info->src_addr[i]);
    if(i < 5) SERIAL.print(":");
  }
  SERIAL.println();
  
  // Send response if this wasn't already a response
  if (!recvData.isResponse) {
    delay(200); // Small delay before responding
    sendResponse(recvData.deviceId, recvData.messageCount);
  }
  
  SERIAL.println("---");
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  SERIAL.print("Send Status: ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    SERIAL.println("Success");
  } else {
    SERIAL.println("Failed");
  }
}