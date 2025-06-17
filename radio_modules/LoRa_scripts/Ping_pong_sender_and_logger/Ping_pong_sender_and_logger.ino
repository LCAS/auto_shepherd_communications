// ============ GATEWAY ============

#include <SPI.h>
#include <RadioLib.h>

#define LORA_SCK     14
#define LORA_MISO    24
#define LORA_MOSI    15
#define LORA_SS      13
#define LORA_RST     23
#define LORA_DIO1    16
#define LORA_BUSY    18
#define LORA_ANT_SW  17

SX1262 radio = new Module(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI1);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  SPI1.setRX(LORA_MISO);
  SPI1.setTX(LORA_MOSI);
  SPI1.setSCK(LORA_SCK);
  SPI1.begin(false);

  int state = radio.begin(868.0, 125.0, 9, 5,
                          RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 17, 14, 0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println("LoRa init failed");
    while (true);
  }

  Serial.println("LoRa Gateway ready");
}

void loop() {
  delay(1000);
  Serial.println("Sending: PING");
 
  int state = radio.transmit("PING");
  unsigned long startTime = millis();
  delay(500);
  String reply;
  state = radio.receive(reply);

  if(reply == "PONG") {
    Serial.println(reply + " recieved");
    unsigned long endTime = millis();
  
    unsigned long elapsedTime = (endTime - startTime);
    Serial.println("elapsed time: " + String(elapsedTime));
  }
  if (state == RADIOLIB_ERR_NONE) {
      

      Serial.println(" RSSI: " + String(radio.getRSSI()) + " dBm | SNR: " + String(radio.getSNR()) + " dB | Freq Error: " + String(radio.getFrequencyError()) + " Hz");



    }
  
}
