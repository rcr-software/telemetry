// TELEMETRY_DATA_RX
// -*- mode: C++ -*-
// RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
 
#include <RH_RF95.h>
//    #include "Adafruit_BLE.h"
//    #include "Adafruit_BluefruitLE_SPI.h"
//    #include "Adafruit_BluefruitLE_UART.h"
//    #include "BluefruitConfig.h"

//for feather m0  
//    #define RFM95_CS 8
//    #define RFM95_RST 4
//    #define RFM95_INT 3

//for radio featherWing
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"

// Utilizing Frequency of 862 MHz for data collection
#define RF95_FREQ 862.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//    Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

#define LED 13
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);




 
void setup()  {
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  Serial.begin(115200);
  
  while(!Serial)  {
    delay(100);
  }
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(100);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
 
  if (!rf95.init() || !rf95.setFrequency(RF95_FREQ)) {
    Serial.println("RX init failed");
    while(1);
  }
  
  Serial.println("RX OK: 862MHz");
  delay(1000);

  rf95.setTxPower(23, false);

  systemStart(); 
} // END SETUP





 
void loop()  {
  if (rf95.available())   {
    if (rf95.recv(buf, &len))   {
      digitalWrite(LED, HIGH);
      Serial.print((char*)buf);
      Serial.println(rf95.lastRssi(), DEC);
      digitalWrite(LED, LOW);
    }
  }
}






void systemStart()  {
  String dataString;
  char radioPacket[] = "BEGIN STARTUP";
  unsigned long timeNow, timeLast;
  
  // waits for input from user to begin initialization
  Serial.println("Type \"START\" when ready!");
  while(dataString != "START") {
    if (Serial.available() > 0) {
      dataString = Serial.readString();
    }
  }
  Serial.println("Waiting for signal...");

  // Sends start signals to TX until we get a reply
  timeLast = millis();
  while (!rf95.available())  {
    timeNow = millis();
    if ((timeNow - timeLast) >= 1000) {
      rf95.send((uint8_t*)radioPacket, sizeof(radioPacket));
      timeLast = timeNow;
    }
  }

  // waits until all TX systems have been verified
  while (dataString != "Ready to Start")  {
    if (rf95.available())   {
      if (rf95.recv(buf, &len))   {
        digitalWrite(LED, HIGH);
        dataString = String((char*)buf);
        Serial.print(dataString);
        Serial.print(",");
        Serial.println(rf95.lastRssi(), DEC);
        digitalWrite(LED, LOW);
      }
    }
  }

  // waits for input from user to start system
  Serial.println("Type \"GO FOR LAUNCH\" when ready!");
  while(dataString != "GO FOR LAUNCH") {
    if (Serial.available() > 0) {
      dataString = Serial.readString();
    }
  }
  Serial.println("Sending Start Signal...");
  
  // Sends start signals to TX until we get a reply
  timeLast = millis();
  strncpy(radioPacket, "GO FOR LAUNCH", sizeof(radioPacket));
  while (!rf95.available())  {
    timeNow = millis();
    if ((timeNow - timeLast) >= 1000) {
      rf95.send((uint8_t*)radioPacket, sizeof(radioPacket));
      timeLast = timeNow;
    }
  }
  Serial.println("SYSTEM STARTING");
}
