// TELEMETRY_DATA_RX
// -*- mode: C++ -*-
// RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
 
#include <RH_RF95.h>

#define RFM95_RST     11   // "A"
// for radio featherWing #1 - 862 MHz
#define RFM95_862_CS      10   // "B"
#define RFM95_862_INT     6    // "D"
// for radio featherWing #2 - 915 MHz
#define RFM95_915_CS      10   // "B"
#define RFM95_915_INT     6    // "D"

// Frequencies for data collection
#define RF95_862_FREQ 862.0
#define RF95_915_FREQ 915.0
 
// Singleton instance of the radio driver
RH_RF95 rf95_862(RFM95_862_CS, RFM95_862_INT);
RH_RF95 rf95_915(RFM95_915_CS, RFM95_915_INT);

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
const int packetSize = 100;
char radioPacket[packetSize];



 
void setup()  {
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
 
  if (!rf95_862.init() || !rf95_862.setFrequency(RF95_862_FREQ)) {
    Serial.println("RX #1 (862 MHz) init failed");
    while(1);
  }
  Serial.println("RX #1 OK: 862 MHz");
  delay(1000);
  
  if (!rf95_915.init() || !rf95_915.setFrequency(RF95_915_FREQ)) {
    Serial.println("RX #1 (915 MHz) init failed");
    while(1);
  }
  Serial.println("RX #2 OK: 915 MHz");
  delay(1000);

  rf95_862.setTxPower(23, false);
  rf95_915.setTxPower(23, false);

  systemStart(); 
} // END SETUP





 
void loop()  {
  if (rf95_862.available())   {
    if (rf95_862.recv(buf, &len))   {
      sprintf(radioPacket, "%s,%d", (char*)buf,rf95_862.lastRssi());
      Serial.println(radioPacket);
    }
  }
  if (rf95_915.available())   {
    if (rf95_915.recv(buf, &len))   {
      sprintf(radioPacket, "%s,%d", (char*)buf,rf95_862.lastRssi());
      Serial.println(radioPacket);
    }
  }
}






void systemStart()  {
  String dataString;
  unsigned long timeNow, timeLast = 0;


  // waits for input from user to begin initialization
  Serial.println("Type \"START\" when ready!");
  while(dataString != "START") {
    if (Serial.available() > 0) {
      dataString = Serial.readString();
    }
  }
  Serial.println("Waiting for signal...");

  // ------------------ INITIALIZE TX_862 ----------------------
  // Sends start signals to TX_862 until we get a reply
  while (!rf95_862.available())  {
    timeNow = millis();
    if ((timeNow - timeLast) >= 1000) {
      sendMsg("BEGIN STARTUP TX_862",862);
      timeLast = timeNow;
    }
  }
  // waits until all TX_862 systems have been verified
  while (dataString != "Ready to Start")  {
    if (rf95_862.available())   {
      if (rf95_862.recv(buf, &len))   {
        sprintf(radioPacket, "%s,%d", (char*)buf,rf95_862.lastRssi());
        Serial.println(radioPacket);
      }
    }
  }

  // ------------------ INITIALIZE TX_915 ----------------------
  // Sends start signals to TX_915 until we get a reply
  while (!rf95_915.available())  {
    timeNow = millis();
    if ((timeNow - timeLast) >= 1000) {
      sendMsg("BEGIN STARTUP TX_915",915);
      timeLast = timeNow;
    }
  }
  // waits until all TX_915 systems have been verified
  while (dataString != "Ready to Start")  {
    if (rf95_915.available())   {
      if (rf95_915.recv(buf, &len))   {
        sprintf(radioPacket, "%s,%d", (char*)buf,rf95_862.lastRssi());
        Serial.println(radioPacket);
      }
    }
  }


  // Wait for user input to start systems
  Serial.println("Type \"GO FOR LAUNCH\" when ready!");
  while(dataString != "GO FOR LAUNCH") {
    if (Serial.available() > 0) {
      dataString = Serial.readString();
    }
  }
  Serial.println("Sending Start Signal...");
  
  // ------------------ START TX_862 ----------------------
  // Sends start signals to TX_862 until we get a reply
  while (!rf95_862.available())  {
    timeNow = millis();
    if ((timeNow - timeLast) >= 1000) {
      sendMsg("TX_862 GO FOR LAUNCH",862);
      timeLast = timeNow;
    }
  }
  
  // ------------------ START TX_915 ----------------------
  // Sends start signals to TX_915 until we get a reply
  while (!rf95_915.available())  {
    timeNow = millis();
    if ((timeNow - timeLast) >= 1000) {
      sendMsg("TX_915 GO FOR LAUNCH",915);
      timeLast = timeNow;
    }
  }
  
  Serial.println("SYSTEM STARTING");
}





// _______________________________________________________________________________________
//                        *** RADIO STRING MESSAGE FUNCTION ***
// _______________________________________________________________________________________
void sendMsg(String dataString, int whichRX) {
    strncpy(radioPacket, dataString.c_str(), sizeof(radioPacket));
    if (whichRX == 862) {
      rf95_862.send((uint8_t *)radioPacket, packetSize);
    }
    else  {
      rf95_915.send((uint8_t *)radioPacket, packetSize);
    }
}
