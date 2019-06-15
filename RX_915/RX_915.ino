// TELEMETRY_DATA_RX
// -*- mode: C++ -*-
// RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
 
#include <RH_RF95.h>

// radio featherWing - 915 MHz
#define RFM95_RST         11   // "A"
#define RFM95_915_CS      10   // "B"
#define RFM95_915_INT     6    // "D"
#define RF95_915_FREQ     915.0
RH_RF95 rf95_915(RFM95_915_CS, RFM95_915_INT);

#define startPB   12
#define do_sendSignal true              // Sends start signals to TX
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
const int packetSize = 100;
char radioPacket[packetSize];



 
void setup()  {
  pinMode(startPB, INPUT_PULLUP);
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

  if (!rf95_915.init() || !rf95_915.setFrequency(RF95_915_FREQ)) {
    Serial.println("RX_915 init failed");
    while(1);
  }
  rf95_915.setTxPower(23, false);
  Serial.println("RX_915 init OK");
  delay(1000);

  #if do_sendSignal
    systemStart();
  #endif
} // END SETUP





 
void loop()  {
  if (rf95_915.available())   {
    if (rf95_915.recv(buf, &len))   {
      sprintf(radioPacket, "1,%s,%d", (char*)buf,rf95_915.lastRssi());
      Serial.println(radioPacket);
    }
  }
}






void systemStart()  {
  String dataString;
  unsigned long timeNow, timeLast = 0;

  // waits for input from user to begin initialization
  Serial.println("Press the start button when ready!");
  while(digitalRead(startPB) == 1) {
    delay(100);
  }
  Serial.println("Waiting for signal...");


  // ------------------ INITIALIZE TX_915 ----------------------
  // Sends start signals to TX_915 until we get a reply
  while (!rf95_915.available())  {
    if ((timeNow = millis()) - timeLast >= 1000)  {
      sendMsg("BEGIN STARTUP TX_915");
      timeLast = timeNow;
    }
  }
  // waits until all TX_915 systems have been verified
  while (dataString != "Ready to Start")  {
    if (rf95_915.available())   {
      if (rf95_915.recv(buf, &len))   {
        dataString = String((char*)buf);
        Serial.println(dataString);
      }
    }
  }


  // Wait for user input to start systems
  Serial.println("Press the start button when go for launch!");
  while(digitalRead(startPB) == 1) {
    if (rf95_915.available()) {
      if (rf95_915.recv(buf, &len)) {
        sprintf(radioPacket, "%s,%d", (char*)buf,rf95_915.lastRssi());
        Serial.println(radioPacket);
      }
    }
  }
  Serial.println("Sending Start Signal...");

  
  // ------------------ START TX_915 ----------------------
  // Sends start signals to TX_915 until we get a reply
  while (!rf95_915.available())  {
    timeNow = millis();
    if ((timeNow - timeLast) >= 1000) {
      sendMsg("TX_915 GO FOR LAUNCH");
      timeLast = timeNow;
    }
  }

  Serial.println("SYSTEM STARTING");
}





// _______________________________________________________________________________________
//                        *** RADIO STRING MESSAGE FUNCTION ***
// _______________________________________________________________________________________
void sendMsg(String dataString) {
  strncpy(radioPacket, dataString.c_str(), sizeof(radioPacket));
  rf95_915.send((uint8_t *)radioPacket, packetSize);
}
