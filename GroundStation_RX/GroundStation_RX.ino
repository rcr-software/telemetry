// TELEMETRY_DATA_RX
// -*- mode: C++ -*-
// RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
 
#include <RH_RF95.h>

#define RFM95_RST     11   // "A"
#define enable_862 true                  // Enables 862MHz radio in code scheme
#define enable_915 false                 // Enables 915MHz radio in code scheme
#define do_sendSignal false              // Sends start signals to TX


// for radio featherWing #1 - 862 MHz
#if enable_862
  #define RFM95_862_CS      10   // "B"
  #define RFM95_862_INT     6    // "D"
  #define RF95_862_FREQ 862.0
  RH_RF95 rf95_862(RFM95_862_CS, RFM95_862_INT);
#endif

// for radio featherWing #2 - 915 MHz
#if enable_915
  #define RFM95_915_CS      10   // "B"
  #define RFM95_915_INT     6    // "D"
  #define RF95_915_FREQ 915.0
  RH_RF95 rf95_915(RFM95_915_CS, RFM95_915_INT);
#endif

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

  #if enable_862
    if (!rf95_862.init() || !rf95_862.setFrequency(RF95_862_FREQ)) {
      Serial.println("RX #1 (862 MHz) init failed");
      while(1);
    }
    rf95_862.setTxPower(23, false);
    Serial.println("RX #1 OK: 862 MHz");
    delay(1000);
  #endif

  #if enable_915
    if (!rf95_915.init() || !rf95_915.setFrequency(RF95_915_FREQ)) {
      Serial.println("RX #1 (915 MHz) init failed");
      while(1);
    }
    rf95_915.setTxPower(23, false);
    Serial.println("RX #2 OK: 915 MHz");
    delay(1000);
  #endif

  #if do_sendSignal
    systemStart();
  #endif
} // END SETUP





 
void loop()  {
  #if enable_862
    if (rf95_862.available())   {
      if (rf95_862.recv(buf, &len))   {
        sprintf(radioPacket, "0,%s,%d", (char*)buf,rf95_862.lastRssi());
        Serial.println(radioPacket);
      }
    }
  #endif

  #if enable_915
    if (rf95_915.available())   {
      if (rf95_915.recv(buf, &len))   {
        sprintf(radioPacket, "1,%s,%d", (char*)buf,rf95_862.lastRssi());
        Serial.println(radioPacket);
      }
    }
  #endif
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
  #if enable_862
    while (!rf95_862.available())  {
      if ((timeNow = millis()) - timeLast >= 1000)  {
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
  #endif


  // ------------------ INITIALIZE TX_915 ----------------------
  // Sends start signals to TX_915 until we get a reply
  #if enable_915
    while (!rf95_915.available())  {
      if ((timeNow = millis()) - timeLast >= 1000)  {
        sendMsg("BEGIN STARTUP TX_915",915);
        timeLast = timeNow;
      }
    }
    // waits until all TX_915 systems have been verified
    while (dataString != "Ready to Start")  {
      if (rf95_915.available())   {
        if (rf95_915.recv(buf, &len))   {
          sprintf(radioPacket, "%s,%d", (char*)buf,rf95_915.lastRssi());
          Serial.println(radioPacket);
        }
      }
    }
  #endif

  // Wait for user input to start systems
  Serial.println("Type \"GO FOR LAUNCH\" when ready!");
  while(dataString != "GO FOR LAUNCH") {
    if (Serial.available() > 0) {
      dataString = Serial.readString();
    }
    
    #if enable_862
      if (rf95_862.available()) {
        if (rf95_862.recv(buf, &len)) {
          sprintf(radioPacket, "%s,%d", (char*)buf,rf95_862.lastRssi());
          Serial.println(radioPacket);
        }
      }
    #endif
    
    #if enable_915
      if (rf95_915.available()) {
        if (rf95_915.recv(buf, &len)) {
          sprintf(radioPacket, "%s,%d", (char*)buf,rf95_862.lastRssi());
          Serial.println(radioPacket);
        }
      }
    #endif
  }
  Serial.println("Sending Start Signal...");

  
  // ------------------ START TX_862 ----------------------
  // Sends start signals to TX_862 until we get a reply
  #if enable_862
    while (!rf95_862.available())  {
      timeNow = millis();
      if ((timeNow - timeLast) >= 1000) {
        sendMsg("TX_862 GO FOR LAUNCH",862);
        timeLast = timeNow;
      }
    }
  #endif
  
  // ------------------ START TX_915 ----------------------
  // Sends start signals to TX_915 until we get a reply
  #if enable_915
    while (!rf95_915.available())  {
      timeNow = millis();
      if ((timeNow - timeLast) >= 1000) {
        sendMsg("TX_915 GO FOR LAUNCH",915);
        timeLast = timeNow;
      }
    }
  #endif
  
  Serial.println("SYSTEM STARTING");
}





// _______________________________________________________________________________________
//                        *** RADIO STRING MESSAGE FUNCTION ***
// _______________________________________________________________________________________
void sendMsg(String dataString, int whichRX) {
    strncpy(radioPacket, dataString.c_str(), sizeof(radioPacket));
    #if enable_862
      if (whichRX == 862) {
        rf95_862.send((uint8_t *)radioPacket, packetSize);
      }
    #endif

    #if enable_915
      if (whichRX == 915) {
        rf95_915.send((uint8_t *)radioPacket, packetSize);
      }
    #endif
}
