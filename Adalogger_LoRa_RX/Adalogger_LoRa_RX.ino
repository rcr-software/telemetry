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
    
    // Blinky on receive
    #define LED 13
    #define START 12
    
    String initSequence;
    bool startup;
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
     
    void setup()
    {
      pinMode(LED, OUTPUT);
      pinMode(START,INPUT);
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
     
      if (!rf95.init()) {
        Serial.println("RX init failed");
        while(1);
      }

      if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("RX setFrequency failed");
        while(1);
      }
      
      Serial.println("RX OK: 862MHz");
      delay(1000);

      rf95.setTxPower(23, false);

      systemStart();
      
//      while (!startup)   {
//       startup = digitalRead(START);
//        delay(1000);
//      }

      // NEED A WAY OF VALIDATING IF THE SIGNAL GETS THROUGH TO START, & IF IT DOESN'T,
      // HAVE RX KEEP SENDING SIGNALS UNTIL IT GETS THROUGH.
      char radioPacket[] = "GO FOR LAUNCH";
      rf95.send((uint8_t*)radioPacket, sizeof(radioPacket));
      Serial.println("SYSTEM STARTING");
      
    } // END SETUP
     
    void loop()
    {
      if (rf95.available())   {
        // Should be a message for us now
        if (rf95.recv(buf, &len))   {
          digitalWrite(LED, HIGH);
//        RH_RF95::printBuffer("Received: ", buf, len);
          Serial.print((char*)buf);
          Serial.print(",");
          Serial.println(rf95.lastRssi(), DEC);
          digitalWrite(LED, LOW);
        }
      }
    }

    void systemStart()  
    {
      Serial.println("Waiting for signal...");
      while (initSequence != "Separation init OK")  {
        if (rf95.available())   {
          if (rf95.recv(buf, &len))   {
            digitalWrite(LED, HIGH);
            initSequence = String((char*)buf);
            Serial.print(initSequence);
            Serial.print(",");
            Serial.println(rf95.lastRssi(), DEC);
            digitalWrite(LED, LOW);
          }
        }
      }
      Serial.println("Ready to Start!");
    }
