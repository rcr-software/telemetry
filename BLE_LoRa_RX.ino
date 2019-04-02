// TELEMETRY_DATA_RX
// -*- mode: C++ -*-
// RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
 
#include <RH_RF95.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"

// for feather m0  
//#define RFM95_CS 8
//#define RFM95_RST 4
//#define RFM95_INT 3

// for radio featherWing
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"

// Utilizing Frequency of 862 MHz for data collection
#define RF95_FREQ 862.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

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
    delay(500);
  }

  if ( !ble.begin(VERBOSE_MODE) )
  {
    Serial.println(F("Couldn't find Bluefruit, make sure it's in Command mode & check wiring?"));
  }

  // Perform a factory reset to make sure everything is in a known state
  Serial.println(F("Performing a factory reset: "));
  if (!ble.factoryReset() ){
       Serial.println(F("Couldn't factory reset"));
  }
  
  ble.echo(false);      // Disable command echo from Bluefruit
  ble.verbose(false);   // debug info is a little annoying after this point!

  // Wait for connection to finish
  while (!ble.isConnected()) {
      delay(5000);
  }

  ble.print("AT+BLEUARTTX=");
  ble.println("BLE OK");
  if (!ble.waitForOK() )
  {
    Serial.println(F("Failed to send_1"));
  }
  delay(1000);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(100);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
 
  if (!rf95.init()||!rf95.setFrequency(RF95_FREQ)) {
    ble.print("AT+BLEUARTTX=");
    ble.println("RX Radio Init Failed");
    if (!ble.waitForOK() )
    {
      Serial.println(F("Failed to send_2"));
    }
    while(1);
  }
  
  rf95.setTxPower(23, false);
  ble.print("AT+BLEUARTTX=");
  ble.println("RX Radio OK: 862 MHz");
  if (!ble.waitForOK() )
  {
    Serial.println(F("Failed to send_3"));
  }
  delay(1000);
  
  systemStart();
  
//  while (!startup)  {
//    startup = digitalRead(START);
//    delay(1000);
//  }

//  if (ble.isConnected())
//  {
//    ble.println("AT+BLEUARTRX");
//    ble.readline();
//    if (strcmp(ble.buffer, "OK") == 0) {
//      // no data
//      return;
//    }
//    // Some data was found, its in the buffer
//    Serial.print(F("[Recv] ")); Serial.println(ble.buffer);
//    ble.waitForOK();
//  }
  
  // NEED A WAY OF VALIDATING IF THE SIGNAL GETS THROUGH TO START, & IF IT DOESN'T,
  // HAVE RX KEEP SENDING SIGNALS UNTIL IT GETS THROUGH.
  char radioPacket[] = "GO FOR LAUNCH";
  rf95.send((uint8_t*)radioPacket, sizeof(radioPacket));

//  while (!rf95.available()) {
//    rf95.send((uint8_t*)radioPacket, sizeof(radioPacket));
//  }
  
  ble.print("AT+BLEUARTTX=");
  ble.println("SYSTEM STARTING");
  if (!ble.waitForOK() )
  {
    Serial.println(F("Failed to send_5"));
  }
  
} // END SETUP






 
void loop()
{
  if (rf95.available())   {
    if (rf95.recv(buf, &len))   {
      digitalWrite(LED, HIGH);
      ble.print("AT+BLEUARTTX=");
      ble.print((char*)buf);
      ble.print(",");
      ble.println(rf95.lastRssi(), DEC);
      if (!ble.waitForOK() )
      {
        Serial.println(F("Failed to send_6"));
      }
      digitalWrite(LED, LOW);
    }
  }
}





void systemStart()  
{
  while (initSequence != "Separation init OK")  {
    if (rf95.available())   {
      if (rf95.recv(buf, &len))   {
        digitalWrite(LED, HIGH);
        initSequence = String((char*)buf);
        ble.print("AT+BLEUARTTX=");
        ble.print(initSequence);
        ble.print(",");
        ble.println(rf95.lastRssi(), DEC);
        if (!ble.waitForOK() )
        {
          Serial.println(F("Failed to send_4"));
        }
        digitalWrite(LED, LOW);
      }
    }
  }
  Serial.println("Ready to Start!");
}
