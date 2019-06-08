// TELEMETRY_DATA_TX
// -*- mode: C++ -*-
// NOTE: RH_RF95 class does not provide for addressing or reliability,
// so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.

// __________________________________________________________________
// *** HEADER FILES ***

#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <RH_RF95.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GPS.h>

// __________________________________________________________________
// *** RADIO FEATHER INITIALIZATION ***

#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"

// Utilizing Frequency of 915 MHz for data collection
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// __________________________________________________________________
// *** SD INITIALIZATION ***

#define cardSelect    4 
File dataFile;

// __________________________________________________________________
// *** BNO055 & BMP280 INITIALIZATION ***

Adafruit_BMP280 bmp;

// __________________________________________________________________
// *** GPS INITIALIZATION ***

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false

// __________________________________________________________________
// *** VALUES ***

#define DEBUG false                                     // Used for debugging purposes
#define VBATPIN A7                                      // Analog reading of battery voltage
#define LED 13                                          // LED on Adalogger
#define SEPARATE A5                                     // Separate Pin - Pull Down Resistor
unsigned long timeNow,timeLast,delta_t;                 // Time values
char fileName[30];                                      // .csv file, dynamically named "data_#"
const uint8_t packetSize = 100;                         // Use this value to change the radio packet size
char radioPacket[packetSize];                           // char array used for radio packet transmission
float alt0;                                             // Pad altitude
float gpsLat,gpsLon,gpsSpeed,gpsAlt;                    // GPS values
bool separation;                                        // Vehicle separation detection boolean
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];                   // Used for packet retrieval 
uint8_t len = sizeof(buf);                              // Used for packet retrieval
float SeaLvlPressure = 1019.5;                          // NOTE: Update with current sea level pressure
                                                        // Go to: https://forecast.weather.gov for Barometer readings





// _______________________________________________________________________________________
//                                    *** SETUP ***
// _______________________________________________________________________________________


void setup()  {

    pinMode(RFM95_RST, OUTPUT);
    pinMode(SEPARATE, INPUT_PULLUP);
    digitalWrite(RFM95_RST,HIGH);
    pinMode(LED, OUTPUT);
    digitalWrite(LED,LOW);
    
    Serial.begin(115200);

//    while(!Serial)  {
//      delay(100);
//    }


    // Radio Module Manual Reset ------------------------------
    digitalWrite(RFM95_RST, LOW);
    delay(100);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);

    
//    Check RF95 and Frequency -------------------------------
    // LED will blink red on TX board if init fails
    while(!rf95.init() || !rf95.setFrequency(RF95_FREQ)) {
        digitalWrite(LED,HIGH);
        delay(1000);
        digitalWrite(LED,LOW);
        delay(1000);
    }

    // TX Power set to max
    rf95.setTxPower(23, false);

    // Wait for Initialize Signal -----------------------------
    String dataString;
    while (dataString != "BEGIN STARTUP TX_915") {
     if (rf95.available())   {
        if (rf95.recv(buf, &len))   {
            dataString = String((char*)buf);
        }
      }
    }

    sendMsg("TX OK: 915 MHz");
    delay(1000);
    
    // Check SD ----------------------------------------------
    while(!SD.begin(cardSelect)) {
        sendMsg("No SD Card Detected");
        while(1);
    }

    for(int i=0;;i++)
    {
      String temp = "data_" + String(i) + ".csv";
      temp.toCharArray(fileName, sizeof(fileName));
      if (!SD.exists(fileName))  {
        sprintf(radioPacket, "File: data_%d", i);
        rf95.send((uint8_t *)radioPacket, packetSize);

        dataFile = SD.open(fileName,FILE_WRITE);
        dataFile.println("Time,Alt,gpsAlt,gpsSpeed,Lat,Lon,Vbat,Temp,Sep");
        dataFile.close();
        break;
      }
    }
    delay(1000);
    
    //Check BMP280 ------------------------------------------
    if (!bmp.begin()) {   
      sendMsg("BMP280 Init Failed");
      while(1);
    }
          
    sendMsg("BMP280 Init OK");
    delay(1000);
          
    // Initialize GPS ----------------------------------------

    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

    sendMsg("GPS Init OK");
    delay(1000);

    //Wait for Startup Signal -------------------------------
    sendMsg("Ready to Start");
    while (dataString != "TX_915 GO FOR LAUNCH") {
     if (rf95.available())   {
        if (rf95.recv(buf, &len))   {
            dataString = String((char*)buf);
        }
      }
    }
    
    //Initialize rocket variables ----------------------------------
    timeLast = millis();

}   // END SETUP







// _______________________________________________________________________________________
//                                  *** LOOP ***
//                         Will send TX packet containing: 
// [Time, Altitude, GPS Speed, GPS Altitude, GPS Lat & Lon, Temperature, Vbat, Separation]
// _______________________________________________________________________________________

void loop() {
   // TIMESTAMP:
   delta_t = (timeNow = millis()) - timeLast;
   
   // Check for New GPS Packet
   GPS.read();
   if (GPS.newNMEAreceived()) {
     if (!GPS.parse(GPS.lastNMEA()))  
       return;
   }
  
   if (delta_t >= 200) {
      timeLast = timeNow;
      
      // Read Altitude
      float alt = bmp.readAltitude(SeaLvlPressure);
      
      // Read Battery Voltage
      float vbat = ((analogRead(VBATPIN))*3.3)/512;  // Convert to voltage  
  
      // Read Separation
      separation = digitalRead(SEPARATE);
  
      // Read Temperature
      int temperature = bmp.readTemperature();

      // Read GPS coordinates
      if (GPS.fix)  {
        gpsLat = GPS.latitudeDegrees;
        gpsLon = GPS.longitudeDegrees;
        gpsSpeed = GPS.speed;
        gpsAlt = GPS.altitude;
      }
      
      // Open SD File
      dataFile = SD.open(fileName,FILE_WRITE);
      
      // Save Data to SD File and Radio Transmit
      sprintf(radioPacket, "%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.2f,%d,%d", float(timeNow)/1000,alt,gpsAlt,gpsSpeed,gpsLat,gpsLon,vbat,temperature,separation);
      dataFile.println(radioPacket);
      rf95.send((uint8_t *)radioPacket, packetSize);
      rf95.waitPacketSent();
            
      // Close SD File
      dataFile.close();
   }
}





// _______________________________________________________________________________________
//                        *** RADIO STRING MESSAGE FUNCTION ***
// _______________________________________________________________________________________
void sendMsg(String dataString) {
    strncpy(radioPacket, dataString.c_str(), sizeof(radioPacket));
    rf95.send((uint8_t *)radioPacket, packetSize);
}

