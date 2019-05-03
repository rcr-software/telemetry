// TELEMETRY_DATA_TX
// -*- mode: C++ -*-
// NOTE: RH_RF95 class does not provide for addressing or reliability,
// so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.

// __________________________________________________________________
// *** HEADER FILES ***

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_GPS.h>
#include <utility/imumaths.h>

// __________________________________________________________________
// *** FEATHER INITIALIZATION ***

#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"

// Utilizing Frequency of 862 MHz for data collection
#define RF95_FREQ 862.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// __________________________________________________________________
// *** SD INITIALIZATION ***

#define cardSelect    4 
File dataFile;

// __________________________________________________________________
// *** BNO055, BMP280, & LIS3DH INITIALIZATION ***

//NOTE: ADR needs to be tied to 3V
Adafruit_BNO055 bno;
Adafruit_LIS3DH lis;
Adafruit_BMP280 bmp;

// __________________________________________________________________
// *** GPS INITIALIZATION ***

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false

// __________________________________________________________________
// *** VALUES ***

#define skip_calibrate false                                    // Skip BNO055 Calibration
#define DEBUG false                                             // Used for debugging purposes
#define VBATPIN A7                                              // Analog reading of battery voltage
#define LED 13                                                  // LED on M0 Feather
#define SEPARATE A5                                             // Separate Pin - Photoresistor/NPN Switch
unsigned long timeNow,timeLast,delta_t;                         // Time values
int dataLog = 0,radioLog = 0;
char fileName[30];                                              // .csv file, dynamically named "data_#"
const uint8_t packetSize = 100;                                 // Use this value to dynamicaly change the radio packet size
char radioPacket[packetSize];                                   // Char array used for radio packet transmission
float filteredVel,filteredVelPrev = 0,alt0,alt;                 // Vehicle State Values
sensors_event_t event;                                          // LIS3DH Sensor Values
float gpsLat,gpsLon,gpsSpeed,gpsAlt;                            // GPS values
bool separation;                                                // Vehicle separation detection
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];                           // Used for packet retrieval 
uint8_t len = sizeof(buf);                                      // Used for packet retrieval
float avgMotorThrust = 2585.5;                                  // Newtons
float avgVehicleMass = 30;                                      // Kilograms
float SeaLvlPressure = 1019.5;                                  // NOTE: Update with current sea level pressure (hPa)
                                                                // Go to: https://forecast.weather.gov for Barometer readings

struct stateStruct  {
  float accel;
};





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

    //      while(!Serial)  {
    //        delay(100);
    //      }
    
    // Radio Module Manual Reset ------------------------------
    digitalWrite(RFM95_RST, LOW);
    delay(100);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
    
    // Check RF95 and Frequency -------------------------------
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
    while (dataString != "BEGIN STARTUP TX1") {
     if (rf95.available())   {
        if (rf95.recv(buf, &len))   {
            dataString = String((char*)buf);
        }
      }
    }
    sendMsg("TX OK: 862MHz");
    delay(1000);
    
    // Initialize GPS ----------------------------------------

    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

    sendMsg("GPS Init OK");
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
        dataFile.println("Time,Alt,filtVel,filtAccel,Lat,Lon,Vbat,Temp,Sep,Gravity,Linear,rawAccel,gpsSpeed,gpsAlt");
        dataFile.close();
        break;
      }
    }
    delay(1000);
    
    // Check BNO055 ------------------------------------------
    if (!bno.begin()) {
        sendMsg("BNO055 Init Failed");
        while(1);
    }

    bno.setExtCrystalUse(true);

    // Vector Calibration ------------------------------------
    uint8_t system, gyro, accel, mag; // Used to calibrate BNO055
    system = gyro = accel = mag = 0;

    #if skip_calibrate
        while(!bno.isFullyCalibrated()) {
            bno.getCalibration(&system, &gyro, &accel, &mag);
            sprintf(radioPacket, "S:%d G:%d A:%d M:%d",system,gyro,accel,mag);
            rf95.send((uint8_t *)radioPacket, packetSize);
            delay(500);
        }
    #endif

    sendMsg("BNO055 Init OK");
    delay(1000);

    // Check LIS3DH ------------------------------------------
    if (!lis.begin()) {
      Serial.println("LIS3DH Init Failed");
      while(1);
    }
    
    lis.setRange(LIS3DH_RANGE_16_G);
    sendMsg("LIS3DH Init OK");
    delay(1000);

    // Check BMP280 ------------------------------------------
    if (!bmp.begin()) {   
      sendMsg("BMP280 Init Failed");
      while(1);
    }
          
    sendMsg("BMP280 Init OK");
    delay(1000);

    // Wait for Start Signal ---------------------------------
    sendMsg("Ready to Start");
    while (dataString != "TX1 GO FOR LAUNCH") {
     if (rf95.available())   {
        if (rf95.recv(buf, &len))   {
            dataString = String((char*)buf);
        }
      }
    }

    // Initialize variables -----------------------------------
    timeLast = millis();
    alt0 = bmp.readAltitude(SeaLvlPressure);

}   // END SETUP







// _______________________________________________________________________________________
//                                  *** LOOP ***
//                         Will send TX packet containing: 
//      [Time, Altitude, Velocity, Acceleration, GPS Lat & Lon, and Separation]
// _______________________________________________________________________________________
void loop() {
    struct stateStruct rawState, filteredState;
    
    // TIMESTAMP:
    delta_t = (timeNow = millis()) - timeLast;

    // Check for New GPS Packet
    GPS.read();
     
    if (GPS.newNMEAreceived()) {
      GPS.parse(GPS.lastNMEA());
    }
    
    if (delta_t >= 10)  {
      timeLast = timeNow; 
    
      // Read Altitude
      alt = bmp.readAltitude(SeaLvlPressure) - alt0;
      
      // Get Acceleration Vector Readings
      imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
      lis.getEvent(&event);
  
      float gx = (float)grav.x();
      float gy = (float)grav.y();
      float gz = (float)grav.z();
      float lx = (float)event.acceleration.x;
      float ly = (float)event.acceleration.y; 
      float lz = (float)event.acceleration.z;
  
      // get the magnitude & direction of gravity vectors
      float gravSum = gx + gy + gz;
      float gravity = sqrt(sq(gx) + sq(gy) + sq(gz));
      if (gravSum < 0) {
          gravity = -gravity;
      }

      // get the magnitude & direction of linear acceleration vectors
      float accelSum = lx + ly + lz;
      float linear = sqrt(sq(lx) + sq(ly) + sq(lz));
      if (accelSum < 0) {
          linear = -linear;
      }

      rawState.accel = linear - gravity;
      
      // Kalman Filter
      kalmanFilter(rawState, &filteredState);
  
      // Velocity Calculations
      // v = vo + at
      filteredVel = filteredVelPrev + (filteredState.accel * (delta_t/1000));
      filteredVelPrev = filteredVel;
      
      // Data Transmission
      dataLog += delta_t;
//      radioLog += delta_t;

      if (dataLog >= 250) {
          dataLog = 0;
        
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
  
          snprintf(radioPacket,packetSize, "%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.2f,%d,%d", float(timeNow)/1000,alt,filteredVel,filteredState.accel,gpsLat,gpsLon,vbat,temperature,separation);
    
          // Save data to SD file ~0.1 sec and radio transmit ~0.5 sec
          dataFile.print(radioPacket);
          rf95.send((uint8_t *)radioPacket, packetSize);
          rf95.waitPacketSent();
          
//          if (radioLog >= 500) {
//            radioLog = 0;
//            rf95.send((uint8_t *)radioPacket, packetSize);
//          }
          
          snprintf(radioPacket,packetSize, ",%.2f,%.2f,%.2f,%.2f,%.2f", gravity,linear,rawState.accel,gpsSpeed,gpsAlt);
          dataFile.println(radioPacket);
          
          // Close SD File
          dataFile.close();
      }
    
      #if DEBUG
        Serial.print("delta_t: ");
        Serial.println(delta_t);
        Serial.print("raw accel: ");
        Serial.println(rawState.accel);
        Serial.print("filtered accel: ");
        Serial.println(filteredState.accel);
        Serial.print("filteredVel: ");
        Serial.println(filteredVel);
      #endif
    }
}






// _______________________________________________________________________________________
//                        *** RADIO STRING MESSAGE FUNCTION ***
// _______________________________________________________________________________________
void sendMsg(String dataString) {
    strncpy(radioPacket, dataString.c_str(), sizeof(radioPacket));
    rf95.send((uint8_t *)radioPacket, packetSize);
}






// _______________________________________________________________________________________
//                           *** KALMAN FILTER FUNCTION ***
// _______________________________________________________________________________________
void kalmanFilter(struct stateStruct rawState, struct stateStruct* filteredState) {
    static float p_k = 0;                
    static float x_k = 0;
    float z_k = rawState.accel;
    float k_gain;
    float u_k = -9.81;
    
    // If R -> 0 : K -> 1 (primarily adjusts with measurement updates)
    // If R -> large : K -> 0 (primarily adjusts with predicted updates)
    // If P -> 0 : measurement updates are mostly ignored
    // Q is used to ensure P isn't too small or goes to zero.
    static float q_k = 0.2;
    static float r_k = 4;

    // Estimate the acceleration for the kalman filter
    if (z_k > 10)  {
      u_k += avgMotorThrust / avgVehicleMass;
    }
    else if (alt < 5) {  // On launch pad
      u_k = 0;
      filteredVelPrev = 0;
    }
    
    //PREDICT:
    p_k = q_k + p_k;

    //KALMAN GAIN:
    k_gain = p_k / (r_k + p_k);

    //UPDATE:
    x_k = x_k + (k_gain * (z_k - x_k));
    p_k = p_k * (1 - k_gain);

    filteredState->accel = x_k;
    
    #if DEBUG
      Serial.print("u_k: ");
      Serial.println(u_k);
      Serial.print("k_gain: ");
      Serial.println(k_gain);
      Serial.print("x_k: ");
      Serial.println(x_k);
      Serial.print("p_k: ");
      Serial.println(p_k);
    #endif
}
