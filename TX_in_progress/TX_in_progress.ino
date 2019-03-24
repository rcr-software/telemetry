// TELEMETRY_DATA_TX
// -*- mode: C++ -*-
// NOTE: RH_RF95 class does not provide for addressing or reliability,
// so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.

// __________________________________________________________________
// *** HEADER FILES ***

//#include <stdio.h>
//#include <string.h>
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
#include <BasicLinearAlgebra.h>
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
// *** BNO055 & BMP280 INITIALIZATION ***

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
// *** KALMAN VARIABLES ***

using namespace BLA;

BLA::Matrix<3,3> Q_k = {1,   0,   0,
                        0, 0.2,   0,
                        0,   0, 0.2};

BLA::Matrix<3,3> R_k = {0.5, 0, 0,
                          0, 4, 0,
                          0, 0, 7};

// __________________________________________________________________
// *** VALUES ***
#define DEBUG false                               // Used for debugging purposes
#define VBATPIN A7                                // Analog reading of battery voltage
#define LED 13                                    // LED on Adalogger
#define SEPARATE 11                               // Separate Pin - Pull Down Resistor
float timeNow,timeLast,timeLog,delta_t;           // Time values
char fileName[20];                                // .csv file, dynamically named "telemetryData_#"
const uint8_t packetSize = 60;                    // Use this value to change the radio packet size
char radioPacket[packetSize];                     // char array used for radio packet transmission
String dataString;                                // String used for radio packet transmission
float alt0;                                       // BMP280 altitude reading
float gpsLat,gpsLon;                              // GPS Latitude and Longitude
bool separation = 0;                              // Vehicle separation detection boolean
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];             // Used for packet retrieval 
uint8_t len = sizeof(buf);                        // Used for packet retrieval
sensors_event_t event;                            // Event capture for LIS3DH accelerometer readings
float SeaLvlPressure = 1019.5;                    // NOTE: Update with current sea level pressure
                                                  // Go to: https://forecast.weather.gov for Barometer readings

struct stateStruct  {
  float alt, vel, accel;
};




// __________________________________________________________________
// *** SETUP ***

void setup()  {

    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST,HIGH);
    pinMode(LED, OUTPUT);
    digitalWrite(LED,LOW);

    Serial.begin(115200);
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

    //      while(!Serial)  {
    //        delay(100);
    //      }

    //    Check RF95 and Frequency -------------------------------
    while(!rf95.init() || !rf95.setFrequency(RF95_FREQ)) {
        digitalWrite(LED,HIGH);
        delay(1000);
        digitalWrite(LED,LOW);
        delay(1000);
    }

    // TX Power set to max
    rf95.setTxPower(23, false);

#if DEBUG
    Serial << "Test" << "\n";
#endif

    //Send a signal that radio is ready to go
    sendMsg("TX OK: 862MHz");
    delay(1000);

    //Check SD ----------------------------------------------
    while(!SD.begin(cardSelect)) {
        sendMsg("No SD Card Detected");
        delay(5000);
    }

    for(int i=0;;i++)
    {
      String temp = "data_" + String(i) + ".csv";
      temp.toCharArray(fileName, sizeof(fileName));
      if (!SD.exists(fileName))  {
        dataString = "File: " + temp;
        sendMsg(dataString);
        delay(1000);

        dataFile = SD.open(fileName,FILE_WRITE);
        dataFile.println("Time,Alt,Vel,Accel,Lat,Lon,Temp,Vbat,Sep");
        dataFile.close();
        break;
      }
    }

    //Check BNO055 ------------------------------------------
    if (!bno.begin()) {
        sendMsg("BNO055 Init Failed");
        while(1);
    }

    bno.setExtCrystalUse(true);

    // Vector Calibration -----------------------------------
    uint8_t system, gyro, accel, mag;           // Used to calibrate BNO055
    system = gyro = accel = mag = 0;

    //      while(!bno.isFullyCalibrated()) {
    //          bno.getCalibration(&system, &gyro, &accel, &mag);
    //          dataString = "S:" + String(system) + " G:" + String(gyro) + " A:" + String(accel) + " M:" + String(mag);
    //          sendMsg(dataString);
    //          delay(500);
    //      }

    sendMsg("BNO055 Init OK");
    delay(1000);

    //Check LIS3DH ------------------------------------------
    if (!lis.begin()) {
      Serial.println("LIS3DH Init Failed");
      while(1);
    }
    
    lis.setRange(LIS3DH_RANGE_16_G);
    sendMsg("LIS3DH Init OK");
    delay(1000);

    //Check BMP280 ------------------------------------------
    if (!bmp.begin()) {   
      sendMsg("BMP280 Init Failed");
      while(1);
    }
          
    sendMsg("BMP280 Init OK");
    delay(1000);
          
    //Wait for GPS Fix --------------------------------------
//    while (!GPS.fix()) {
//        sendMsg("No GPS Fix");
//        if (GPS.newNMEAreceived()) {
//            if (GPS.parse(GPS.lastNMEA()))
//        }
//        delay(1000);
//    }

    sendMsg("GPS init OK");
    delay(1000);
    
    //Check Separation Hardware -----------------------------
    //      separation = digitalRead(SEPARATE);
    //      
    //      while (separation)  {
    //        sendMsg("Separation Not Ready");
    //        delay(5000);
    //        
    //        separation = digitalRead(SEPARATE);
    //      }

    sendMsg("Separation init OK");

    //      //Wait for Startup Signal -------------------------------
    //      while (dataString != "GO FOR LAUNCH") {
    //       if (rf95.available())   {
    //          if (rf95.recv(buf, &len))   {
    //              dataString = String((char*)buf);
    //          }
    //        }
    //      }

    //Initialize rocket variables ----------------------------------
    timeLast = millis();
    alt0 = bmp.readAltitude(SeaLvlPressure);

}   // END SETUP







// __________________________________________________________________
// *** LOOP ***
// *** Will send TX packet containing: [Time, Altitude, Velocity, Acceleration, GPS Lat & Lon, Temperature, Vbat, Separation] ***

void loop() {
    struct stateStruct rawState, filteredState;

    if (!separation) {
      
      // Read Altitude
      rawState.alt = bmp.readAltitude(SeaLvlPressure) - alt0;
  
      // Get Acceleration Vector Values
      float xL,yL,zL,xG,yG,zG,velPrev = 0;                  // Acceleration Values (p == previous value)
      imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
      lis.getEvent(&event);
  
      xG = (float)gravity.x();
      yG = (float)gravity.y();
      zG = (float)gravity.z();
      xL = (float)event.acceleration.x;
      yL = (float)event.acceleration.y; 
      zL = (float)event.acceleration.z;
      rawState.accel = (xL - xG) + (yL - yG) + (zL - zG);
  
      // Calculate Velocity from Acceleration Reading
      rawState.vel = (rawState.accel*delta_t) + velPrev;
  
      #if DEBUG
        Serial << "Alt: " << rawState.alt << '\n';
        Serial << "xG: " << xG << " yG: " << yG << " zG: " << zG << '\n';
        Serial << "xL: " << xL << " yL: " << yL << " zL: " << zL << '\n';
        Serial << "Accel: " << rawState.accel << '\n';
        Serial << "Vel: " << rawState.vel << '\n';
      #endif
  
      // KALMAN FILTER:
      kalmanFilter(rawState, &filteredState);
  
      // Update previous time and velocity
      velPrev = filteredState.vel;
      float timeLog += delta_t;
        
//    else  {
//        delay(5000);
//        if (GPS.newNMEAreceived()) {
//            if (GPS.parse(GPS.lastNMEA()))
//               gpsLat = GPS.lat;
//               gpsLon = GPS.lon;
//        }
//    }
      
        // Append Data
        if (timeLog > 0.5) {
          
          // Open SD File
          dataFile = SD.open(fileName,FILE_WRITE);
      
          // Read Battery Voltage
          float vbat = ((analogRead(VBATPIN))*3.3)/512;  // Convert to voltage  
      
          // Read Separation
//          separation = digitalRead(SEPARATE);
      
          // Read Temperature
          int temperature = bmp.readTemperature();
    
          dataString  = String(timeNow/1000) + ",";
          dataString += String(filteredState.alt) + ",";
          dataString += String(filteredState.vel) + ",";
          dataString += String(filteredState.accel) + ",";
//          dataString += String(gpsLat) + ",";
//          dataString += String(gpsLon) + ",";
          dataString += String(temperature) + ",";
          dataString += String(vbat) + ",";
          dataString += String(separation);
          timeLog = 0;
    
          // Save Data to SD File and Radio Transmit
          dataFile.println(dataString);
          sendMsg(dataString);
          
          // Close SD File
          dataFile.close();
        }
    }
}







// __________________________________________________________________
// *** Radio Message Function ***
void sendMsg(String dataString) {
    digitalWrite(LED,HIGH);
    strncpy(radioPacket, dataString.c_str(), sizeof(radioPacket));
    rf95.send((uint8_t *)radioPacket, packetSize);
    digitalWrite(LED,LOW);
}






// __________________________________________________________________
// *** KALMAN FILTER FUNCTION ***
void kalmanFilter(struct stateStruct rawState, struct stateStruct* filteredState) {
    static BLA::Matrix<3,3> p_k = {0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0};
    static BLA::Matrix<3> x_k = {0,0,0};
    BLA::Matrix<3>   z_k;
    BLA::Matrix<3,3> k_gain;
    BLA::Matrix<3>   tempMat;
    BLA::Matrix<3>   tempMat_1;
    BLA::Matrix<3>   tempMat_2;
    BLA::Matrix<3,3> tempMat_3;
    BLA::Matrix<3,3> tempMat_4;
    BLA::Matrix<1>   u_k = {0};
    BLA::Matrix<3>   B_k;
    BLA::Matrix<3,3> A_k = {1, 0, 0,
                            0, 1, 0,
                            0, 0, 0};

    delta_t = (float)((timeNow = millis()) - timeLast) / 1000;
    timeLast = timeNow;
    //Serial.println(delta_t);
    z_k(0) = rawState.alt;
    z_k(1) = rawState.vel;
    z_k(2) = rawState.accel;

    B_k(0) = (sq(delta_t)) / 2;
    //  b_k[0] = b_k[0] / 2;
    B_k(1) = delta_t;
    B_k(2) = 1;

    A_k(0,1) = delta_t;

    //PREDICT:
    //x_k = A_k*x_k' + B_k*u_k
    Multiply(B_k,u_k,tempMat);
    Multiply(A_k,x_k,tempMat_1);
    Add(tempMat,tempMat_1,x_k);
#if DEBUG
    Serial << "tempMat: " << tempMat << '\n';
    Serial << "tempMat_1: " << tempMat_1 << '\n';
    Serial << "x_k: " << x_k << '\n';
#endif

    //p_k = Q_k + A_k*p_k'*T(A_k)
    Multiply(A_k,p_k,tempMat_3);
    BLA::Matrix<3,3> A_k_T = ~A_k;
    Multiply(tempMat_3,A_k_T,tempMat_4);
    Add(Q_k,tempMat_4,p_k);
#if DEBUG
    Serial << "tempMat_3: " << tempMat_3 << '\n';
    Serial << "tempMat_4: " << tempMat_4 << '\n';
    Serial << "p_k: " << p_k << '\n';
#endif

    //KALMAN GAIN:
    //p_k*T(H_k) / (R_k + H_k * p_k * T(H_k)) == p_k / (R_k + p_k)    When h_k = eye(3)
    Add(R_k,p_k,tempMat_3);
    Invert(tempMat_3);
    Multiply(p_k,tempMat_3,k_gain);
#if DEBUG
    Serial << "tempMat_3: " << tempMat_3 << '\n';
    Serial << "k_gain: " << k_gain << '\n';
#endif

    //UPDATE:
    //x_k = x_k(-) + (k_gain * (z_k - x_k(-)))
    Subtract(z_k,x_k,tempMat_1);
    Multiply(k_gain,tempMat_1,tempMat_2);
    x_k += tempMat_2;
#if DEBUG
    Serial << "tempMat_1: " << tempMat_1 << '\n';
    Serial << "tempMat_2: " << tempMat_2 << '\n';
    Serial << "x_k: " << x_k << '\n';
#endif

    //p_k = p_k - (k_gain * p_k) IS THIS RIGHT???
    Multiply(k_gain,p_k,tempMat_3);
    p_k -= tempMat_3;
#if DEBUG
    Serial << "tempMat_3: " << tempMat_3 << '\n';
    Serial << "p_k: " << p_k << '\n';
#endif

    filteredState->alt = x_k(0);
    filteredState->vel = x_k(1);
    filteredState->accel = x_k(2);
    
#if DEBUG
    Serial.println(filteredState->alt);
    Serial.println(filteredState->vel);
    Serial.println(filteredState->accel);
#endif

    timeNow = millis();
}

