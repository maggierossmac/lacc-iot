/* 
   Environmental Sensor
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.

   Authors: Hillel Mermelstein, Joshua Mermelstein, and Jayesh Bhakta
   Institution: Los Angeles City College, Los Angeles, CA 90029
   Date: 11/09/2018 
   Version: 1.0

   Harware Connections to Arduino:
   
   SCL-Carbon Dioxide Sensor SCL and RT Clock SCL
   SDA-Carbon Dioxide Sensor SCA and RT Clock SCA
   D0-GPS TX
   D4-PM2.4 Sensor RxD
   D5-PM2.5 Sensor TxD
   D7-Status LED
   D9-RTClock/SD Card (SDCS) - Chip select
   D11-RTClcok/SD Card (MOSI)
   D12-RTClock/SD Card (MISO)
   D13-RTClock(SCL)  
*/

/************************************************************************
*
* Include section
*
*************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h"
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "SparkFun_SCD30_Arduino_Library.h" 


/************************************************************************
*
* Global Variables/Defines
*
*************************************************************************/
//debugging messages, define DEBUG to enable monitoring messages from serial port
//#define DEBUG 1

//software serial connections
#define PMS_TX 5
#define PMS_RX 6

//CO2 sensor
SCD30 airSensor;

//GPS
TinyGPS gps; // create gps object
long lat;
long lon; // create variable for latitude and longitude object
unsigned long fix;
char gpsChar;
bool newgpsdata;

//PM2.5 Particle Sensor
SoftwareSerial pmsSerial(PMS_TX,PMS_RX);
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data data;

struct sensor_data {
  char date_string[25];
  bool pm_valid;
  uint16_t pm10s;
  uint16_t pm25s;
  uint16_t pm100s;
  bool CO2_valid;
  uint16_t CO2;
  float humidity;
  float temp;
  bool gps_valid;
  long lat;
  long lon;  
};

struct sensor_data last_sensor_data;

//RTC SD
RTC_DS1307 RTC;
const int chipSelect = 9;
char dateString[25];
DateTime now;
    
//Status LED
const int statusLED=7;

/************************************************************************
*
* Setup
*
*************************************************************************/
void setup() {
  //the hardware serial RX is used to receive GPS data, TX is used for device monitoring
  Serial.begin(4800);
  pmsSerial.begin(9600);
  
  //I2C interfaces
  Wire.begin();
  RTC.begin();
  airSensor.begin(); 

  //set time
  if (!RTC.isrunning()) 
  {
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  
  // make sure that the chip select pin is set to output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);

  //set up status LED
  pinMode(statusLED,OUTPUT);
  digitalWrite(statusLED,LOW);
}


/************************************************************************
*
* loop function
*
*************************************************************************/
void loop() {

  //set the status light to on
  digitalWrite(statusLED,HIGH);

  //get the current datetime
  now = RTC.now();
  sprintf(dateString, "%02d/%02d/%02d %02d:%02d:%02d   ",
        now.month(), now.day(), now.year(), now.hour(), now.minute(), now.second());
  
  //copy string to last sensor data block
  strncpy(last_sensor_data.date_string,dateString,25);
   
  if (readPMSdata())
  {
    //set values in last sensor data struct
    last_sensor_data.pm_valid=true;
    last_sensor_data.pm25s=data.pm25_standard;
    last_sensor_data.pm10s=data.pm10_standard;
    last_sensor_data.pm100s=data.pm100_standard;          
  }
  else 
  {
    last_sensor_data.pm_valid=false;   
  }

  //Get the particle sensor data
  if (airSensor.dataAvailable())
  {
    //set last sensor data values
    last_sensor_data.CO2_valid=true;
    last_sensor_data.CO2=airSensor.getCO2();
    last_sensor_data.temp=airSensor.getTemperature();
    last_sensor_data.humidity=airSensor.getHumidity();   
  }
  else 
  {
    last_sensor_data.CO2_valid=false;
  }
  
  //get gps data

  //set the newdata flag to false
  newgpsdata=false;
  
  //try for 2 seconds to get a valid reading
  for (fix = millis(); millis() - fix < 2000;)
  {
    if(Serial.available())
    {
      gpsChar=Serial.read();
    
      if(gps.encode(gpsChar))
      {
        newgpsdata=true;
        break;
      }
    }
  }

  if(!newgpsdata)
  {     
    last_sensor_data.gps_valid=false;
  }
  else
  {
    // encode gps data
    gps.get_position(&lat,&lon,&fix); // get latitude and longitude
    
    //set last sensor data values
    last_sensor_data.gps_valid=true;
    last_sensor_data.lat=lat;
    last_sensor_data.lon=lon;
  
    //blink LED 3 times
    for(fix=0;fix<3;fix++)
    {
      digitalWrite(statusLED,LOW);
      delay(250);
      digitalWrite(statusLED,HIGH);
      delay(250);
    }
  }

  //transmit the data
  txData();
  
  //delay 6 seconds
  delay(6000);
     
}
//End of Loop


/*************************************************************************
* Function name      : readPMSdata
* returns            : true/false
* Description        : processes data returned from particulate sensor
* Notes              : obtained from https://learn.adafruit.com/pm25-air-quality-sensor/arduino-code
**************************************************************************/
boolean readPMSdata() {
  
  //wait for characters to be received
  while(! (pmsSerial.available()>32)) {}
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  while (pmsSerial.peek() != 0x42) {
    pmsSerial.read();
  }
 
  uint8_t buffer[32];    
  uint16_t sum = 0;
  pmsSerial.readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) {
    return false;
  }
  // success!
  return true;
}


/*************************************************************************
* Function name      : txData
* returns            : void
* Description        : transmits last sensor data
* Notes              : 
**************************************************************************/
void txData(){

  //send data in JSON format
  Serial.print("{");

  //date element
  Serial.print("\"");
  Serial.print("date");
  Serial.print("\""); 
  Serial.print(":");
  Serial.print("\""); 
  Serial.print(last_sensor_data.date_string);
  Serial.print("\"");
  Serial.print(",");
  Serial.flush();

  //pm10s element
  Serial.print("\"");
  Serial.print("pm1");
  Serial.print("\""); 
  Serial.print(":"); 
  Serial.print(last_sensor_data.pm10s);
  Serial.print(",");
  Serial.flush();
  
  //pm25s element
  Serial.print("\"");
  Serial.print("pm2.5");
  Serial.print("\""); 
  Serial.print(":");
  Serial.print(last_sensor_data.pm25s);
  Serial.print(",");
  Serial.flush();
  
  //pm100s element
  Serial.print("\"");
  Serial.print("pm10");
  Serial.print("\""); 
  Serial.print(":");  
  Serial.print(last_sensor_data.pm100s);
  Serial.print(",");
  Serial.flush();

  //CO2 element
  Serial.print("\"");
  Serial.print("CO2");
  Serial.print("\""); 
  Serial.print(":"); 
  Serial.print(last_sensor_data.CO2);
  Serial.print(",");
  Serial.flush();

  //humidity element
  Serial.print("\"");
  Serial.print("humidity");
  Serial.print("\""); 
  Serial.print(":"); 
  Serial.print(last_sensor_data.humidity);
  Serial.print(",");
  Serial.flush();
  
  //temp element
  Serial.print("\"");
  Serial.print("temp");
  Serial.print("\""); 
  Serial.print(":");
  Serial.print(last_sensor_data.temp);
  Serial.print(",");
  Serial.flush();
  
  //lat element
  Serial.print("\"");
  Serial.print("lat");
  Serial.print("\""); 
  Serial.print(":"); 
  Serial.print(last_sensor_data.lat);
  Serial.print(",");
  Serial.flush();
  
  //lon element
  Serial.print("\"");
  Serial.print("lon");
  Serial.print("\""); 
  Serial.print(":");
  Serial.print(last_sensor_data.lon);
  Serial.println("}");
  Serial.flush(); 
  
  
}
/*end of file*/
