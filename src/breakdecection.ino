#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "SPIFFS.h"
#include <movingAvgFloat.h>
#include <PeakDetection.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

float newX=0,newZ=0,newY=0, rollX, pitchY, yawZ;

String myString="";

int FileCounter=0;
String path="";

PeakDetection peakDetection;

movingAvgFloat avgDataX(10);
movingAvgFloat avgDataY(10);
movingAvgFloat avgDataZ(10);

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    "); 
  
  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 "); 
      break;
    default:
      Serial.print  ("???? "); 
      break;
  }  
  Serial.println(" Hz");  
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}


void writeFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        //Serial.println("- file written");
    } else {
        //Serial.println("- frite failed");
    }

    file.close();
}

void setup(void) 
{
#ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif
  Serial.begin(115200);
  Serial.println("Accelerometer Test"); Serial.println("");


// Launch SPIFFS file system  
  if(!SPIFFS.begin()){ 
    Serial.println("An Error has occurred while mounting SPIFFS");  
  }

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  //accel.setRange(ADXL345_RANGE_16_G);
   accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  // accel.setRange(ADXL345_RANGE_2_G);

   accel.setDataRate(ADXL345_DATARATE_25_HZ);

  peakDetection.begin(8, 4, 0);   //

  avgDataX.begin();
  avgDataY.begin();
  avgDataZ.begin();
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Display additional settings (outside the scope of sensor_t) */
  displayDataRate();
  displayRange();
  Serial.println("");


  File root = SPIFFS.open("/");
  File file = root.openNextFile();
 
  while(file){
 
      //Serial.print("FILE: ");
      //Serial.println(file.name());
      file = root.openNextFile();
      FileCounter++;
  }

   if(FileCounter<10)
   {
    path = "/00"+ String(FileCounter)+".txt";
    }
    else
    {
      path = "/0"+ String(FileCounter)+".txt";
    }

  //writeFile(SPIFFS, "/test.txt", charBuf);

  file = SPIFFS.open(path, FILE_WRITE);
  file.println(path);
  file.close();
  
}


const float alpha = 0.3;

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);

  //newX = 0.6 * newX + 0.4 * event.acceleration.x;
  //newY = 0.6 * newY + 0.4 * event.acceleration.y;
  //newZ = 0.6 * newZ + 0.4 * event.acceleration.z;


  //Low Pass Filter
    //newX = event.acceleration.x * alpha + (newX * (1.0 - alpha));
    //newY = event.acceleration.y * alpha + (newY * (1.0 - alpha));
    //newZ = event.acceleration.z * alpha + (newZ * (1.0 - alpha));

  //Roll & Pitch Equations
    //rollX  = (atan2(-newY, newZ)*180.0)/M_PI;
    //pitchY = (atan2(newX, sqrt(newY*newY + newZ*newZ))*180.0)/M_PI;
    //YawZ

  //MMA
    float readThisX = event.acceleration.x; 
    float avgX = avgDataX.reading(readThisX); 


    float readThisY = event.acceleration.y + 0.8;

    if(readThisY < 0.0)
    {
      readThisY = 0;
    }
    if(readThisY > 8.0)
    {
      readThisY = 8.0;
    }

    float finalReadThisY = readThisY - 4.0;
    
    float avgY = avgDataY.reading(readThisY); 

    float readThisZ = event.acceleration.z; 
    float avgZ = avgDataZ.reading(readThisZ); 


  myString = String(avgX) + ", " + String(avgY) + ", " + String(avgZ);
  //myString.toCharArray(charBuf, 50);

 /* File file = SPIFFS.open(path, "a");
  file.println(myString);
  //file.close();

*/

  double dataSensor = ((double)avgDataY.reading(finalReadThisY)/4)-1.0;  //zmiana na -1.0 byla spoko
  peakDetection.add(dataSensor);
  int peak = peakDetection.getPeak();
  double filtered = peakDetection.getFilt();
  Serial.print(finalReadThisY);
  Serial.print(",");
  //Serial.print(readThisY);
  //Serial.print(",");
  Serial.print(dataSensor);
  Serial.print(",");
  Serial.println(peak);
  //Serial.print(",");
  //Serial.println(filtered);


 /* float dataSensor = (float)avgY;
  peakDetection.add(dataSensor);
  int peak = peakDetection.getPeak();
  double filtered = peakDetection.getFilt(); */



  /* Display the results (acceleration is measured in m/s^2) */
/*  Serial.print("X:"); *///Serial.print(avgX); Serial.print(", ");
/*  Serial.print("Y:");*///Serial.print(avgY); Serial.print(", ");
/*  Serial.print("Z:");*///Serial.print(peak); /*Serial.print(", ");*/
                          //Serial.println();



  // LOW-PASS FILTER

  delay(50);
}
