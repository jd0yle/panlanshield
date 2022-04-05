#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DotStar.h>
#include "Adafruit_TSL2591.h"
#include <ArduinoJson.h>
#include "BranchNode.h"


/**
* CONFIG
**/
#define ENABLESERIAL true;

/**
* LIGHT SENSOR
**/
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

/**
* LSM6DS032 Accelerometer, on secondary I2C bus
* (ESP32 has 2 I2C busses, one for the board pins and the second for the STEMMA port)
**/
#define DEFAULT_I2C_PORT &Wire
#define SECONDARY_I2C_PORT &Wire1

Adafruit_LSM6DSO32 dso32;

/**
* DOTSTAR STRIP
* Using SPI for Dotstar; doesn't need number (uses SCL and MOSI)
**/
#define NUMPIXELS 522
Adafruit_DotStar strip(NUMPIXELS, DOTSTAR_BGR);


/**
* LED INDEX CONFIG
**/
struct Branch {
    int index;
    int parentIndex;
    int childIndexes[2];
};

int numBranches = 10;
Branch branches[10];

int endPointIndexes[] = {47, 69, 122, 135, 151, 165, 201, 213, 246, 257, 301, 314, 348, 364, 399, 415, 446, 463, 492, 512}; 
int endPointIndexesLength = sizeof endPointIndexes / sizeof endPointIndexes[0];

int midEndIndexes[] = {7, 48, 70, 91, 123, 136, 152, 166, 169, 188, 202, 214, 236, 247, 258, 268, 294, 302, 315, 332, 349, 365, 400, 416, 424, 447, 464, 477, 493};
int midEndIndexesLength = sizeof midEndIndexes / sizeof midEndIndexes[0];

int powerStartIndexes[] = {0, 70, 166, 258, 365, 416};
int powerStartIndexesLength = sizeof powerStartIndexes / sizeof powerStartIndexes[0];


void setup() {
    //#ifdef ENABLESERIAL
    Serial.begin(9600);
    if (!Serial) {
        delay(100); // will pause to see if serial comes online, but won't halt indefinitely
    }

    Serial.println("Starting up...");
    //#endif

    // Initialize SD library
/*     if (!SD.begin()) {
      Serial.println(F("Failed to initialize SD library"));
      delay(500);
    } */




    loadBranchesFile();
    setupAccelerometer();
    setupLightSensor();

    /**
     * Dotstar Strip Setup
     */
    strip.begin(); // Initialize pins for output
    strip.clear();
    strip.show();    // Turn all LEDs off ASAP
    strip.setBrightness(25);

}

int startIndex = 0;
int strikeLength = 72;
float intensity = 1.0; 
float duration = .75;

uint32_t lightningColor = strip.Color(255, 255, 255);

int toggle = 0;

void loop() {
  //  /* Get the light sensor reading */
//    sensors_event_t accel;
//    sensors_event_t gyro;
//    sensors_event_t temp;
//    dso32.getEvent(&accel, &gyro, &temp);
//    printAccelData(accel, gyro, temp);
//
//    //intensity = (accel.acceleration.y + 10) / 20;
//
//    //uint16_t x = tsl.getLuminosity(TSL2591_VISIBLE);
//    float x = tsl.getLuminosity(TSL2591_VISIBLE);
//    intensity = x / 60000;
//    Serial.print("X: "); Serial.println(x);
//    Serial.print("Intensity: "); Serial.println(intensity);

// UNCOMMENT THIS FOR TESTING LIGHT INTENSITY
//    for (int i = 0; i < NUMPIXELS; i++) {
//      strip.setPixelColor(i, (accel.acceleration.x + 10) / 20 * 255, 0, 196); 
//    }
//
//    strip.setBrightness(255 * intensity);
//
//    strip.show();
//
//    simpleRead();
//
//    //delay(20);
//
//    return;

    pixelIndexTest();

    int i = 0;
    while(branches[i].index > -1 && i < numBranches) {
        strip.setPixelColor(branches[i].index, 64, 255, 0);
        strip.setPixelColor(branches[i].childIndexes[0], 255, 64, 0);
        strip.setPixelColor(branches[i].childIndexes[1], 255, 64, 0);
        i++;
    }
    strip.show();
    delay(200);
    /* if (spiffsLoaded) {
        strip.setPixelColor(1, 0, 255, 128);
        strip.show();
        delay(200);
    } else {
        strip.setPixelColor(2, 255, 0, 64);
        strip.show();
        delay(200);
    } */

    int strikePath[30] = [9, 8, 7, 3, 2, 1];

    return;
  
   
    //      for (int blinkIndex = startIndex; blinkIndex < startIndex + strikeLength; blinkIndex += 5) {
      for (int blinkIndex = startIndex; blinkIndex < startIndex + strikeLength; blinkIndex += 3) {    
      	strip.setPixelColor(blinkIndex, lightningColor);
      	strip.setPixelColor(blinkIndex + 1, lightningColor);
      	strip.setPixelColor(blinkIndex + 2, lightningColor);
      	//	strip.setPixelColor(blinkIndex + 3, lightningColor);
      	//	strip.setPixelColor(blinkIndex + 4, lightningColor);	
      	strip.show();
      	delay(1);
      }
      
      Serial.println("Starting strike paint");
      for (int i = startIndex; i < startIndex + strikeLength; i++) {
	      strip.setPixelColor(i, lightningColor);
      }
      Serial.println("Ended strike paint");
      
      for (int b = 40; b > 12; b = b - 8) {
      	strip.setBrightness(b * intensity);
      	strip.show();
      	delay(20 * duration);
      }
      
      strip.setBrightness(100 * intensity);
      strip.show();
      delay(10 * duration);      
      
      strip.setBrightness(6 * intensity);
      strip.show();      
      delay(50 * duration);
      
      /*      strip.setBrightness(25 * intensity);
      strip.show();      
      delay(50);
      
      strip.setBrightness(12 * intensity);
      strip.show();      
      delay(100);
      
      strip.setBrightness(75 * intensity);
      strip.show();      
      delay(100);
      */
      //      strip.setBrightness(0 * intensity);
      strip.setBrightness(1);
      strip.show();      
      delay(50 * duration);

      strip.setBrightness(100 * intensity);
      strip.show();      
      delay(150 * duration);

      //      for (int b = 75; b > 12; b = b - 8) {
      for (int b = 100; b > 12; b = b - 8) {      
      	strip.setBrightness(b * intensity);
      	strip.show();
      	delay(1);
      }

      strip.setBrightness(1);
      strip.show();
      delay(1);
      
      strip.clear();
      strip.show();
      delay(1000);


      startIndex = startIndex + strikeLength;
      if (startIndex > NUMPIXELS) {
	      //startIndex = 145;
	  	  startIndex = 0;
      }
      
    /*    blinkIndex++;
    if (blinkIndex >= 230) {
      blinkIndex = 200;
    }*/
			
    
    /*    float theta = atan2(lis.x, lis.y) * 180 / 3.14159265 + 180;
    int ledIndex = NUMPIXELS - round(theta / 360 * 288);
    
    strip.clear();
    setPixelColor(ledIndex, strip.Color(0, 255, 0)); 
    for(int x = 1; x < 10; x++) {
        setPixelColor(ledIndex + x, strip.Color(1, 10 - x, 0));
        setPixelColor(ledIndex - x, strip.Color(1, 10 - x, 0));        
    }    
    strip.show();
    delay(25);*/
}


/**
* Accelerometer Setup
*/
void setupAccelerometer() {
    #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
      // ESP32 is kinda odd in that secondary I2C ports must be manually assigned their pins with setPins()!
      Wire1.setPins(SDA1, SCL1);
    #endif

    if (!dso32.begin_I2C(0x6A, &Wire1)) {
      Serial.println("Failed to find LSM6DSO32 chip");
      //while (1) { delay(10); }
    }
    dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
    Serial.print("Accelerometer range set to: ");
    Serial.println(dso32.getAccelRange());

    // dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
    Serial.print("Gyro range set to: ");
    Serial.println(dso32.getGyroRange());

    dso32.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
    Serial.print("Accelerometer data rate set to: ");
    Serial.println(dso32.getAccelDataRate());

    // dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
    Serial.print("Gyro data rate set to: ");
    Serial.println(dso32.getGyroDataRate());
}


void setupLightSensor(void) {
    if (tsl.begin(&Wire1, 0x29))  {
        Serial.println(F("Found a TSL2591 sensor"));
    } else {
        Serial.println(F("No sensor found ... check your wiring?"));
    }

    // You can change the gain on the fly, to adapt to brighter/dimmer light situations
    //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
    tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
    //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
    
    // Changing the integration time gives you a longer time over which to sense light
    // longer timelines are slower, but are good in very low light situtations!
    //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)
  
    /* Display the gain and integration time for reference sake */  
    Serial.println(F("------------------------------------"));
    Serial.print  (F("Gain:         "));
    tsl2591Gain_t gain = tsl.getGain();
    switch(gain)
    {
      case TSL2591_GAIN_LOW:
        Serial.println(F("1x (Low)"));
        break;
      case TSL2591_GAIN_MED:
        Serial.println(F("25x (Medium)"));
        break;
      case TSL2591_GAIN_HIGH:
        Serial.println(F("428x (High)"));
        break;
      case TSL2591_GAIN_MAX:
        Serial.println(F("9876x (Max)"));
        break;
    }
    Serial.print  (F("Timing:       "));
    Serial.print((tsl.getTiming() + 1) * 100, DEC); 
    Serial.println(F(" ms"));
    Serial.println(F("------------------------------------"));
    Serial.println(F(""));
}


void printAccelData (sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp) {
    if(Serial) {
        Serial.print("\t\tTemperature ");
        Serial.print(temp.temperature);
        Serial.println(" deg C");

        /* Display the results (acceleration is measured in m/s^2) */
        Serial.print("\t\tAccel X: ");
        Serial.print(accel.acceleration.x);
        Serial.print("\t\tAccel X RAW: ");
        Serial.print(dso32.rawAccX);
        Serial.print(" \tY: ");
        Serial.print(accel.acceleration.y);
        Serial.print(" \tZ: ");
        Serial.print(accel.acceleration.z);
        Serial.println(" m/s^2 ");

        /* Display the results (rotation is measured in rad/s) */
        Serial.print("\t\tGyro X: ");
        Serial.print(gyro.gyro.x);
        Serial.print(" \tY: ");
        Serial.print(gyro.gyro.y);
        Serial.print(" \tZ: ");
        Serial.print(gyro.gyro.z);
        Serial.println(" radians/s ");
        Serial.println();
    }
}

void loadBranchesFile() {
    //const char *jsonFilename = "./branches.json";
    //File jsonFile = SD.open(jsonFilename);

    const char *jsonFile = "{"nodes":[{"index":6,"parentIndex":5,"childIndexes":[7,70]},{"index":32,"parentIndex":31,"childIndexes":[33,48]},{"index":85,"parentIndex":84,"childIndexes":[86,91]},{"index":90,"parentIndex":89,"childIndexes":[136,152]},{"index":103,"parentIndex":102,"childIndexes":[104,123]}]}";

    StaticJsonDocument<1024> doc;

    DeserializationError error = deserializeJson(doc, jsonFile);
    if (error) {
        Serial.println(F("Failed to read file, using default configuration"));
    }

    int i = 0;
    while (doc["nodes"][i]) {
        branches[i].index = doc["nodes"][i]["index"];
        branches[i].childIndexes[0] = doc["nodes"][i]["childIndexes"][0];
        branches[i].childIndexes[1] = doc["nodes"][i]["childIndexes"][1];
        i++;
    }

}

void pixelIndexTest() {
  for (int i = 0; i < strip.numPixels(); i++){
    if (i % 100 == 0) {
      strip.setPixelColor(i, 255, 255, 255);
    } else if (i % 10 == 0) {
      strip.setPixelColor(i, 64, 64, 64);
    } else {
      strip.setPixelColor(i, 16, 16, 16);
    }
    
    for (int x = 0; x < endPointIndexesLength && endPointIndexes[x] <= i; x++) {
      if (endPointIndexes[x] == i) {
        strip.setPixelColor(i, 0, 0, 255);
      }
    }
    for (int x = 0; x < midEndIndexesLength && midEndIndexes[x] <= i; x++) {
      if (midEndIndexes[x] == i) {
        strip.setPixelColor(i, 255, 0, 0);
      }
    }
    for (int x = 0; x < powerStartIndexesLength && powerStartIndexes[x] <= i; x++) {
      if (powerStartIndexes[x] == i) {
        strip.setPixelColor(i, 0, 255, 0);
      }
    }
  }  
  //strip.fill(strip.Color(255, 255, 255));
  strip.show();
  return;
}

void setPixelColor (int index, uint32_t color) {
    if (index > NUMPIXELS - 1) {
        setPixelColor(index - NUMPIXELS, color);
    }
    if (index < 0) {
        setPixelColor(index + NUMPIXELS, color);
    }
    if (index < NUMPIXELS) {
        strip.setPixelColor(index, color);
    } 
    return;
}



void simpleRead(void) {
  // Simple data read example. Just read the infrared, fullspecrtrum diode 
  // or 'visible' (difference between the two) channels.
  // This can take 100-600 milliseconds! Uncomment whichever of the following you want to read
  uint16_t x = tsl.getLuminosity(TSL2591_VISIBLE);
  //uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  //uint16_t x = tsl.getLuminosity(TSL2591_INFRARED);

  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("Luminosity: "));
  //Serial.println(x, DEC);
  Serial.println(x);
}







/*
    // Accelerometer Setup
    #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
      // ESP32 is kinda odd in that secondary I2C ports must be manually assigned their pins with setPins()!
      Wire1.setPins(SDA1, SCL1);
    #endif

    if (!dso32.begin_I2C(0x6A, &Wire1)) {
      Serial.println("Failed to find LSM6DSO32 chip");
//      while (1) {
//        delay(10);
//      }
    }
  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (dso32.getAccelRange()) {
    case LSM6DSO32_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case LSM6DSO32_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case LSM6DSO32_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
    case LSM6DSO32_ACCEL_RANGE_32_G:
      Serial.println("+-32G");
      break;
  }

  // dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (dso32.getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS:
      Serial.println("125 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_250_DPS:
      Serial.println("250 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
    case ISM330DHCX_GYRO_RANGE_4000_DPS:
      break; // unsupported range for the DSO32
  }

  dso32.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (dso32.getAccelDataRate()) {
      case LSM6DS_RATE_SHUTDOWN:
        Serial.println("0 Hz");
        break;
      case LSM6DS_RATE_12_5_HZ:
        Serial.println("12.5 Hz");
        break;
      case LSM6DS_RATE_26_HZ:
        Serial.println("26 Hz");
        break;
      case LSM6DS_RATE_52_HZ:
        Serial.println("52 Hz");
        break;
      case LSM6DS_RATE_104_HZ:
        Serial.println("104 Hz");
        break;
      case LSM6DS_RATE_208_HZ:
        Serial.println("208 Hz");
        break;
      case LSM6DS_RATE_416_HZ:
        Serial.println("416 Hz");
        break;
      case LSM6DS_RATE_833_HZ:
        Serial.println("833 Hz");
        break;
      case LSM6DS_RATE_1_66K_HZ:
        Serial.println("1.66 KHz");
        break;
      case LSM6DS_RATE_3_33K_HZ:
        Serial.println("3.33 KHz");
        break;
      case LSM6DS_RATE_6_66K_HZ:
        Serial.println("6.66 KHz");
        break;
  }

  // dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (dso32.getGyroDataRate()) {
      case LSM6DS_RATE_SHUTDOWN:
        Serial.println("0 Hz");
        break;
      case LSM6DS_RATE_12_5_HZ:
        Serial.println("12.5 Hz");
        break;
      case LSM6DS_RATE_26_HZ:
        Serial.println("26 Hz");
        break;
      case LSM6DS_RATE_52_HZ:
        Serial.println("52 Hz");
        break;
      case LSM6DS_RATE_104_HZ:
        Serial.println("104 Hz");
        break;
      case LSM6DS_RATE_208_HZ:
        Serial.println("208 Hz");
        break;
      case LSM6DS_RATE_416_HZ:
        Serial.println("416 Hz");
        break;
      case LSM6DS_RATE_833_HZ:
        Serial.println("833 Hz");
        break;
      case LSM6DS_RATE_1_66K_HZ:
        Serial.println("1.66 KHz");
        break;
      case LSM6DS_RATE_3_33K_HZ:
        Serial.println("3.33 KHz");
        break;
      case LSM6DS_RATE_6_66K_HZ:
        Serial.println("6.66 KHz");
        break;
  }
*/