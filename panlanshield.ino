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
#define NUMSTRIKEPATHS 3

struct Node {
    int index;
    int parentIndex;
    int childIndexes[2];
    bool isEndpoint = false;
    bool isBranchStart;
};

//int endPointIndexes[] = {47, 69, 122, 135, 151, 165, 201, 213, 246, 257, 301, 314, 348, 364, 399, 415, 446, 463, 492, 512};
int endPointIndexes[] = {122, 135, 151, 165, 201, 213, 246, 257, 301, 314, 348, 364, 399, 415, 446, 463, 492, 512, 47, 69};
int endPointIndexesLength = sizeof endPointIndexes / sizeof endPointIndexes[0];

int midEndIndexes[] = {7, 48, 70, 91, 123, 136, 152, 166, 169, 188, 202, 214, 236, 247, 258, 268, 294, 302, 315, 332, 349, 365, 400, 416, 424, 447, 464, 477, 493};
int midEndIndexesLength = sizeof midEndIndexes / sizeof midEndIndexes[0];

int powerStartIndexes[] = {0, 70, 166, 258, 365, 416};
int powerStartIndexesLength = sizeof powerStartIndexes / sizeof powerStartIndexes[0];

int centerNodeIndexes[] = {0, 166};
int centerNodeIndexesLength = sizeof centerNodeIndexes / sizeof centerNodeIndexes[0];

Node nodes[NUMPIXELS];



void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    //#ifdef ENABLESERIAL
    Serial.begin(9600);
    /* while (!Serial) {
        for (int x = 0; x < 10; x++) {
            digitalWrite(13, HIGH);
            delay(50);
            digitalWrite(13, LOW);
            delay(50);
        }
        //delay(100); // will pause to see if serial comes online, but won't halt indefinitely
    } */

    Serial.println("Starting up...");
    //#endif


    delay(500);

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
float intensity = .5;
float duration = .1;

uint32_t lightningColor = strip.Color(255, 255, 255);

int toggle = 0;
int endpointThingIndex = 2;
//int pathStartIndexes[NUMSTRIKEPATHS] = {47, 69, 122};
int pathStartIndexes[NUMSTRIKEPATHS];

void loop() {

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    dso32.getEvent(&accel, &gyro, &temp);
    printAccelData(accel, gyro, temp);
//
//    //intensity = (accel.acceleration.y + 10) / 20;
//
 //  /* Get the light sensor reading */
//    //uint16_t x = tsl.getLuminosity(TSL2591_VISIBLE);
//    float x = tsl.getLuminosity(TSL2591_VISIBLE);
//    intensity = x / 60000;
//    Serial.print("X: "); Serial.println(x);
//    Serial.print("Intensity: "); Serial.println(intensity);

    /* pixelIndexTest();
    return; */

    /* lightningStrike();
    return; */

    //float theta = atan2(lis.x, lis.y) * 180 / 3.14159265 + 180;
    float theta = atan2(accel.acceleration.x, accel.acceleration.y) * 180 / 3.14159265 + 180;
    //int ledIndex = NUMPIXELS - round(theta / 360 * 288);
    int ledIndex = 20 - round(theta / 360 * 20);

    /* strip.clear();
    setPixelColor(endPointIndexes[ledIndex], strip.Color(0, 255, 0)); */
    lightningStrike(ledIndex);
    //strip.show();
    delay(25);
}


/**
* Accelerometer Setup
*/
void setupAccelerometer() {
    #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
      // ESP32 is kinda odd in that secondary I2C ports must be manually assigned their pins with setPins()!
      Wire1.setPins(SDA1, SCL1);
      if (!dso32.begin_I2C(0x6A, &Wire1)) {
        Serial.println("Failed to find LSM6DSO32 chip");
        //while (1) { delay(10); }
      }
    #else
      if (!dso32.begin_I2C(0x6A, &Wire)) {
        Serial.println("Failed to find LSM6DSO32 chip");
        //while (1) { delay(10); }
      }
    #endif

    /* if (!dso32.begin_I2C(0x6A, &Wire1)) {
      Serial.println("Failed to find LSM6DSO32 chip");
      //while (1) { delay(10); }
    } */
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
    #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
      // ESP32 is kinda odd in that secondary I2C ports must be manually assigned their pins with setPins()!
      if (tsl.begin(&Wire1, 0x29))  {
              Serial.println(F("Found a TSL2591 sensor"));
          } else {
              Serial.println(F("No sensor found ... check your wiring?"));
          }
    #else
      if (tsl.begin(&Wire, 0x29))  {
              Serial.println(F("Found a TSL2591 sensor"));
          } else {
              Serial.println(F("No sensor found ... check your wiring?"));
          }
    #endif


/*
    if (tsl.begin(&Wire1, 0x29))  {
        Serial.println(F("Found a TSL2591 sensor"));
    } else {
        Serial.println(F("No sensor found ... check your wiring?"));
    }
 */

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

void setupNodes() {
    for (int i = 0; i < NUMPIXELS; i++) {
        nodes[i].index = i;
        nodes[i].parentIndex = i - 1;
        //nodes[i].childIndexes = [i + 1, -1];
        nodes[i].isEndpoint = false;
        nodes[i].isBranchStart = false;
    }

    /* for (int i = 0; i < endPointIndexesLength; i++) {
        int pixelIndex = endPointIndexes[i];
        nodes[pixelIndex].isEndpoint = true;
        nodes[pixelIndex].childIndexes[0] = -1;
        nodes[pixelIndex].childIndexes[1] = -1;
    } */
}

void loadBranchesFile() {
    //const char *jsonFilename = "./branches.json";
    //File jsonFile = SD.open(jsonFilename);

    //const char *jsonFile = "{\"nodes\":[{\"index\":6,\"parentIndex\":5,\"childIndexes\":[7,70]},{\"index\":32,\"parentIndex\":31,\"childIndexes\":[33,48]},{\"index\":85,\"parentIndex\":84,\"childIndexes\":[86,91]},{\"index\":90,\"parentIndex\":89,\"childIndexes\":[136,152]},{\"index\":103,\"parentIndex\":102,\"childIndexes\":[104,123]}]}";
    //const char *jsonFile = "{\"nodes\":[{\"index\":6,\"parentIndex\":5,\"childIndexes\":[7,70]},{\"index\":7,\"parentIndex\":6,\"isBranchStart\":true},{\"index\":32,\"parentIndex\":31,\"childIndexes\":[33,48]},{\"index\":33,\"parentIndex\":32,\"isBranchStart\":true},{\"index\":47,\"parentIndex\":46,\"isEndpoint\":true},{\"index\":48,\"parentIndex\":32,\"isBranchStart\":true},{\"index\":69,\"parentIndex\":46,\"isEndpoint\":true},{\"index\":70,\"parentIndex\":6,\"isBranchStart\":true},{\"index\":85,\"parentIndex\":84,\"childIndexes\":[86,91]},{\"index\":90,\"parentIndex\":89,\"childIndexes\":[136,152]},{\"index\":103,\"parentIndex\":102,\"childIndexes\":[104,123]},{\"index\":122,\"isEndpoint\":true},{\"index\":135,\"isEndpoint\":true},{\"index\":151,\"isEndpoint\":true},{\"index\":165,\"isEndpoint\":true},{\"index\":201,\"isEndpoint\":true},{\"index\":213,\"isEndpoint\":true},{\"index\":246,\"isEndpoint\":true},{\"index\":257,\"isEndpoint\":true},{\"index\":301,\"isEndpoint\":true},{\"index\":314,\"isEndpoint\":true},{\"index\":348,\"isEndpoint\":true},{\"index\":364,\"isEndpoint\":true},{\"index\":399,\"isEndpoint\":true},{\"index\":415,\"isEndpoint\":true},{\"index\":446,\"isEndpoint\":true},{\"index\":463,\"isEndpoint\":true},{\"index\":492,\"isEndpoint\":true},{\"index\":512,\"isEndpoint\":true}]}";

    char jsonFile[] = "{\"nodes\":[{\"index\":6,\"childIndexes\":[7,70]},{\"index\":7,\"isBranchStart\":true},{\"index\":32,\"childIndexes\":[33,48]},{\"index\":33,\"isBranchStart\":true},{\"index\":47,\"isEndpoint\":true},{\"index\":48,\"parentIndex\":32,\"isBranchStart\":true},{\"index\":69,\"isEndpoint\":true},{\"index\":70,\"parentIndex\":6,\"isBranchStart\":true},{\"index\":85,\"childIndexes\":[86,91]},{\"index\":90,\"childIndexes\":[136,152]},{\"index\":91,\"parentIndex\":85,\"isBranchStart\":true},{\"index\":103,\"childIndexes\":[104,123]},{\"index\":122,\"isEndpoint\":true},{\"index\":123,\"parentIndex\":103,\"isBranchStart\":true},{\"index\":135,\"isEndpoint\":true},{\"index\":136,\"parentIndex\":90,\"isBranchStart\":true},{\"index\":151,\"isEndpoint\":true},{\"index\":152,\"parentIndex\":90,\"isBranchStart\":true},{\"index\":165,\"isEndpoint\":true},{\"index\":166,\"parentIndex\":2},{\"index\":169,\"parentIndex\":168,\"isBranchStart\":true},{\"index\":188,\"parentIndex\":187,\"isBranchStart\":true},{\"index\":201,\"isEndpoint\":true},{\"index\":202,\"parentIndex\":187,\"isBranchStart\":true},{\"index\":213,\"isEndpoint\":true},{\"index\":214,\"parentIndex\":168,\"isBranchStart\":true},{\"index\":236,\"parentIndex\":235,\"isBranchStart\":true},{\"index\":246,\"isEndpoint\":true},{\"index\":247,\"parentIndex\":235,\"isBranchStart\":true},{\"index\":257,\"isEndpoint\":true},{\"index\":258,\"parentIndex\":2},{\"index\":268,\"parentIndex\":267,\"isBranchStart\":true},{\"index\":294,\"parentIndex\":293,\"isBranchStart\":true},{\"index\":301,\"isEndpoint\":true},{\"index\":302,\"parentIndex\":293,\"isBranchStart\":true},{\"index\":314,\"isEndpoint\":true},{\"index\":315,\"parentIndex\":271,\"isBranchStart\":true},{\"index\":332,\"parentIndex\":331,\"isBranchStart\":true},{\"index\":348,\"isEndpoint\":true},{\"index\":349,\"parentIndex\":331,\"isBranchStart\":true},{\"index\":364,\"isEndpoint\":true},{\"index\":365,\"parentIndex\":267,\"isBranchStart\":true},{\"index\":382,\"parentIndex\":381,\"isBranchStart\":true},{\"index\":399,\"isEndpoint\":true},{\"index\":400,\"parentIndex\":381,\"isBranchStart\":true},{\"index\":415,\"isEndpoint\":true},{\"index\":416,\"parentIndex\":2},{\"index\":424,\"parentIndex\":423,\"isBranchStart\":true},{\"index\":431,\"parentIndex\":430,\"isBranchStart\":true},{\"index\":446,\"isEndpoint\":true},{\"index\":447,\"parentIndex\":430,\"isBranchStart\":true},{\"index\":463,\"isEndpoint\":true},{\"index\":464,\"parentIndex\":423,\"isBranchStart\":true},{\"index\":477,\"parentIndex\":476,\"isBranchStart\":true},{\"index\":492,\"isEndpoint\":true},{\"index\":493,\"parentIndex\":476,\"isBranchStart\":true},{\"index\":512,\"isEndpoint\":true}]}";

    //StaticJsonDocument<2048> doc;
    StaticJsonDocument<4096> doc;

    DeserializationError error = deserializeJson(doc, jsonFile);
    if (error) {
        Serial.println(F("Failed to read file, using default configuration"));
    }

    setupNodes();

    int i = 0;
    while (doc["nodes"][i]) {
        int index = doc["nodes"][i]["index"];
        //nodes[i].index =

        //if (doc["nodes"][i]["childIndexes"][0] > -1) {
            nodes[index].childIndexes[0] = doc["nodes"][i]["childIndexes"][0];
        //}

        //if (doc["nodes"][i]["childIndexes"][1] > -1) {
            nodes[index].childIndexes[1] = doc["nodes"][i]["childIndexes"][1];
        //}

        //if (doc["nodes"][i]["parentIndex"] > -1) {
            nodes[index].parentIndex = doc["nodes"][i]["parentIndex"];
            if (nodes[index].parentIndex <= 0) {
                nodes[index].parentIndex = nodes[index].index - 1;
            }
            Serial.print("Setting parentIndex to ");
            Serial.println(nodes[index].parentIndex);
        //} else {
            //nodes[index].parentIndex = index - 1;
        //}

        //if (doc["nodes"][i]["isEndpoint"] == true) {
            nodes[index].isEndpoint = doc["nodes"][i]["isEndpoint"];
        //}

        //if (doc["nodes"][i]["isBranchStart"] == true) {
            nodes[index].isBranchStart = doc["nodes"][i]["isBranchStart"];
        //}

        i++;
    }

    for (int x = 0; x < NUMPIXELS; x++) {
        Serial.print("x: ");
        Serial.print(x);
        Serial.print(" index: ");
        Serial.print(nodes[x].index);
        Serial.print(" parentIndex: ");
        Serial.print(nodes[x].parentIndex);
        Serial.print(" nodes[x].childIndexes[0]: ");
        Serial.print(nodes[x].childIndexes[0]);
        Serial.print(" nodes[x].childIndexes[1]: ");
        Serial.print(nodes[x].childIndexes[1]);
        Serial.print(" nodes[x].isEndpoint: ");
        Serial.print(nodes[x].isEndpoint);
        Serial.print(" nodes[x].isBranchStart: ");
        Serial.println(nodes[x].isBranchStart);
    }

    /* int i = 0;
    while (doc["nodes"][i]) {
        branches[i].index = doc["nodes"][i]["index"];
        branches[i].childIndexes[0] = doc["nodes"][i]["childIndexes"][0];
        branches[i].childIndexes[1] = doc["nodes"][i]["childIndexes"][1];
        if (doc["nodes"][i]["parentIndex"] > -1) {
            branches[i].parentIndex = doc["nodes"][i]["parentIndex"];
        } else {
             branches[i].parentIndex = branches[i].index - 1;
        }
        i++;
    } */
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
/*     for (int x = 0; x < powerStartIndexesLength && powerStartIndexes[x] <= i; x++) {
      if (powerStartIndexes[x] == i) {
        strip.setPixelColor(i, 0, 255, 0);
      }
    } */
    if (nodes[i].parentIndex == -1 || nodes[i].parentIndex == 2) {
        strip.setPixelColor(i, 0, 255, 0);
    }
    /* for (int x = 0; x < centerNodeIndexesLength && centerNodeIndexes[x] <= i; x++) {
      if (centerNodeIndexes[x] == i) {
        strip.setPixelColor(i, 0, 255, 0);
      }
    } */
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


void lightningStrike(int endPointArrayIndex) {
    // For random but grouped strikes...
    /* int randomIndex = random(0, endPointIndexesLength - NUMSTRIKEPATHS);
    for (int i = 0; i < NUMSTRIKEPATHS; i++) {
        pathStartIndexes[i] = endPointIndexes[randomIndex + i];
    } */

    // For doing the endpoints in rotation
    /* for (int x = 0; x < NUMSTRIKEPATHS; x++) {
        if (x < NUMSTRIKEPATHS - 1) {
            pathStartIndexes[x] = pathStartIndexes[x + 1];
        } else {
            pathStartIndexes[x] = endPointIndexes[endpointThingIndex];
        }
    }
    endpointThingIndex++;
    if (endpointThingIndex >= endPointIndexesLength) {
        endpointThingIndex = 0;
    } */

    pathStartIndexes[0] = endPointIndexes[endPointArrayIndex - 1];
    pathStartIndexes[1] = endPointIndexes[endPointArrayIndex];
    pathStartIndexes[2] = endPointIndexes[endPointArrayIndex + 1];




    int strikePaths[NUMSTRIKEPATHS][NUMPIXELS];
    int pathLengths[NUMSTRIKEPATHS];

    // Initialize all the strike path indexes to -1
    for (int x = 0; x < NUMSTRIKEPATHS; x++) {
        for (int y = 0; y < NUMPIXELS; y++) {
            strikePaths[x][y] = -1;
        }
    }

    // Determine the path from node 0 to the endpoint given
    for (int currStrike = 0; currStrike < NUMSTRIKEPATHS; currStrike++) {
        int x = pathStartIndexes[currStrike];
        int numNodesInPath = 0;

        // Keep moving along the path by setting x to the parentIndex.
        // TODO: FIX POTENTIAL INFINITE LOOP CAUSED BY CIRCULAR REFERENCE! (ex: 3 -> 2 -> 1 -> 3...)
        while (x >= 0) {
            strikePaths[currStrike][numNodesInPath] = nodes[x].index;
            numNodesInPath++;
            x = nodes[x].parentIndex;
        }
        numNodesInPath--; // For some reason, this gets incremented an extra time. I can't be bothered to figure out why, so enjoy this lovely hack. -JD
        pathLengths[currStrike] = numNodesInPath;
    }

    // Reverse the strikePath arrays
    // We only care about the first  numNodesInPath elements of the array, and we only need to iterate over half the values
    // So we stop after half the nodes in the path (the other elements should all be -1)
    for (int x = 0; x < NUMSTRIKEPATHS; x++) {
        int numNodesInPath = pathLengths[x];
        for (int y = 0; y < numNodesInPath; y++) {
            if (numNodesInPath / 2 <= y) {
                break;
            }
            int tmp = strikePaths[x][y];
            strikePaths[x][y] = strikePaths[x][numNodesInPath - y];
            strikePaths[x][numNodesInPath - y] = tmp;

            //std::cout << strikePaths[x][y] << ", ";
        }
    }



    //Serial.println("Setting pixel colors");
    for (int z = 0; z < NUMPIXELS; z++) {
        bool isEndOfPath = true;
        for (int x = 0; x < NUMSTRIKEPATHS; x++) {
            if (strikePaths[x][z] > -1) {
                strip.setPixelColor(strikePaths[x][z], 255, 255, 255);
                isEndOfPath = false;
            }
        }
        if (z % 10 == 0) {
        //if (z % 1 == 0) {
            strip.show();
        }
        //if (z > NUMPIXELS) {
        if (isEndOfPath) {
            break;
        }
        /* if (strikePath1[z] == -1 && strikePath2[z] == -1 && strikePath3[z] == -1) {
            //Serial.println("Ducking out early!");
            break;
        } */
    }
    strip.show();

/*     delay(200);
    strip.clear();
    strip.show(); */


    //for (int b = 40; b > 12; b = b - 8) {
    for (int b = 40; b > 12; b = b - 10) {
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

    strip.setBrightness(1);
    strip.show();
    delay(50 * duration);

    strip.setBrightness(100 * intensity);
    strip.show();
    delay(150 * duration);

    //      for (int b = 75; b > 12; b = b - 8) {
    for (int b = 100; b > 12; b = b - 10) {
        strip.setBrightness(b * intensity);
        strip.show();
        delay(1);
    }

    strip.setBrightness(1);
    strip.show();
    delay(1);

    strip.clear();
    strip.show();
    delay(200);
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