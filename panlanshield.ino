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
#define ENABLESERIAL true
#define NUMSTRIKEPATHS 3
#define PIXELS_PER_STRIKE_STEP 3

// The minimum amount of time in ms to wait between fetching the current luminosity reading
#define LUMINOSITY_POLLING_INTERVAL 500
// THe number of historical luminosity readings to cache for rolling average calculation
#define LUMINOSITY_HISTORY_LENGTH 10

// The number of LEDs to light up as stars for the constellation mode
#define NUM_CONSTELLATION_STARS 20

/**
* BUTTON CONFIG
**/
#define BUTTON1_PIN A2
#define BUTTON2_PIN A4

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

volatile int button1State = 0;
volatile int button2State = 0;

/*
0 = config
1 = strikes
2 = constellations
 3 = wheel
 4 = random strikes
*/
int currentFunction = 1;


struct ConstellationStar {
    int index;
    uint16_t hue = 0;
    int saturation = 0;
    int value = 0;
};

ConstellationStar constellations[NUM_CONSTELLATION_STARS];

void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    //#ifdef ENABLESERIAL
    Serial.begin(9600);
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
    setBrightness(255);

    /**
    * Button Interrupts
    **/
    pinMode(BUTTON1_PIN, INPUT);
    pinMode(BUTTON2_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), handleButtonPress, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), handleButtonPress, CHANGE);

    for (int i = 0; i < NUM_CONSTELLATION_STARS; i++) {
        constellations[i].index = i;
        constellations[i].hue = 0;
        constellations[i].saturation = 0;
        constellations[i].value = 0;
    }

}

float intensity = 1;
float duration = .5;
int pathStartIndexes[20];

uint16_t wheelColor = 0;
float luminosity = 0;
unsigned long lastLuminosityPollTime = 0;
double luminosityHistory[LUMINOSITY_HISTORY_LENGTH] = {0};

/**
 * SetBrightness
 */
void setBrightness (int brightness, int minimum = 0) {
    strip.setBrightness(max(minimum, strip.gamma8(brightness * intensity)));
}


/**
 * setPixelColor
 */
// TODO: Update setPixelColor to use mod math rollover.
// TODO: Update setPixelColor to use the dotstar::getPixels function (to write directly to buffer)
void setPixelColor (int index, uint8_t red, uint8_t green, uint8_t blue) {
    setPixelColor(index, strip.Color(red, green, blue));
}

void setPixelColor (int index, uint32_t color) {
    strip.setPixelColor((index + NUMPIXELS) % NUMPIXELS, strip.gamma32(color));
    return;
}


void loop() {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    dso32.getEvent(&accel, &gyro, &temp);
    //printAccelData(accel, gyro, temp);
//
//    //intensity = (accel.acceleration.y + 10) / 20;
//

    ///* Get the light sensor reading */
    if (lastLuminosityPollTime + LUMINOSITY_POLLING_INTERVAL < millis()) {
      unsigned double luminositySum = 0;
      for (int x = LUMINOSITY_HISTORY_LENGTH - 1; x > 0; x++) {
          luminosityHistory[x] = luminosityHistory[x - 1];
          luminositySum += luminosityHistory[x];
      }
      luminosityHistory[0] = tsl.getLuminosity(TSL2591_VISIBLE);
      luminositySum += luminosityHistory[0];
      luminosity = (int) (luminositySum / LUMINOSITY_HISTORY_LENGTH);

      Serial.print("luminosity: "); Serial.println(luminosity);
      lastLuminosityPollTime = millis();
    }
    luminosity = constrain(luminosity, 500, 30000);
    intensity = luminosity / 25000;
    if (intensity > 1.0) {
        intensity = 1.0;
    } else if (intensity < 0.1) {
        intensity = 0.1;
    }

    Serial.print("Intensity: "); Serial.println(intensity);

    if (button1State == HIGH) {
        currentFunction = (currentFunction + 1) % 5;
        strip.clear();
        strip.setBrightness(255);
        strip.show();
        button1State = LOW;
    }
    if (button2State == HIGH) {
        currentFunction = 2;
        strip.clear();
        strip.setBrightness(255);
        strip.show();
        button2State = LOW;
    }

    // We need to do the magnitude
    float theta = atan2(accel.acceleration.x, accel.acceleration.y) * 180 / 3.14159265 + 180;
    // There are 20 possible endpoints
    //int ledIndex = 20 - round(theta / 360 * 20);
    int ledIndex = endPointIndexesLength - round(theta / 360 * endPointIndexesLength);

    int groups = NUMPIXELS / NUM_CONSTELLATION_STARS;
    int numInGroup = NUMPIXELS / groups;

    switch (currentFunction) {
        case 1: // Accelerometer-determined lightning strike
            intensity = 1;
            lightningStrike(ledIndex, false, NUMSTRIKEPATHS);
            delay(1000);
            break;
        case 2: // Starfield]
            for (int i = 0; i < NUM_CONSTELLATION_STARS; i++) {
                constellations[i].value = constellations[i].value - (random(0, 5) - 1);
                if (constellations[i].value <= 16) {
                    strip.setPixelColor(constellations[i].index, 0, 0, 0);

                    constellations[i].index = i * groups + random(0, numInGroup);
                    constellations[i].value = random(30, 255 * intensity);
                    constellations[i].hue = random(0, 65535);

                    //For random colors, set saturation = 255, for white, saturation = 0
                    constellations[i].saturation = 0;
                }
                // Commented for setPixelColor testing
                //strip.setPixelColor(constellations[i].index, strip.ColorHSV(constellations[i].hue, constellations[i].saturation, constellations[i].value));
                setPixelColor(constellations[i].index, strip.ColorHSV(constellations[i].hue, constellations[i].saturation, constellations[i].value));
            }
            strip.show();
            delay(50);
            break;
        case 3: // Color Wheel
            setBrightness(255);
            for (int i = 0; i < NUMPIXELS; i++) {
                //strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(wheelColor, 255, 255 * intensity)));
                setPixelColor(i, strip.ColorHSV(wheelColor, 255, 255));
                wheelColor = (wheelColor + 100) % 65534;
                if (i % 10 == 0) {
                    strip.show();
                }
            }
            strip.show();
            break;
        case 4: //Random lightning strike
           // intensity = 1;
            setBrightness(255);
            //duration = random(1, 150) / 100;
            duration = 1;
            lightningStrike(ledIndex, true, random(2,5));
            delay(random(1, 1000));
            break;
        case 5: //Pixel index test
            pixelIndexTest();
            break;
        default:
            break;
    }
}


/**
* Interrupts Handler
**/
void handleButtonPress() {
    button1State = digitalRead(BUTTON1_PIN);
    button2State = digitalRead(BUTTON2_PIN);
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
    //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
    tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

    // Changing the integration time gives you a longer time over which to sense light
    // longer timelines are slower, but are good in very low light situtations!
    tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);

    // JD - Default / Standard
    //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);

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
        nodes[i].isEndpoint = false;
        nodes[i].isBranchStart = false;
    }
}

void loadBranchesFile() {
    char jsonFile[] = "{\"nodes\":[{\"index\":6,\"childIndexes\":[7,70]},{\"index\":7,\"isBranchStart\":true},{\"index\":32,\"childIndexes\":[33,48]},{\"index\":33,\"isBranchStart\":true},{\"index\":47,\"isEndpoint\":true},{\"index\":48,\"parentIndex\":32,\"isBranchStart\":true},{\"index\":69,\"isEndpoint\":true},{\"index\":70,\"parentIndex\":6,\"isBranchStart\":true},{\"index\":85,\"childIndexes\":[86,91]},{\"index\":90,\"childIndexes\":[136,152]},{\"index\":91,\"parentIndex\":85,\"isBranchStart\":true},{\"index\":103,\"childIndexes\":[104,123]},{\"index\":122,\"isEndpoint\":true},{\"index\":123,\"parentIndex\":103,\"isBranchStart\":true},{\"index\":135,\"isEndpoint\":true},{\"index\":136,\"parentIndex\":90,\"isBranchStart\":true},{\"index\":151,\"isEndpoint\":true},{\"index\":152,\"parentIndex\":90,\"isBranchStart\":true},{\"index\":165,\"isEndpoint\":true},{\"index\":166,\"parentIndex\":2},{\"index\":169,\"parentIndex\":168,\"isBranchStart\":true},{\"index\":188,\"parentIndex\":187,\"isBranchStart\":true},{\"index\":201,\"isEndpoint\":true},{\"index\":202,\"parentIndex\":187,\"isBranchStart\":true},{\"index\":213,\"isEndpoint\":true},{\"index\":214,\"parentIndex\":168,\"isBranchStart\":true},{\"index\":236,\"parentIndex\":235,\"isBranchStart\":true},{\"index\":246,\"isEndpoint\":true},{\"index\":247,\"parentIndex\":235,\"isBranchStart\":true},{\"index\":257,\"isEndpoint\":true},{\"index\":258,\"parentIndex\":2},{\"index\":268,\"parentIndex\":267,\"isBranchStart\":true},{\"index\":294,\"parentIndex\":293,\"isBranchStart\":true},{\"index\":301,\"isEndpoint\":true},{\"index\":302,\"parentIndex\":293,\"isBranchStart\":true},{\"index\":314,\"isEndpoint\":true},{\"index\":315,\"parentIndex\":271,\"isBranchStart\":true},{\"index\":332,\"parentIndex\":331,\"isBranchStart\":true},{\"index\":348,\"isEndpoint\":true},{\"index\":349,\"parentIndex\":331,\"isBranchStart\":true},{\"index\":364,\"isEndpoint\":true},{\"index\":365,\"parentIndex\":267,\"isBranchStart\":true},{\"index\":382,\"parentIndex\":381,\"isBranchStart\":true},{\"index\":399,\"isEndpoint\":true},{\"index\":400,\"parentIndex\":381,\"isBranchStart\":true},{\"index\":415,\"isEndpoint\":true},{\"index\":416,\"parentIndex\":2},{\"index\":424,\"parentIndex\":423,\"isBranchStart\":true},{\"index\":431,\"parentIndex\":430,\"isBranchStart\":true},{\"index\":446,\"isEndpoint\":true},{\"index\":447,\"parentIndex\":430,\"isBranchStart\":true},{\"index\":463,\"isEndpoint\":true},{\"index\":464,\"parentIndex\":423,\"isBranchStart\":true},{\"index\":477,\"parentIndex\":476,\"isBranchStart\":true},{\"index\":492,\"isEndpoint\":true},{\"index\":493,\"parentIndex\":476,\"isBranchStart\":true},{\"index\":512,\"isEndpoint\":true}]}";

    StaticJsonDocument<4096> doc;

    DeserializationError error = deserializeJson(doc, jsonFile);
    if (error) {
        Serial.println(F("Failed to read file, using default configuration"));
    }

    setupNodes();

    int i = 0;
    while (doc["nodes"][i]) {
        int index = doc["nodes"][i]["index"];
        nodes[index].childIndexes[0] = doc["nodes"][i]["childIndexes"][0];
        nodes[index].childIndexes[1] = doc["nodes"][i]["childIndexes"][1];
        nodes[index].parentIndex = doc["nodes"][i]["parentIndex"];
        if (nodes[index].parentIndex <= 0) {
            nodes[index].parentIndex = nodes[index].index - 1;
        }
        Serial.print("Setting parentIndex to ");
        Serial.println(nodes[index].parentIndex);

        nodes[index].isEndpoint = doc["nodes"][i]["isEndpoint"];
        nodes[index].isBranchStart = doc["nodes"][i]["isBranchStart"];

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


void lightningStrike(int endPointArrayIndex, bool randomStrikes, int numStrikePaths) {
    int strikePaths[numStrikePaths][NUMPIXELS];
    int pathLengths[numStrikePaths];

    // Determine the end points for the strikes
    if (randomStrikes) {
        int randomIndex = random(0, endPointIndexesLength - numStrikePaths);
 
        for (int i = 0; i < numStrikePaths; i++) {
            pathStartIndexes[i] = endPointIndexes[randomIndex + i];
        }
    } else {
        int firstArrayIndex = (int) (endPointArrayIndex - (numStrikePaths / 2));
        for (int i = 0; i < numStrikePaths; i++) {
          // We do 'i + endPointIndexesLength' to prevent the result of modulus being negative
          pathStartIndexes[i] = endPointIndexes[(firstArrayIndex + i + endPointIndexesLength) % endPointIndexesLength];
        }
    }

    // Initialize all the strike path indexes to -1
    for (int x = 0; x < numStrikePaths; x++) {
        for (int y = 0; y < NUMPIXELS; y++) {
            strikePaths[x][y] = -1;
        }
    }

    // Determine the path from endpoint given to node 0
    for (int currStrike = 0; currStrike < numStrikePaths; currStrike++) {
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
    for (int x = 0; x < numStrikePaths; x++) {
        int numNodesInPath = pathLengths[x];
        for (int y = 0; y < numNodesInPath; y++) {
            if (numNodesInPath / 2 <= y) {
                break;
            }
            int tmp = strikePaths[x][y];
            strikePaths[x][y] = strikePaths[x][numNodesInPath - y];
            strikePaths[x][numNodesInPath - y] = tmp;
        }
    }

    //strip.setBrightness(min(25, intensity * 255));
    setBrightness(255, 25);

    //Serial.println("Setting pixel colors");
    for (int z = 0; z < NUMPIXELS; z++) {
        bool isEndOfPath = true;
        for (int x = 0; x < numStrikePaths; x++) {
            if (strikePaths[x][z] > -1) {
                setPixelColor(strikePaths[x][z], 255, 255, 255);
                isEndOfPath = false;
            }
        }
        if (z % PIXELS_PER_STRIKE_STEP == 0) {
            //delay(2);
            strip.show();            
        }

        if (isEndOfPath) {
            break;
        }
    }
    strip.show();

    for (int b = 40; b > 12; b = b - 10) {
        //strip.setBrightness(b * intensity);
        setBrightness(b);
        strip.show();
        delay(20 * duration);
    }

    //setBrightness(100);
    setBrightness(255);
    strip.show();
    delay(10 * duration);

    setBrightness(6);
    strip.show();
    delay(50 * duration);

    setBrightness(1);
    strip.show();
    delay(50 * duration);

    //setBrightness(100);
    setBrightness(255);
    strip.show();
    delay(150 * duration);

    //      for (int b = 75; b > 12; b = b - 8) {
    for (int b = 100; b > 12; b = b - 10) {
        setBrightness(b);
        strip.show();
        delay(1);
    }

    setBrightness(1);
    strip.show();
    delay(1);

    strip.clear();
    strip.show();
    //delay(200);
}
