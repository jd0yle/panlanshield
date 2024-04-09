#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_DotStar.h>
#include "Adafruit_TSL2591.h"
#include <ArduinoJson.h>
#include <Arduino.h>
//#include "BranchNode.h"


/**
 * TODOS
 * - Add capacitor to battery voltage reader
 * - Low Power mode (https://github.com/arduino-libraries/ArduinoLowPower/blob/master/examples/ExternalWakeup/ExternalWakeup.ino)
 * - Direct write to dotstar memory buffer (faster than strip.show())
 * - Calibrate light sensor vs intensity
 * - Battery percentage indicator
 * - Settings/config menu
 *     - Select mode/effect
 *     - Set brightness/intensity
 * - Add large capacitor to battery output?
 */

/**
* CONFIG
**/
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define ENABLESERIAL true
//#define ENABLESERIAL false
#define DEBUG_PATH false
#define DEBUG_LIGHTNING false
#define DEBUG_LUMINOSITY false
#define DEBUG_VOLTAGE false
#define DEBUG_BUTTONS true

#define BATTERY_VOLTAGE_MAX 8.4
#define BATTERY_VOLTAGE_MIN 6.4

#define NUMSTRIKEPATHS 3
#define PIXELS_PER_STRIKE_STEP 3
#define RANDOM_STRIKE_INTERVAL_MIN 1000
#define RANDOM_STRIKE_INTERVAL_MAX 20000

#define MAX_PATH_LENGTH 54

// The minimum amount of time in ms to wait between fetching the current luminosity reading
#define LUMINOSITY_POLLING_INTERVAL 500
//#define LUMINOSITY_POLLING_INTERVAL 60000
// THe number of historical luminosity readings to cache for rolling average calculation
#define LUMINOSITY_HISTORY_LENGTH 5

#define VOLTAGE_POLLING_INTERVAL 100
// Number of voltage readings to keep to determine rolling average of battery state
#define VOLTAGE_HISTORY_LENGTH 100

// The number of LEDs to light up as stars for the constellation mode
#define NUM_CONSTELLATION_STARS 20

/**
* BUTTON CONFIG
**/
#define BUTTON1_PIN A2
#define BUTTON2_PIN A1

/**
* LIGHT SENSOR
**/
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

/**
* LSM6DS032 Accelerometer, on secondary I2C bus
* (ESP32 has 2 I2C busses, one for the board pins and the second for the STEMMA port)
**/
/**
* ACCELEROMETER
**/
#define DEFAULT_I2C_PORT &Wire
#define SECONDARY_I2C_PORT &Wire1

#define ACCELEROMETER_INTERRUPT_PIN 6
//#define CLICKTHRESHHOLD 8
#define CLICKTHRESHHOLD 9

//Adafruit_LIS3DH lis = Adafruit_LIS3DH();
Adafruit_LIS3DH lis = Adafruit_LIS3DH(SECONDARY_I2C_PORT);

/**
* DOTSTAR STRIP
* Using SPI for Dotstar; doesn't need number (uses SCL and MOSI)
**/
#define NUMPIXELS 522
Adafruit_DotStar strip(NUMPIXELS, DOTSTAR_BGR);

/**
 * Onboard NEOPIXEL config
 */
//#define NEOPIXEL_LED_PIN 8
// Declare our NeoPixel strip object:
Adafruit_NeoPixel builtinNeopixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

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

int endPointIndexes[] = {122, 135, 151, 165, 201, 213, 246, 257, 301, 314, 348, 364, 399, 415, 446, 463, 492, 512, 47, 69};
int endPointIndexesLength = sizeof endPointIndexes / sizeof endPointIndexes[0];

int midEndIndexes[] = {7, 48, 70, 91, 123, 136, 152, 166, 169, 188, 202, 214, 236, 247, 258, 268, 294, 302, 315, 332, 349, 365, 400, 416, 424, 447, 464, 477, 493};
int midEndIndexesLength = sizeof midEndIndexes / sizeof midEndIndexes[0];

int powerStartIndexes[] = {0, 70, 166, 258, 365, 416};
int powerStartIndexesLength = sizeof powerStartIndexes / sizeof powerStartIndexes[0];

int centerNodeIndexes[] = {0, 166};
int centerNodeIndexesLength = sizeof centerNodeIndexes / sizeof centerNodeIndexes[0];

Node nodes[NUMPIXELS];

struct StrikePath {
    int endIndex;
    int path[MAX_PATH_LENGTH];
    int pathLength;
};

StrikePath strikePaths[20];

volatile int button1State = 0;
volatile int button2State = 0;
volatile bool tapInterrupt = false;
bool newTapEvent = false;

/*
0 = strikes
1 = constellations
2 = random strikes
4 = wheel
*/
//int currentFunction = 0;
int currentFunction = 6;
#define NUM_FUNCTIONS 3


struct ConstellationStar {
    int index;
    uint16_t hue = 0;
    int saturation = 0;
    int value = 0;
};

ConstellationStar constellations[NUM_CONSTELLATION_STARS];

float batteryPercent = 1.0;

void setup() {
    //pinMode(13, OUTPUT);
    //digitalWrite(13, HIGH);

    if (ENABLESERIAL) {
        //Serial.begin(9600);
        Serial.begin(115200);
    }

    /*digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
    digitalWrite(13, HIGH);*/
    builtinNeopixel.begin();
    builtinNeopixel.setPixelColor(0, 255, 0, 0);
    builtinNeopixel.show();
    delay(100);
    builtinNeopixel.setPixelColor(0, 196, 64, 0);
    builtinNeopixel.show();
    delay(100);
    builtinNeopixel.setPixelColor(0, 255, 255, 0);
    builtinNeopixel.show();
    delay(100);
    builtinNeopixel.setPixelColor(0, 0, 255, 0);
    builtinNeopixel.show();
    delay(100);
    builtinNeopixel.setPixelColor(0, 0, 0, 255);
    builtinNeopixel.show();
    delay(100);
    builtinNeopixel.setPixelColor(0, 255, 0, 255);
    builtinNeopixel.show();
    delay(300);

    if (ENABLESERIAL) {
        Serial.println("Starting up...");
        delay(500);
    }

    loadBranchesFile();

    /**
     * Dotstar Strip Setup
     */
    strip.begin(); // Initialize pins for output
    strip.clear();
    strip.show();    // Turn all LEDs off ASAP
    strip.setBrightness(255);



    /**
     *  Accelerometer init
     */
    //setupAccelerometer();
    if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
        while (1) {
            if (ENABLESERIAL) {
                Serial.println("COULD NOT START: LIS3DH NOT FOUND!");
            }
            digitalWrite(13, HIGH);
            builtinNeopixel.setPixelColor(0, 255, 0, 0);
            builtinNeopixel.show();
            delay(500);
            digitalWrite(13, LOW);
            builtinNeopixel.setPixelColor(0, 144, 32, 2);
            builtinNeopixel.show();
            delay(200);
        }
    }
    if (ENABLESERIAL) {
        Serial.println("LIS3DH found!");
    }
    lis.setRange(LIS3DH_RANGE_16_G);
    // c, threashhold, timelimit 10, timelatency(doubleclikc), timewindow
    //lis.setClick(1, CLICKTHRESHHOLD, 5, 20, 500);
    lis.setClick(1, CLICKTHRESHHOLD, 5, 20, 255);

    /**
     *  Light Sensor init
     */
    setupLightSensor();


    /**
    * Button Interrupts init
    **/
    pinMode(BUTTON1_PIN, INPUT);
    pinMode(BUTTON2_PIN, INPUT);
    //attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), handleButtonPress, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), handleButtonPress, CHANGE);

    /**
     * Accelerometer Click Detection Interrupt
     */
    //pinMode(ACCELEROMETER_INTERRUPT_PIN, INPUT);
    //attachInterrupt(digitalPinToInterrupt(ACCELEROMETER_INTERRUPT_PIN), handleTapEvent, HIGH);

    /**
     * Constellation nodes init
     */
    for (int i = 0; i < NUM_CONSTELLATION_STARS; i++) {
        constellations[i].index = i;
        constellations[i].hue = 0;
        constellations[i].saturation = 0;
        constellations[i].value = 0;
    }

    calculateStrikePaths();

    if (DEBUG_PATH && ENABLESERIAL) {
        for (int i = 0; i < endPointIndexesLength; i++) {
            Serial.print("endIndex: ");
            Serial.print(strikePaths[i].endIndex);
            Serial.print(" pathLength: ");
            Serial.print(strikePaths[i].pathLength);
            Serial.print("  Path: ");
            for (int x = 0; x < strikePaths[i].pathLength; x++) {
                Serial.print(strikePaths[i].path[x]);
                Serial.print("; ");
            }
            Serial.println("");
        }
    }

    /**
     * Scheduler processes init
     */
    //Scheduler.startLoop(readLightSensorLoop);
    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;

    xTaskCreate( vTaskPollLuminosity, "vTaskPollLuminosity", 10000, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );
    xTaskCreate( vTaskPollAccel, "vTaskPollAccel", 10000, &ucParameterToPass, tskIDLE_PRIORITY, NULL );
    xTaskCreate( vTaskPollVoltage, "vTaskPollVoltage", 10000, &ucParameterToPass, tskIDLE_PRIORITY, NULL );
    xTaskCreate( vTaskPollButtons, "vTaskPollButtons", 10000, &ucParameterToPass, tskIDLE_PRIORITY, NULL );
}

float intensity = .01;
//float intensity = .5;
float duration = .75;
int pathStartIndexes[20];

uint16_t wheelColor = 0;
float luminosity = 0;
unsigned long lastLuminosityPollTime = 0;
unsigned long lastStarfieldUpdate = 2000;
unsigned long nextRandomStrikeTime = 2000;
unsigned long lastRadialSpokeUpdate = 2000;
double luminosityHistory[LUMINOSITY_HISTORY_LENGTH] = {0};

/**
 * SetBrightness
 */
void setBrightness (int brightness, int minimum = 0) {
    strip.setBrightness(max(minimum, (int) (brightness * intensity)));
}


/**
 * setPixelColor
 */
// TODO: Update setPixelColor to use the dotstar::getPixels function (to write directly to buffer)
void setPixelColor (int index, uint8_t red, uint8_t green, uint8_t blue) {
    if (red > 0 && red * intensity < 1) {
        red = 1;
    } else {
        red = red * intensity;
    }
    if (blue > 0 && blue * intensity < 1) {
        blue = 1;
    } else {
        blue = blue * intensity;
    }
    if (green > 0 && green * intensity < 1) {
        green = 1;
    } else {
        green = green * intensity;
    }
    setPixelColor(index, strip.Color(red, green, blue));
}

void setPixelColor (int index, uint32_t color) {
    strip.setPixelColor((index + NUMPIXELS) % NUMPIXELS, color);
    return;
}

sensors_event_t accelEvent;
sensors_event_t accelTapEvent;

float theta;
int ledIndex = 0;




int groups = NUMPIXELS / NUM_CONSTELLATION_STARS;
int numInGroup = NUMPIXELS / groups;

int starfieldColorMode = 0;

bool isTapStrikeEnabled = false;
bool isRandomStrikeEnabled = false;

bool button1Event = false;
bool button2Event = false;

long button1EmbargoTime = 0;
long button2EmbargoTime = 0;

float tapEventAccelX = 0;
float tapEventAccelY = 0;

int radialStrikeIndex = 0;

void loop() {

    //if (button1State == HIGH) {
    if (button1Event) {
        if (ENABLESERIAL) {
            Serial.println("Button 1 pushed, changing mode");
        }
        currentFunction++;
        strip.clear();
        strip.show();
        button1State = LOW;
        starfieldReset();

        uint32_t modeColor;
        switch (currentFunction) {
            case 0:
                modeColor = builtinNeopixel.Color(255, 255, 255);
                break;
            case 1:
                modeColor = builtinNeopixel.Color(0,0 , 255);
                break;
            case 2:
                modeColor = builtinNeopixel.Color(255, 0, 255);
                break;
            case 3:
                modeColor = builtinNeopixel.Color(255, 128, 64);
                break;
            case 4:
                modeColor = builtinNeopixel.Color(0, 255, 128);
                break;
            case 5:
                modeColor = builtinNeopixel.Color(32, 128, 255);
                break;
            default:
                modeColor = builtinNeopixel.Color(255, 0, 0);
                break;
        }
        for (int i = 0; i < currentFunction + 1; i++) {
            builtinNeopixel.setPixelColor(0, 0, 0, 0);
            builtinNeopixel.show();
            delay(100);
            builtinNeopixel.setPixelColor(0, modeColor);
            builtinNeopixel.show();
            delay(100);
        }
        button1Event = false;
        button1EmbargoTime = millis() + 300;
    }
    //if (button2State == HIGH) {
    if (button2Event) {
        if (ENABLESERIAL) {
            Serial.println("Button 2 pushed, reverting to default values");
        }
        currentFunction = 0;
        for (int x = 0; x < LUMINOSITY_HISTORY_LENGTH; x++) {
            luminosityHistory[x] = 1;
        }
        intensity = .01;
        strip.clear();
        strip.show();
        button2State = LOW;
        button2Event = false;
        button2EmbargoTime = millis() + 300;
    }

    if (isTapStrikeEnabled && newTapEvent && millis() > 5000) {
        if (ENABLESERIAL) {
            Serial.print("Lightning because tap event: ");
            Serial.print("accelX: ");Serial.print(tapEventAccelX);
            Serial.print("  accelY: ");Serial.print(tapEventAccelY);
        }
        //duration = 1;
        duration = random(25, 100) / 100;
        //theta = atan2(accelEvent.acceleration.x, accelEvent.acceleration.y) * 180 / 3.14159265 + 180;
        theta = atan2(tapEventAccelX, tapEventAccelY) * 180 / 3.14159265 + 180;
        ledIndex = endPointIndexesLength - round(theta / 360 * endPointIndexesLength);
        if (ENABLESERIAL) {
            Serial.print(" Theta: ");Serial.println(theta);
        }
        lightningStrike(ledIndex, false, random(2, 5));
        newTapEvent = false;
    } else if (isRandomStrikeEnabled && millis() > nextRandomStrikeTime && millis() > 5000) {
        if (ENABLESERIAL && DEBUG_LIGHTNING) {
            Serial.println("Doing random strike");
        }
        duration = random(25, 100) / 100;
        lightningStrike(ledIndex, true, random(2,5));
        if (random(1,5) != 1) {
            nextRandomStrikeTime = millis() + random(RANDOM_STRIKE_INTERVAL_MIN, RANDOM_STRIKE_INTERVAL_MAX);
        }
    }

    switch (currentFunction) {
        case 0: // Starfield with accelerometer event strike and random strike, white
            isTapStrikeEnabled = true;
            isRandomStrikeEnabled = true;
            starfieldColorMode = 0;
            starfieldUpdate();
            break;
        case 1: // Starfield with accelerometer event strike and random strike, gradient
            isTapStrikeEnabled = true;
            isRandomStrikeEnabled = false;
            starfieldColorMode = 2;
            starfieldUpdate();
            break;
        case 2: // Starfield with random strike
            isTapStrikeEnabled = false;
            isRandomStrikeEnabled = true;
            starfieldColorMode = 0;
            starfieldUpdate();
            break;
        case 3: // Accelerometer-determined lightning strike
            isTapStrikeEnabled = true;
            isRandomStrikeEnabled = false;
            break;
        case 4: //Random lightning strike
            /*isTapStrikeEnabled = false;
            isRandomStrikeEnabled = true;*/
            radialStrikes();
            break;
        case 5: //Radial Expansion
            /*isTapStrikeEnabled = true;
            isRandomStrikeEnabled = true;*/
            radialExpansion();
            break;
        case 6: // Radial spokes
            isTapStrikeEnabled = false;
            isRandomStrikeEnabled = false;
            //if (lastRadialSpokeUpdate + 50 < millis()) {
            if (lastRadialSpokeUpdate + 500 < millis()) {
                radialSpokes();
                lastRadialSpokeUpdate = millis();
            }
            break;
        case 7:
            isTapStrikeEnabled = false;
            isRandomStrikeEnabled = false;
            lightUpEveryXPixel();
            break;
        case 8:
            for (int i = 0; i < NUMPIXELS; i++) {
                setPixelColor(i, 255, 255, 255);
            }
            strip.show();
            break;
        case 9:
            lightningStrike(radialStrikeIndex, false, 1);
            radialStrikeIndex = (radialStrikeIndex + 1) % endPointIndexesLength;
            break;
        default:
            currentFunction = 0;
            break;
    }

}


void radialStrikes () {
    duration = random(10, 100) / 100;
    lightningStrike(radialStrikeIndex, false, random(2,5));
    radialStrikeIndex = (radialStrikeIndex + random(2, 5)) % endPointIndexesLength;
    /*if (radialStrikeIndex != 0) {
        lastLuminosityPollTime = millis();
    } else {
        lastLuminosityPollTime = 1;
    }*/
}

void testingThing () {
    //int interval = 65535 / NUMPIXELS;
    int interval = 32767 / NUMPIXELS;
    int color = 32767;
    for (int i = 0; i < NUMPIXELS; i++) {
        setPixelColor(i, strip.gamma32(strip.ColorHSV(color, 255, 128)));
        color += interval;
    }
    strip.show();
}

void testingThingCorrected () {
    int interval = 65535 / NUMPIXELS;
    int color = 1;
    for (int i = 0; i < NUMPIXELS; i++) {
        setPixelColor(i, strip.gamma32(strip.ColorHSV(color, 255, 128)));
        color += interval;
    }
    strip.show();
}



//endPointIndexes[]
//endPointIndexesLength
int radialSpokeIndex = 0;
/*void radialSpokes() {
    strip.clear();
    // Testing MAX_PATH_LENGTH instead of NUMPIXELS
    //int thePath[NUMPIXELS];
    int thePath[MAX_PATH_LENGTH];
    int pathLength = 0;
    pathLength = getStrikePath(endPointIndexes[radialSpokeIndex], thePath);

    for (int i = 0; i < pathLength; i++) {
        setPixelColor(thePath[i], 255, 255, 255);
    }
    strip.show();
    radialSpokeIndex++;
    if (radialSpokeIndex >= endPointIndexesLength) radialSpokeIndex = 0;

};*/

void radialSpokes() {
    strip.clear();
    for (int i = 0; i < strikePaths[radialSpokeIndex].pathLength; i++) {
        setPixelColor(strikePaths[radialSpokeIndex].path[i], 255, 255, 255);
        //setPixelColor(strikePaths[radialSpokeIndex].path[i], 0, 0, 255);
    }
    strip.show();
    //radialSpokeIndex = (radialSpokeIndex + 1 + 19) % 19;
    radialSpokeIndex = (radialSpokeIndex + 1 + endPointIndexesLength) % endPointIndexesLength;
    //radialSpokeIndex = (radialSpokeIndex + 1 + 5) % 5;
}

void radialExpansion () {
    for (int y = 0; y < MAX_PATH_LENGTH; y++){
        strip.clear();
        for (int x = 0; x < endPointIndexesLength; x++) {
            int numTails = 8;
            int b = 255 / numTails;
            for (int z = 0; z < numTails; z++) {
                if (y - z < strikePaths[x].pathLength && y - z >= 0) {
                    //setPixelColor(strikePaths[x].path[y - z], 255 - (b * z), 255 - (b * z), 255 - (b * z));
                    setPixelColor(strikePaths[x].path[y - z], strip.gamma32(strip.Color(255 - (b * z), 0, 255 - (b * z / 2))));
                }
            }
            /*if (y < strikePaths[x].pathLength) {
                setPixelColor(strikePaths[x].path[y], 255, 255, 255);
            }
            if (y > 0 && y - 1 < strikePaths[x].pathLength) {
                setPixelColor(strikePaths[x].path[y] - 1, 128, 128, 128);
            }
            if (y > 1 && y - 2 < strikePaths[x].pathLength) {
                setPixelColor(strikePaths[x].path[y] - 2, 32, 32, 32);
            }*/
        }
        strip.show();
        delay(10);
    }
}

int radialStartOffset = 0;

/*void radialExpansion() {
    int thePaths[endPointIndexesLength][MAX_PATH_LENGTH];
    int pathLengths[endPointIndexesLength];
    int maxPathLength = 0;

    for (int x = 0; x < endPointIndexesLength; x++) {
        pathLengths[x] = getStrikePath(endPointIndexes[x], thePaths[x]);
    }

    for (int i = 0; i < endPointIndexesLength; i++) {
        int index = (radialStartOffset + i + endPointIndexesLength) % endPointIndexesLength;
        Serial.print("index ");Serial.println(index);
        for (int offset = index * 2; offset > 0; offset--) {
            for (int z = pathLengths[index]; z > 0; z--) {
                thePaths[index][z] = thePaths[index][z - 1];
            }
            thePaths[index][0] = -1;
            pathLengths[index]++;
        }
        if (pathLengths[index] > maxPathLength) maxPathLength = pathLengths[index];
    }

    strip.clear();
    strip.show();

    for (int pathIndex = 0; pathIndex < maxPathLength; pathIndex++) {
        for (int spokeIndex = 0; spokeIndex < endPointIndexesLength; spokeIndex++) {
            if (pathIndex < pathLengths[spokeIndex] ) {
                setPixelColor(thePaths[spokeIndex][pathIndex], 255, 255, 255);
            }
            if (pathIndex > 0) {
                setPixelColor(thePaths[spokeIndex][pathIndex - 1], 0, 0, 0);
            }
        }
        strip.show();
        delay(2);
    }

    strip.show();
    radialStartOffset++;
    if (radialStartOffset > endPointIndexesLength - 1) radialStartOffset = 0;
}*/

int radialCurrentFirstPath = 0;

int numStrikePaths = 20;



void radiateSpiral () {
    int radialCurrentIndex = 0;
    int radialCurrentOffset = 0;
    // 20 - radialCurrentIndex >= 0
    // 20 - 0 = 20
    // 20 - 1 = 19


    int offsetFactor = 2;

    // The indexOfPathStep is the current index of the strikePaths.path[] array (ie, strikePaths[x].path[indexOfPathStep] = the NUMPIXELS index of the LED at this point in the path
    for (int indexOfPathStep = 0; indexOfPathStep < MAX_PATH_LENGTH + 40; indexOfPathStep++) {

        strip.clear();

        // The indexOfStrikePathArray holds which of the 20 strikePaths to work with
        for (int indexOfStrikePathArray = 0; indexOfStrikePathArray < numStrikePaths; indexOfStrikePathArray++) {
            //int indexWithOffset = (indexOfPathStep - (indexOfStrikePathArray * offsetFactor)) % strikePaths[indexOfStrikePathArray].pathLength;
            // indexWithOffset is the index of the path array, accounting for the offset that makes the spiral pattern occur
            int indexWithOffset = indexOfPathStep - (indexOfStrikePathArray * offsetFactor);
            if (indexWithOffset >= 0 && indexWithOffset < strikePaths[indexOfStrikePathArray].pathLength) {
                setPixelColor(strikePaths[indexOfStrikePathArray].path[indexWithOffset], 255, 255, 255);
            }
        }
        strip.show();
        delay(2);
    }
}

void lightUpEveryXPixel () {
    int index = 0;
    //int iterations = (int) strikePaths[0].pathLength / interval;
    //int numberOfPixelsToUse = (int) strikePaths[0].pathLength / interval;
    int numberOfPixelsToUse = 1;
    int interval = 50;


    //for (int startAt = 0; startAt < strikePaths[0].pathLength; startAt++) {
    //for (int startAt = -20; startAt < strikePaths[0].pathLength; startAt++) {
    for (int startAt = 0; startAt < 44; startAt++) {
        strip.clear();
        for (int stripIndex = 0; stripIndex < 20; stripIndex++) {
            index = startAt - (stripIndex * 2);

            //while (index >= 0 && index < strikePaths[stripIndex].pathLength) {
            while (index < strikePaths[stripIndex].pathLength) {
                if (index >= 0) {
                    setPixelColor(strikePaths[stripIndex].path[index], 255, 255, 255);
                }
                index += interval;
                /*for (int i = 0; i < numberOfPixelsToUse; i++) {
                    index = (interval * i + startAt - stripIndex) % strikePaths[stripIndex].pathLength;

                }*/
            }
            /*while (index < strikePaths[0].pathLength) {
                setPixelColor(strikePaths[0].path[index], 255, 255, 255);
                index += interval;
            }*/

        }
        strip.show();
        delay(20);
    }
}

void calculateStrikePaths () {
    for (int i = 0; i < endPointIndexesLength; i++) {
        strikePaths[i].endIndex = endPointIndexes[i];
        strikePaths[i].pathLength = getStrikePath(endPointIndexes[i], strikePaths[i].path);
    }
}

/**
* Interrupts Handler
**/
void handleButtonPress() {
    button1State = digitalRead(BUTTON1_PIN);
    button2State = digitalRead(BUTTON2_PIN);
}





void colorWheel() {
    for (int i = 0; i < NUMPIXELS; i++) {
        setPixelColor(i, strip.ColorHSV(wheelColor, 255, 255));
        wheelColor = (wheelColor + 100) % 65534;
        if (i % 10 == 0) {
            strip.show();
        }
    }
    strip.show();
}

uint8_t batteryLedRed = 0;
uint8_t batteryLedGreen = 0;
uint8_t batteryLedBlue = 0;

void vTaskPollVoltage ( void * pvParameters ) {
    float voltageHistory[VOLTAGE_HISTORY_LENGTH] = {0};
    float voltageAverage = 0;
    float voltageReading = 0;
    float voltageSum = 0;

    while (true) {
        voltageReading = analogRead(A3);
        voltageReading = voltageReading / 100;
        //voltageReading = voltageReading * 7.65 / .939;
        //voltageReading = voltageReading * 8.2 / .939;
        voltageReading = voltageReading * BATTERY_VOLTAGE_MAX / .939;
        if (voltageAverage == 0) {
            voltageAverage = BATTERY_VOLTAGE_MAX;//voltageReading;
        }

        /*if (abs(voltageReading - voltageAverage) > .2) {
            if (voltageReading > voltageAverage) {
                voltageReading = voltageAverage + .05;
            } else {
                voltageReading = voltageAverage - .05;
            }
        }*/
        voltageSum = 0;
        for (int i = VOLTAGE_HISTORY_LENGTH - 1; i > 0; i--) {
            voltageHistory[i] = voltageHistory[i - 1];
            if (voltageHistory[i] > 0) voltageSum += voltageHistory[i];
        }

        voltageHistory[0] = voltageReading;

        voltageSum += voltageReading;
        voltageAverage = voltageSum / VOLTAGE_HISTORY_LENGTH;

        if (voltageAverage < BATTERY_VOLTAGE_MIN) voltageAverage = BATTERY_VOLTAGE_MIN;

        batteryPercent = (voltageAverage - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN);
        if (batteryPercent > .8) {
            batteryPercent = 1.0;
        } else if (batteryPercent < .2) {
            batteryPercent = 0.0;
        }

        if (ENABLESERIAL && DEBUG_VOLTAGE) {
            Serial.print("VoltageReading: ");Serial.print(voltageReading);Serial.print("  Battery: ");Serial.print(batteryPercent);
            Serial.print(" Raw reading: ");Serial.println(analogRead(A3));
        }

        builtinNeopixel.setPixelColor(0, 255 * (1 - batteryPercent) * intensity, 255 * batteryPercent * intensity, 0);
        builtinNeopixel.show();

        vTaskDelay(10);
    }
}

/**
 * Reset baord function
 */
void(* resetFunc) (void) = 0; // create a standard reset function




void setupLightSensor(void) {
#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
    // ESP32 is kinda odd in that secondary I2C ports must be manually assigned their pins with setPins()!
      if (tsl.begin(&Wire1, 0x29))  {
          if (ENABLESERIAL) {
              Serial.println(F("Found a TSL2591 sensor"));
          }
      } else {
          if (ENABLESERIAL) {
              Serial.println(F("No sensor found ... check your wiring?"));
          }
      }
#else
    if (tsl.begin(&Wire, 0x29))  {
        if (ENABLESERIAL) {
            Serial.println(F("Found a TSL2591 sensor"));
        }
    } else {
        if (ENABLESERIAL) {
            Serial.println(F("No sensor found ... check your wiring?"));
        }
    }
#endif

    // You can change the gain on the fly, to adapt to brighter/dimmer light situations
    //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
    //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
    tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

    // Changing the integration time gives you a longer time over which to sense light
    // longer timelines are slower, but are good in very low light situtations!
    //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);

    // JD - Default / Standard
    tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);

    // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

    /* Display the gain and integration time for reference sake */
    if (ENABLESERIAL) {
        Serial.println(F("------------------------------------"));
        Serial.print(F("Gain:         "));
        tsl2591Gain_t gain = tsl.getGain();
        switch (gain) {
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
        Serial.print(F("Timing:       "));
        Serial.print((tsl.getTiming() + 1) * 100, DEC);
        Serial.println(F(" ms"));
        Serial.println(F("------------------------------------"));
        Serial.println(F(""));
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



#define STARFIELD_COLOR_INTERVAL 100
#define STARFIELD_UPDATE_INTERVAL 200
#define STARFIELD_MAX_HUE 65535
#define STARFIELD_MIN_HUE 32767
int starfieldColorIncrease = STARFIELD_COLOR_INTERVAL;
int starfieldHue = STARFIELD_MIN_HUE;

void starfieldReset() {
    for (int i = 0; i < NUM_CONSTELLATION_STARS; i++) {
        constellations[i].value = 0;
        strip.setPixelColor(constellations[i].index, 0, 0, 0);
    }
    lastStarfieldUpdate = 1;
}

void starfieldUpdate() {
    if (lastStarfieldUpdate + STARFIELD_UPDATE_INTERVAL < millis()) {
        for (int i = 0; i < NUM_CONSTELLATION_STARS; i++) {
            constellations[i].value = constellations[i].value - (random(0, 4) - 1);
            if (constellations[i].value <= 10) {
                strip.setPixelColor(constellations[i].index, 0, 0, 0);

                // To keep the stars spread out, split the LEDs into groups, only use numInGroup stars per group
                constellations[i].index = i * groups + random(0, numInGroup);
                // Start the new star at a random brightness
                constellations[i].value = random(30, 255);
                switch (starfieldColorMode) {
                    case 0: // White stars
                        constellations[i].hue = 1;
                        constellations[i].saturation = 0;
                        break;
                    case 1: // Random color stars
                        constellations[i].hue = random(1, 65535);
                        constellations[i].saturation = 0;
                        break;
                    case 2: // Slowly change star color across a gradient and back
                        constellations[i].hue = starfieldHue;
                        constellations[i].saturation = 255;

                        starfieldHue += starfieldColorIncrease;
                        if (starfieldHue > STARFIELD_MAX_HUE) {
                            starfieldHue = STARFIELD_MAX_HUE;
                            starfieldColorIncrease = STARFIELD_COLOR_INTERVAL * -1;
                        } else if (starfieldHue < STARFIELD_MIN_HUE) {
                            starfieldHue = STARFIELD_MIN_HUE;
                            starfieldColorIncrease = STARFIELD_COLOR_INTERVAL;
                        }
                        break;
                    default:
                        constellations[i].hue = 1;
                        constellations[i].saturation = 0;
                        break;
                }
            }

            // Apply gamma color correction, but only if we're not doing white pixels.
            if (constellations[i].saturation == 0) {
                setPixelColor(constellations[i].index, strip.ColorHSV(constellations[i].hue, constellations[i].saturation, max((int) (constellations[i].value * intensity), 1)));
            } else {
                setPixelColor(constellations[i].index, strip.gamma32(strip.ColorHSV(constellations[i].hue, constellations[i].saturation, max((int) (constellations[i].value * intensity), 1))));
            }

        }
        strip.show();
        lastStarfieldUpdate = millis();
    }
}

void loadBranchesFile() {
    char jsonFile[] = "{\"nodes\":[{\"index\":6,\"childIndexes\":[7,70]},{\"index\":7,\"isBranchStart\":true},{\"index\":32,\"childIndexes\":[33,48]},{\"index\":33,\"isBranchStart\":true},{\"index\":47,\"isEndpoint\":true},{\"index\":48,\"parentIndex\":32,\"isBranchStart\":true},{\"index\":69,\"isEndpoint\":true},{\"index\":70,\"parentIndex\":6,\"isBranchStart\":true},{\"index\":85,\"childIndexes\":[86,91]},{\"index\":90,\"childIndexes\":[136,152]},{\"index\":91,\"parentIndex\":85,\"isBranchStart\":true},{\"index\":103,\"childIndexes\":[104,123]},{\"index\":122,\"isEndpoint\":true},{\"index\":123,\"parentIndex\":103,\"isBranchStart\":true},{\"index\":135,\"isEndpoint\":true},{\"index\":136,\"parentIndex\":90,\"isBranchStart\":true},{\"index\":151,\"isEndpoint\":true},{\"index\":152,\"parentIndex\":90,\"isBranchStart\":true},{\"index\":165,\"isEndpoint\":true},{\"index\":166,\"parentIndex\":2},{\"index\":169,\"parentIndex\":168,\"isBranchStart\":true},{\"index\":188,\"parentIndex\":187,\"isBranchStart\":true},{\"index\":201,\"isEndpoint\":true},{\"index\":202,\"parentIndex\":187,\"isBranchStart\":true},{\"index\":213,\"isEndpoint\":true},{\"index\":214,\"parentIndex\":168,\"isBranchStart\":true},{\"index\":236,\"parentIndex\":235,\"isBranchStart\":true},{\"index\":246,\"isEndpoint\":true},{\"index\":247,\"parentIndex\":235,\"isBranchStart\":true},{\"index\":257,\"isEndpoint\":true},{\"index\":258,\"parentIndex\":2},{\"index\":268,\"parentIndex\":267,\"isBranchStart\":true},{\"index\":294,\"parentIndex\":293,\"isBranchStart\":true},{\"index\":301,\"isEndpoint\":true},{\"index\":302,\"parentIndex\":293,\"isBranchStart\":true},{\"index\":314,\"isEndpoint\":true},{\"index\":315,\"parentIndex\":271,\"isBranchStart\":true},{\"index\":332,\"parentIndex\":331,\"isBranchStart\":true},{\"index\":348,\"isEndpoint\":true},{\"index\":349,\"parentIndex\":331,\"isBranchStart\":true},{\"index\":364,\"isEndpoint\":true},{\"index\":365,\"parentIndex\":267,\"isBranchStart\":true},{\"index\":382,\"parentIndex\":381,\"isBranchStart\":true},{\"index\":399,\"isEndpoint\":true},{\"index\":400,\"parentIndex\":381,\"isBranchStart\":true},{\"index\":415,\"isEndpoint\":true},{\"index\":416,\"parentIndex\":2},{\"index\":424,\"parentIndex\":423,\"isBranchStart\":true},{\"index\":431,\"parentIndex\":430,\"isBranchStart\":true},{\"index\":446,\"isEndpoint\":true},{\"index\":447,\"parentIndex\":430,\"isBranchStart\":true},{\"index\":463,\"isEndpoint\":true},{\"index\":464,\"parentIndex\":423,\"isBranchStart\":true},{\"index\":477,\"parentIndex\":476,\"isBranchStart\":true},{\"index\":492,\"isEndpoint\":true},{\"index\":493,\"parentIndex\":476,\"isBranchStart\":true},{\"index\":512,\"isEndpoint\":true}]}";

    StaticJsonDocument<4096> doc;

    DeserializationError error = deserializeJson(doc, jsonFile);
    if (error) {
        if (ENABLESERIAL) {
            Serial.println(F("Failed to read file, using default configuration"));
        }
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
        /*if (ENABLESERIAL) {
            Serial.print("Setting parentIndex to ");Serial.println(nodes[index].parentIndex);
        }*/

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
        if (nodes[i].parentIndex == -1 || nodes[i].parentIndex == 2) {
            strip.setPixelColor(i, 0, 255, 0);
        }
    }
    strip.show();
    return;
}


int getStrikePath (int pixelIndex, int *strikePath) {
    //int strikePath[NUMPIXELS];
    // length of strikePath array is sizeof strikePath / sizeof strikePath[0];
    int strikePathArrayLength = sizeof strikePath / sizeof strikePath[0];
    int pathLength = 0;

    // Initialize path indexes to -1
    for (int i = 0; i < strikePathArrayLength; i++) {
        strikePath[i] = -1;
    }

    int x = pixelIndex;
    // Keep moving along the path by setting x to the parentIndex.
    // TODO: FIX POTENTIAL INFINITE LOOP CAUSED BY CIRCULAR REFERENCE! (ex: 3 -> 2 -> 1 -> 3...)
    while (x >= 0) {
        strikePath[pathLength] = nodes[x].index;
        pathLength++;
        x = nodes[x].parentIndex;
    }
    pathLength--; // For some reason, this gets incremented an extra time. I can't be bothered to figure out why, so enjoy this lovely hack. -JD

    // Reverse the strikePath arrays
    // We only care about the first  numNodesInPath elements of the array, and we only need to iterate over half the values
    // So we stop after half the nodes in the path (the other elements should all be -1)
    int numNodesInPath = pathLength;
    for (int y = 0; y < numNodesInPath; y++) {
        if (numNodesInPath / 2 <= y) {
            break;
        }
        int tmp = strikePath[y];
        strikePath[y] = strikePath[numNodesInPath - y];
        strikePath[numNodesInPath - y] = tmp;
    }

    /*if (ENABLESERIAL) {
        Serial.print("Path length: ");Serial.println(numNodesInPath);
    }*/

    return pathLength;
}

void lightningStrike(int endPointArrayIndex, bool randomStrikes, int numStrikePaths) {
    if (DEBUG_LIGHTNING && ENABLESERIAL) {
        Serial.println("In lightningStrike");
    }
    // Trying to fix a stack overflow error when switching to QT PY ESP32. Array too long?
    //int strikePaths[numStrikePaths][NUMPIXELS];
    int strikePaths[numStrikePaths][MAX_PATH_LENGTH];

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

    if (DEBUG_LIGHTNING && ENABLESERIAL) {
        Serial.println("[lightningStrike]: Found endpoints");
    }

    // Initialize all the strike path indexes to -1
    for (int x = 0; x < numStrikePaths; x++) {
        //for (int y = 0; y < NUMPIXELS; y++) {
        for (int y = 0; y < MAX_PATH_LENGTH; y++) {
            if (DEBUG_LIGHTNING && ENABLESERIAL) {
                Serial.print("[init strikePath]: x=");Serial.print(x);Serial.print(" y=");Serial.println(y);
            }
            strikePaths[x][y] = -1;
        }
    }

    if (DEBUG_LIGHTNING && ENABLESERIAL) {
        Serial.println("[lightningStrike]: 1");
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
        for (int z = 0; z < NUMPIXELS; z++) {
            bool isEndOfPath = true;
            for (int x = 0; x < numStrikePaths; x++) {
                if (strikePaths[x][z] > -1) {
                    setPixelColor(strikePaths[x][z], b, b, b);
                    isEndOfPath = false;
                }
            }
            if (isEndOfPath) {
                break;
            }
        }

        strip.show();
        delay(20 * duration);
    }

    for (int z = 0; z < NUMPIXELS; z++) {
        bool isEndOfPath = true;
        for (int x = 0; x < numStrikePaths; x++) {
            if (strikePaths[x][z] > -1) {
                setPixelColor(strikePaths[x][z], 255, 255, 255);
                isEndOfPath = false;
            }
        }
        if (isEndOfPath) {
            break;
        }
    }

    strip.show();
    delay(10 * duration);

    for (int z = 0; z < NUMPIXELS; z++) {
        bool isEndOfPath = true;
        for (int x = 0; x < numStrikePaths; x++) {
            if (strikePaths[x][z] > -1) {
                setPixelColor(strikePaths[x][z], 16, 16, 16);
                isEndOfPath = false;
            }
        }
        if (isEndOfPath) {
            break;
        }
    }
    strip.show();
    delay(50 * duration);

    for (int z = 0; z < NUMPIXELS; z++) {
        bool isEndOfPath = true;
        for (int x = 0; x < numStrikePaths; x++) {
            if (strikePaths[x][z] > -1) {
                setPixelColor(strikePaths[x][z], 1, 1, 1);
                isEndOfPath = false;
            }
        }
        if (isEndOfPath) {
            break;
        }
    }
    strip.show();
    delay(50 * duration);

    for (int z = 0; z < NUMPIXELS; z++) {
        bool isEndOfPath = true;
        for (int x = 0; x < numStrikePaths; x++) {
            if (strikePaths[x][z] > -1) {
                setPixelColor(strikePaths[x][z], 255, 255, 255);
                isEndOfPath = false;
            }
        }
        if (isEndOfPath) {
            break;
        }
    }
    strip.show();
    delay(150 * duration);

    //      for (int b = 75; b > 12; b = b - 8) {
    for (int b = 100; b > 12; b = b - 10) {
        for (int z = 0; z < NUMPIXELS; z++) {
            bool isEndOfPath = true;
            for (int x = 0; x < numStrikePaths; x++) {
                if (strikePaths[x][z] > -1) {
                    setPixelColor(strikePaths[x][z], b, b, b);
                    isEndOfPath = false;
                }
            }
            if (isEndOfPath) {
                break;
            }
        }
        strip.show();
        delay(1);
    }

    for (int z = 0; z < NUMPIXELS; z++) {
        bool isEndOfPath = true;
        for (int x = 0; x < numStrikePaths; x++) {
            if (strikePaths[x][z] > -1) {
                setPixelColor(strikePaths[x][z], 10, 10, 10);
                isEndOfPath = false;
            }
        }
        if (isEndOfPath) {
            break;
        }
    }
    strip.show();
    delay(1);

    //strip.clear();
    for (int z = 0; z < NUMPIXELS; z++) {
        bool isEndOfPath = true;
        for (int x = 0; x < numStrikePaths; x++) {
            if (strikePaths[x][z] > -1) {
                setPixelColor(strikePaths[x][z], 0, 0, 0);
                isEndOfPath = false;
            }
        }
        if (isEndOfPath) {
            break;
        }
    }
    strip.show();
}

void vTaskPollLuminosity ( void * pvParameters ) {
    TickType_t xLastWakeTime;
    double luminositySum = 0;

    while (true) {
        xLastWakeTime = xTaskGetTickCount();
        luminositySum = 0;
        for (int x = LUMINOSITY_HISTORY_LENGTH - 1; x > 0; x--) {
            luminosityHistory[x] = luminosityHistory[x - 1];
            luminositySum += luminosityHistory[x];
        }
        luminosityHistory[0] = tsl.getLuminosity(TSL2591_VISIBLE);
        luminositySum += luminosityHistory[0];
        luminosity = (int) (luminositySum / LUMINOSITY_HISTORY_LENGTH);
        lastLuminosityPollTime = millis();
        if (luminosity > 500) {
            intensity = sqrt(luminosity) / 100;
        } else {
            intensity = sqrt(luminosity) / 1000;
        }

        if (intensity > 1.0) {
            intensity = 1.0;
        }

        if (ENABLESERIAL && DEBUG_LUMINOSITY) {
            Serial.print("Intensity(0-1): "); Serial.print(intensity);Serial.print("  luminosity: "); Serial.print(luminosity);
            Serial.print("   raw luminosity: "); Serial.println(luminosityHistory[0]);
        }

        vTaskDelayUntil( &xLastWakeTime, LUMINOSITY_POLLING_INTERVAL );
    }
}

long accelEmbargoTime = 0;

void vTaskPollAccel (void * pvParameters) {
    TickType_t xLastWakeTime;

    while (true) {
        uint8_t click = lis.getClick();
        if (millis() > accelEmbargoTime && (click & 0x10 || click & 0x20) ) {
            lis.getEvent(&accelEvent);
            tapEventAccelX = accelEvent.acceleration.x;
            tapEventAccelY = accelEvent.acceleration.y;
            if (ENABLESERIAL) {
                Serial.print("Click detected (0x");
                Serial.print(click, HEX);
                Serial.print(") (");
                Serial.print(click, BIN);
                Serial.print("): ");

                Serial.print("accelX: ");Serial.print(accelEvent.acceleration.x);
                Serial.print("  accelY: ");Serial.print(accelEvent.acceleration.y);

                if (click & 0x10) {
                    Serial.print(" single click ");
                }
                if (click & 0x20) Serial.print(" double click");
                // Is it the positive or negative axis?
                if (click & 0x8) {
                    Serial.print("-");
                } else {
                    Serial.print("+");
                }
                if (click & 0x4) {
                    Serial.println("Z");
                } else if (click & 0x2) {
                    Serial.println("Y");
                } else if (click & 0x1) {
                    Serial.println("X");
                }
                Serial.println();
            }
            accelEmbargoTime = millis() + 100;
            newTapEvent = true;
        }
        //lis.getEvent(&accelEvent);

        vTaskDelay(10);
    }
}

void vTaskPollButtons (void * pvParameters) {
    bool button1Embargo = false,
        button2Embargo = false;


    while (true) {
        button1State = digitalRead(BUTTON1_PIN);
        button2State = digitalRead(BUTTON2_PIN);

         // Don't register a new button press event until the button has been released (button1Embargo = false)
        if (button1State == HIGH && button1EmbargoTime < millis()) {
            button1Event = true;
        } else if (button1State == LOW) {
            button1Embargo = false;
        }

        if (button2State == HIGH && button2EmbargoTime < millis()) {
            button2Event = true;
        } else if (button2State == LOW) {
            button2Embargo = false;
        }
        vTaskDelay(10);
    }
}


#pragma clang diagnostic pop