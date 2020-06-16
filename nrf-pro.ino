// 
// Example program for NRF51Pro sensor buttons from AliExpress
//
//
// Basic iBeacon with LED, button and (optional) KX022 IMU
// NrfSensor Pro with buttons, RGB LED, IMU, temp and light sensors
//
// Uses Sandeep Mistry's nrf5x Arduino runtime, and the BLEPeripheral
// library by the same author.
//
//    https://github.com/sandeepmistry/arduino-nRF5
//    https://github.com/sandeepmistry/arduino-BLEPeripheral
//    
//

#include <Wire.h>
#include <SPI.h>
#include "BLEPeripheral.h"
#include "BLETypedCharacteristics.h"
#include <Adafruit_BMP280.h>

#define ENABLE_LEDS 1
#define ENABLE_BUTTONS 1


#ifdef ENABLE_BMP
Adafruit_BMP280 bmp; // I2C
bool bmpReady = false;
uint32_t lastTemp;
float temp;
float pressure;
#endif

#ifdef ENABLE_ALS
#include <AP3216_WE.h>
AP3216_WE myAP3216 = AP3216_WE();
uint32_t lastAls=0;
#endif


#define DEVICE_NAME "nrf_pro_0001"

BLEPeripheral device = BLEPeripheral();

// Create a 'custom service' (use a uuid generator to generate a random UUID)

BLEService service = BLEService(
  "48cbf5ae2fa411eaa4c2666574681792");

// Services have one or more characteristics.
// Characteristics may support read,write,notify and other operations
//
// Each characteristic can have zero or more descriptors that provide
// extra info.
//

struct presentationFormat {
  uint8_t  format;
  int8_t   exponent;
  uint16_t unit;
  uint8_t  blenamespace;
  uint16_t description;
} __attribute__((packed));

struct presentationFormat fmtU16 = {0x06, 0, 0, 0, 0 };
struct presentationFormat fmtU32 = {0x08, 0, 0, 0, 0 };
struct presentationFormat fmtU32_deci = {0x08, -1, 0, 0, 0 };
struct presentationFormat fmtS16_centi = {0x0E, -2, 0, 0, 0 };
struct presentationFormat fmtF32 = {0x14, 0, 0, 0, 0 };
  
//
// cLed represents our built in LED.   We can read its state and write
// its state.
//
#ifdef ENABLE_LEDS
BLEUnsignedLongCharacteristic cLed = BLEUnsignedLongCharacteristic(
  "5f5a14ae2fa411eab948666574681792", BLERead | BLEWrite );
BLEDescriptor dLedName = BLEDescriptor("2901", "LED color");
BLEDescriptor dLedFormat = BLEDescriptor("2904", (unsigned char *)&fmtU32, sizeof(fmtU32));

BLEUnsignedShortCharacteristic cLedCycle = BLEUnsignedShortCharacteristic(
  "b5bfe7e22fa411eab9d5666574681792", BLERead | BLEWrite);
BLEDescriptor dLedCycleName = BLEDescriptor("2901", "LED cycle time");
BLEDescriptor dLedCycleFormat = BLEDescriptor("2904", (unsigned char *)&fmtU16, sizeof(fmtU16));

BLEUnsignedShortCharacteristic cLedDuty = BLEUnsignedShortCharacteristic(
  "c7a43b022fa411ea83af666574681792", BLERead | BLEWrite);
BLEDescriptor dLedDutyName = BLEDescriptor("2901", "LED duty percent");
BLEDescriptor dLedDutyFormat = BLEDescriptor("2904", (unsigned char *)&fmtU16, sizeof(fmtU16));
#endif

//
// cButtonN represents an input (presumed active low with pullup).
//
// cCounterN represents the number of edges seen by the button.  You can
// reset the value by writing to it.   This can be used to detect clicks
// holds and doubleclicks (TODO: use button2 library to directly expose
// more button stunts)
//
#ifdef ENABLE_BUTTONS
BLEUnsignedShortCharacteristic cButton1 = BLEUnsignedShortCharacteristic(
  "d988af2e2fa411ea8dd5666574681792", BLERead | BLENotify);
BLEDescriptor dButton1Name = BLEDescriptor("2901", "Button 1");
BLEDescriptor dButton1Format = BLEDescriptor("2904", (unsigned char *)&fmtU16, sizeof(fmtU16));

BLEUnsignedLongCharacteristic cCounter1 = BLEUnsignedLongCharacteristic(
  "eb6c19562fa411eabe27666574681792", BLERead | BLEWrite | BLENotify);
BLEDescriptor dCounter1Name = BLEDescriptor("2901", "Counter 1");
BLEDescriptor dCounter1Format = BLEDescriptor("2904", (unsigned char *)&fmtU32, sizeof(fmtU32));

BLEUnsignedShortCharacteristic cButton2 = BLEUnsignedShortCharacteristic(
  "fd502a902fa411eabe1f666574681792", BLERead | BLENotify);
BLEDescriptor dButton2Name = BLEDescriptor("2901", "Button 2");
BLEDescriptor dButton2Format = BLEDescriptor("2904", (unsigned char *)&fmtU16, sizeof(fmtU16));

BLEUnsignedLongCharacteristic cCounter2 = BLEUnsignedLongCharacteristic(
  "5f5a14ae2fa411eab948666574681792", BLERead | BLEWrite | BLENotify);
BLEDescriptor dCounter2Name = BLEDescriptor("2901", "Counter 2");
BLEDescriptor dCounter2Format = BLEDescriptor("2904", (unsigned char *)&fmtU32, sizeof(fmtU32));
#endif

#ifdef ENABLE_BMP
BLEShortCharacteristic cTemp = BLEShortCharacteristic(
  "2A6E", BLERead | BLENotify );
BLEDescriptor dTempName = BLEDescriptor("2901", "Temperature");
BLEDescriptor dTempFormat = BLEDescriptor("2904", (unsigned char *)&fmtS16_centi, sizeof(fmtS16_centi));

BLEUnsignedLongCharacteristic cPressure = BLEUnsignedLongCharacteristic(
  "2A6D", BLERead | BLENotify);
BLEDescriptor dPressureName = BLEDescriptor("2901", "Pressure");
BLEDescriptor dPressureFormat = BLEDescriptor("2904", (unsigned char *)&fmtU32_deci, sizeof(fmtU32_deci));
#endif

#ifdef ENABLE_ALS
BLEFloatCharacteristic cLight = BLEFloatCharacteristic(
  "de6b0d82-471b-11ea-9ff8-cfc14d72b9c3", BLERead | BLENotify );
BLEDescriptor dLightName = BLEDescriptor("2901", "Light");
BLEDescriptor dLightFormat = BLEDescriptor("2904", (unsigned char *)&fmtF32, sizeof(fmtF32));

BLEUnsignedShortCharacteristic cPresence = BLEUnsignedShortCharacteristic(
  "defb582e-471b-11ea-9b52-53f422fea8f0", BLERead | BLENotify);
BLEDescriptor dPresenceName = BLEDescriptor("2901", "Presence");
BLEDescriptor dPresenceFormat = BLEDescriptor("2904", (unsigned char *)&fmtU16, sizeof(fmtU16));
#endif

//
// These are the pin assignments discovered by trial and error with the
// AliExpress nrf51 pro beacon (the one with RGB led and light sensor)
//
#ifdef ENABLE_LEDS
const byte ledPins[3] = {17,18,19};
bool led = false;
byte ledColor[3] = {51,0,102};
uint16_t ledCycle = 60000/72;
uint16_t ledDuty = 50;
#endif

#ifdef ENABLE_BUTTONS
int   button1Pin = 28;
int   button2Pin = 21;
bool button1 = HIGH;
uint32_t count1 = 0;
bool button2 = HIGH;
uint32_t count2 = 0;
#endif

const byte io1Pin = 0;
const byte io2Pin = 2;
const byte io3Pin = 3;
const byte io4Pin = 4;

const byte motionIntPin = 5;
const byte lightIntPin = 11;
const byte sclPin = 10;
const byte sdaPin = 9;

const byte tp1Pin = 30;
const byte tp2Pin = 16;
const byte tp3Pin = 15;
const byte tp4Pin = 14;

// 
// State of our LEDs and buttons
// 


bool connected = false;

void iosetup() 
{
  // 
  // Set up the IO pins and flash the LED four timesin [red,grn,blue,white]
  //
#ifdef ENABLE_BUTTONS
  pinMode(button1Pin, INPUT/*_PULLUP*/);
  pinMode(button2Pin, INPUT/*_PULLUP*/);
#endif
#ifdef ENABLE_LEDS
  pinMode(ledPins[0], OUTPUT);
  digitalWrite(ledPins[0], HIGH);
  pinMode(ledPins[1], OUTPUT);
  digitalWrite(ledPins[1], HIGH);
  pinMode(ledPins[2], OUTPUT);
  digitalWrite(ledPins[2], HIGH);
#endif
}

void setleds(bool r,bool g,bool b) 
{
#ifdef ENABLE_LEDS
  digitalWrite(ledPins[0], r);
  digitalWrite(ledPins[1], g);
  digitalWrite(ledPins[2], b);
  delay(250);
#endif
}

void flashleds()
{
#ifdef ENABLE_LEDS
  // 
  // flash the LED four timesin [red,grn,blue,white]
  // 
  digitalWrite(ledPins[0], HIGH);
  digitalWrite(ledPins[1], HIGH);
  digitalWrite(ledPins[2], HIGH);
  delay(500);
  for (int i=0; i<4;i++) {
    if (i<3) {
      digitalWrite(ledPins[i], LOW);
    }
    else {
      digitalWrite(ledPins[0], LOW);
      digitalWrite(ledPins[1], LOW);
      digitalWrite(ledPins[2], LOW);
    }
    delay(250);

    if (i<3) {
      digitalWrite(ledPins[i], HIGH);
    }
    else {
      digitalWrite(ledPins[0], HIGH);
      digitalWrite(ledPins[1], HIGH);
      digitalWrite(ledPins[2], HIGH);
    }
    delay(250);
  }
  delay(1000);
#endif
}

void setup() {
  Serial.begin(115200);
  iosetup();

#ifdef ENABLE_LEDS
  // 4 green flashes
  for (int i=0; i<4;i++) {
    setleds(HIGH,LOW,HIGH);
    setleds(HIGH, HIGH, HIGH);
  }
#endif
  
#ifdef ENABLE_BMP
    if (bmp.begin()) {
      bmpReady = true;

      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
		      Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
		      Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
		      Adafruit_BMP280::FILTER_X16,      /* Filtering. */
		      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    }
    else {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
      digitalWrite(ledPins[0], LOW);
      delay(1000);
      digitalWrite(ledPins[0], HIGH);
      delay(1000);
      digitalWrite(ledPins[0], LOW);
      delay(1000);
      digitalWrite(ledPins[0], HIGH);
      delay(1000);
      digitalWrite(ledPins[0], LOW);
      delay(1000);
      digitalWrite(ledPins[0], HIGH);
      delay(1000);
    }
#endif

#ifdef ENABLE_ALS
    Wire.begin();
    myAP3216.init();
    myAP3216.setLuxRange(RANGE_20661);
    myAP3216.setMode(ALS_PS_ONCE);
#endif


    setleds(LOW, HIGH, HIGH); // red

  device.setDeviceName(DEVICE_NAME);
  device.setLocalName(DEVICE_NAME);

  // add attributes (services, characteristics, descriptors) to the device
  device.addAttribute(service);

#ifdef ENABLE_LEDS
  device.addAttribute(cLed);
  device.addAttribute(dLedName);
  device.addAttribute(dLedFormat);

  device.addAttribute(cLedCycle);
  device.addAttribute(dLedCycleName);
  device.addAttribute(dLedCycleFormat);
  device.addAttribute(cLedDuty);
  device.addAttribute(dLedDutyName);
  device.addAttribute(dLedDutyFormat);
#endif

#ifdef ENABLE_BUTTONS
  device.addAttribute(cButton1);
  device.addAttribute(dButton1Name);
  device.addAttribute(dButton1Format);
  device.addAttribute(cCounter1);
  device.addAttribute(dCounter1Name);
  device.addAttribute(dCounter1Format);

  device.addAttribute(cButton2);
  device.addAttribute(dButton2Name);
  device.addAttribute(dButton2Format);
  device.addAttribute(cCounter2);
  device.addAttribute(dCounter2Name);
  device.addAttribute(dCounter2Format);
#endif

#ifdef ENABLE_BMP
  device.addAttribute(cTemp);
  device.addAttribute(dTempName);
  device.addAttribute(dTempFormat);

  device.addAttribute(cPressure);
  device.addAttribute(dPressureName);
  device.addAttribute(dPressureFormat);
#endif

#ifdef ENABLE_ALS
  device.addAttribute(cLight);
  device.addAttribute(dLightName);
  device.addAttribute(dLightFormat);

  device.addAttribute(cPresence);
  device.addAttribute(dPresenceName);
  device.addAttribute(dPresenceFormat);
#endif
  
  setleds(HIGH, LOW, HIGH);// green

#ifdef ENABLE_LEDS    
  cLed.setValue((ledColor[0]<<16)|(ledColor[1]<<8)|ledColor[2]);
  cLedCycle.setValue(ledCycle);
  cLedDuty.setValue(ledDuty);
#endif

#ifdef ENABLE_BUTTONS
  cCounter1.setValue(0);
  cCounter2.setValue(0);
#endif

  setleds(HIGH, HIGH, LOW); // blue
  device.begin();

  setleds(LOW, LOW, LOW);  // white
  flashleds();
}

void loop() {
  uint32_t now = millis();

  //device.poll();
  
  // 
  // Check whether a 'Central' device has connected to us.
  // 
  BLECentral central = device.central();

  if (!connected && central) {
    // 
    // We've detected a connection event
    // 
    Serial.print(F("Connection from central: "));
    Serial.println(central.address());
    connected = true;
  }
  else if (connected && !central) {
    // 
    // We've detected a diconnection event
    // 
    Serial.print(F("Disconnected "));
    Serial.println(central.address());
    connected = false;
  }
  if (connected){
    // 
    // Listen for operations on this device, until the connection
    // is closed.
    //
    // Reads are handled pretty much automatically, once we set the value.
    //
#ifdef ENABLE_LEDS
    if (cLed.written()) {
      // 
      // Handle a write operation on the LED state
      // 
      uint32_t color = cLed.value();
      ledColor[0] = (color>>16)&0xFF;
      ledColor[1] = (color>>8)&0xFF;
      ledColor[2] = color&0xff;
    }
    if (cLedCycle.written()) {
      // 
      // Handle a write operation on the LED cycle time
      // 
      ledCycle = cLedCycle.value();
    }
    if (cLedDuty.written()) {
      // 
      // Handle a write operation on the LED state
      // 
      ledDuty = cLedDuty.value();
    }
#endif

    // 
    // Check if the button count is changed
    //
#if 0
    if (cCounter1.written()) {
      count1 = cCounter1.value();
    }

    if (cCounter2.written()) {
      count2 = cCounter2.value();
    }
#endif
  }

  //
  // read the button and update its value/count if changed.
  //
#ifdef ENABLE_BUTTONS
  bool newButton1 = digitalRead(button1Pin);
  if (newButton1 != button1) {
    button1 = newButton1;
    cButton1.setValue(button1);
    cCounter1.setValue(++count1);
    if (newButton1) {
      ledColor[0] = 0xff;
    }
    else {
      ledColor[0] = 0x00;
    }
    
  }
  
  bool newButton2 = digitalRead(button2Pin);
  if (newButton2 != button2) {
    button2 = newButton2;
    cButton2.setValue(button2);
    cCounter2.setValue(++count2);
    if (newButton1) {
      ledColor[2] = 0xff;
    }
    else {
      ledColor[2] = 0x00;
    }
  }

/*
  // random wobble on counter 1
  if ((now % 1) && count1 < 4096) {
    cCounter1.setValue(++count1);
  }
  else if (count1 > 0) {
    cCounter1.setValue(--count1);
  }
*/
#endif
  
  // do any other loop processing here
#ifdef ENABLE_LEDS
  uint32_t cyclePos = now % ledCycle;
  uint32_t cycleTip = ledCycle * ledDuty / 100;

  if (!led && (cyclePos <= cycleTip) )
  {
    // turn led on
    analogWrite(ledPins[0], 255-ledColor[0]);
    analogWrite(ledPins[1], 255-ledColor[1]);
    analogWrite(ledPins[2], 255-ledColor[2]);
    led = true;
  }
  else if (led && (cyclePos > cycleTip) ) {
    // turn LED off
    analogWrite(ledPins[0], 255);
    analogWrite(ledPins[1], 255);
    analogWrite(ledPins[2], 255);
    led = false;
  }
#endif

#ifdef ENABLE_BMP
  if (bmpReady) {
    if ((now - lastTemp) > 2000) {
      lastTemp = now;
      temp = bmp.readTemperature();
      int16_t tempValue = temp*100;
      cTemp.setValue(tempValue);
      pressure = bmp.readPressure();
      uint32_t pressureValue = pressure*10;
      //cPressure.setValue(pressureValue);
    }
  }
#endif

#ifdef ENABLE_ALS
  if ((now - lastAls) > 000) {
    float als = myAP3216.getAmbientLight();
    cLight.setValue(als);
    unsigned int prox = myAP3216.getProximity();
    cPresence.setValue(prox);
    myAP3216.setMode(ALS_PS_ONCE);
  }
#endif

}
