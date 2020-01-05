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

#include <SPI.h>
#include <BLEPeripheral.h>
#include "BLETypedCharacteristics.h"

#define DEVICE_NAME "nrf_pro"

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
  
//
// cLed represents our built in LED.   We can read its state and write
// its state.
//
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

//
// cButtonN represents an input (presumed active low with pullup).
//
// cCounterN represents the number of edges seen by the button.  You can
// reset the value by writing to it.   This can be used to detect clicks
// holds and doubleclicks (TODO: use button2 library to directly expose
// more button stunts)
//
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
  "", BLERead | BLEWrite | BLENotify);
BLEDescriptor dCounter2Name = BLEDescriptor("2901", "Counter 2");
BLEDescriptor dCounter2Format = BLEDescriptor("2904", (unsigned char *)&fmtU32, sizeof(fmtU32));

//
// These are the pin assignments discovered by trial and error with the
// AliExpress nrf51 pro beacon (the one with RGB led and light sensor)
//
const byte ledPins[3] = {17,18,19};
uint16_t   button1Pin = 28;
uint16_t   button2Pin = 21;

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
bool led = false;
byte ledColor[3] = {51,0,102};
uint16_t ledCycle = 60000/72;
uint16_t ledDuty = 10;

bool button1;
uint32_t count1 = 0;
bool button2;
uint32_t count2 = 0;

bool connected = false;


void setup() {
  Serial.begin(115200);

  device.setDeviceName(DEVICE_NAME);
  device.setLocalName(DEVICE_NAME);

  // add attributes (services, characteristics, descriptors) to the device
  device.addAttribute(service);

  device.addAttribute(cLed);
  device.addAttribute(dLedName);
  device.addAttribute(dLedFormat);

  device.addAttribute(cLedCycle);
  device.addAttribute(dLedCycleName);
  device.addAttribute(dLedCycleFormat);
  device.addAttribute(cLedDuty);
  device.addAttribute(dLedDutyName);
  device.addAttribute(dLedDutyFormat);

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

  cLed.setValue((ledColor[0]<<16)|(ledColor[1]<<8)|ledColor[2]);
  cLedCycle.setValue(ledCycle);
  cLedDuty.setValue(ledDuty);
  cCounter1.setValue(0);
  cCounter2.setValue(0);

  device.begin();

  // 
  // Set up the IO pins and flash the LED four timesin [red,grn,blue,white]
  // 
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(ledPins[0], OUTPUT);
  digitalWrite(ledPins[0], HIGH);
  pinMode(ledPins[1], OUTPUT);
  digitalWrite(ledPins[1], HIGH);
  pinMode(ledPins[2], OUTPUT);
  digitalWrite(ledPins[2], HIGH);
  
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
  
}

void loop() {

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
    if (cLed.written()) {
      // 
      // Handle a write operation on the LED state
      // 
      uint32_t color = cLed.value();
      ledColor[0] = (color>>16)&0xFF;
      ledColor[1] = (color>>8)&0xFF;
      ledColor[2] = color&0xff;
    }
    
    // 
    // Check if the button count is changed, then
    // read the button and update its value/count if changed.
    // 
    if (cCounter1.written()) {
      count1 = cCounter1.value();
    }
    bool newButton1 = digitalRead(button1Pin);
    if (newButton1 != button1) {
      cButton1.setValue(button1 = newButton1);
      cCounter1.setValue(++count1);
    }

    if (cCounter2.written()) {
      count2 = cCounter2.value();
    }
    bool newButton2 = digitalRead(button2Pin);
    if (newButton2 != button2) {
      cButton2.setValue(button2 = newButton2);
      cCounter2.setValue(++count2);
    }
  } // end if connected

  // do any other loop processing here
  uint32_t now = millis();
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


}
