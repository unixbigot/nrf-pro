// 
// Example program for NRF51 sensor buttons from AliExpress
//
// Tested with two types
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

#define DEVICE_NAME "nrf_button"

BLEPeripheral device = BLEPeripheral();

// Create a 'custom service' (use a uuid generator to generate a random UUID)

BLEService service = BLEService(
  "008267262ecb11eaaae2666574681792");

// Services have one or more characteristics.
// Characteristics may support read,write,notify and other operations
//
// Each characteristic can have zero or more descriptors that provide
// extra info.
//

//
// cLed represents our built in LED.   We can read its state and write
// its state.
//
BLEUnsignedShortCharacteristic cLed = BLEUnsignedShortCharacteristic(
  "1a3581622ecb11ea8515666574681792", BLERead | BLEWrite );
BLEDescriptor dLedName = BLEDescriptor("2901", "LED");

//
// cButton represents an input (presumed active low with pullup).
//
// Writing a value to this characteristic sets the pin number (perhaps
// you have a crappy undocumented device like me and you need to hunt the
// for the button's pin number.
//
BLEUnsignedShortCharacteristic cButton = BLEUnsignedShortCharacteristic(
  "e1bbb9822ecf11eaa429666574681792", BLERead | BLEWrite | BLENotify);
BLEDescriptor dButtonName = BLEDescriptor("2901", "button");

//
// cCounter represents the number of edges seen by the button.  You can
// reset the value by writing to it.   This can be used to detect clicks
// holds and doubleclicks (TODO: use button2 library to directly expose
// more button stunts)
//
BLEUnsignedLongCharacteristic cCounter = BLEUnsignedLongCharacteristic(
  "f0dc57a42edf11eaa8ca666574681792", BLERead | BLEWrite | BLENotify);
BLEDescriptor dCounterName = BLEDescriptor("2901", "counter");

//
// These are the pin assignments discovered by trial and error with the
// AliExpress keychain beacon (the one with a semicircular scallop in one side)
//
uint16_t ledPin = 29;
uint16_t buttonPin = 28;

// 
// State of our LED and button
// 
bool led = false;
bool button;
uint32_t count = 0;


void setup() {
  Serial.begin(115200);

  device.setDeviceName(DEVICE_NAME);
  device.setLocalName(DEVICE_NAME);

  // add attributes (services, characteristics, descriptors) to the device
  device.addAttribute(service);

  device.addAttribute(cLed);
  device.addAttribute(dLedName);

  device.addAttribute(cButton);
  device.addAttribute(dButtonName);

  device.addAttribute(cCounter);
  device.addAttribute(dCounterName);

  cCounter.setValue(0);

  device.begin();

  // 
  // Set up the IO pins and flash the LED twice to signify we're ready
  // 
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  for (int i=0; i<4;i++) {
    led=!led;
    digitalWrite(ledPin, led);
    delay(200);
  }
  
}

void loop() {

  // 
  // Check whether a 'Central' device has connected to us.
  // 
  BLECentral central = device.central();
  if (central) {

    // 
    // We're connected
    // 
    Serial.print(F("Connection from central: "));
    Serial.println(central.address());

    while (central.connected()) {

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
        led = cLed.value();
        digitalWrite(ledPin, led);
      }

      // 
      // Check if the button pin is changed.
      // Read the button and update its value.
      // 
      if (cButton.written()) {
	buttonPin = cButton.value();
	pinMode(buttonPin, INPUT_PULLUP);
      }
      bool newButton = digitalRead(buttonPin);
      if (newButton != button) {
	cButton.setValue(button = newButton);
	cCounter.setValue(++count);
      }

    }

    // The central device disconnected
    Serial.print(F("Disconnected "));
    Serial.println(central.address());
  }

  
}
