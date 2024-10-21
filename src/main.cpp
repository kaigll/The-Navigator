#include <Arduino.h>
#include <ArduinoBLE.h>

BLEService controlService("180A"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic directionCharacteristic("1337", BLERead | BLEWrite);

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  if (!BLE.begin()) {
    Serial.println("Could not start BLE!");
    exit(1);
  }
  // set advertised local name and service UUID:
  BLE.setLocalName("Tac Shooter");
  BLE.setAdvertisedService(controlService);
  controlService.addCharacteristic(directionCharacteristic);
  BLE.addService(controlService);
  // set the initial value for the characteristic:
  directionCharacteristic.writeValue(0);
  // start advertising
  BLE.advertise();

  Serial.println("Started BLE Robot");
}

void loop() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (directionCharacteristic.written()) {
        switch (directionCharacteristic.value()) {   // any value other than 0
          case 1:
            Serial.println("Forward");
            digitalWrite(LED_BUILTIN, HIGH);            // will turn the LED on
            break;
          case 2:
              Serial.println("Left");
              digitalWrite(LED_BUILTIN, HIGH);         // will turn the LED on
              delay(500);
              digitalWrite(LED_BUILTIN, LOW);         // will turn the LED off
              delay(500);
              digitalWrite(LED_BUILTIN, HIGH);      // will turn the LED on
              delay(500);
              digitalWrite(LED_BUILTIN, LOW);       // will turn the LED off
            break;
          case 3:
            Serial.println("Right");
            digitalWrite(LED_BUILTIN, HIGH);         // will turn the LED on
              delay(1000);
              digitalWrite(LED_BUILTIN, LOW);         // will turn the LED off
              delay(1000);
              digitalWrite(LED_BUILTIN, HIGH);      // will turn the LED on
              delay(1000);
              digitalWrite(LED_BUILTIN, LOW);       // will turn the LED off
            break;
          default:
            Serial.println(F("Stop"));
            digitalWrite(LED_BUILTIN, LOW);          // will turn the LED off
            break;
        }
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, LOW);         // will turn the LED off
  }
}