#include <Arduino.h>
#include <ArduinoBLE.h>

#include <HCSR04.h>
#include <GPY0E02B.h>

BLEService controlService("180A"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic directionCharacteristic("1337", BLERead | BLEWrite);

byte usr1Pin = 7; // represents D7 - P0.23
UltraSonicDistanceSensor us1(usr1Pin);
GPY0E02B ir1;

/*
Read from ir1
*/
void readIRSensor()
{
  float distance = ir1.measureDistanceCm();
  if (distance != -1)
  {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  else
  {
    Serial.println("Error reading sensor data");
  }
  delay(500);
}

/*
Read from us1
*/
void readUltrasonicSensor()
{
  float distance = us1.measureDistanceCm();
  if (distance != -1)
  {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  else
  {
    Serial.println("Error reading sensor data");
  }
  delay(1000);
}

/*
Arduino Bluetooh example code
*/
void bluetoothTest()
{
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();
  // if a central is connected to peripheral:
  if (central)
  {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected())
    {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (directionCharacteristic.written())
      {
        switch (directionCharacteristic.value())
        { // any value other than 0
        case 1:
          Serial.println("Forward");
          digitalWrite(LED_BUILTIN, HIGH); // will turn the LED on
          break;
        case 2:
          Serial.println("Left");
          digitalWrite(LED_BUILTIN, HIGH); // will turn the LED on
          delay(500);
          digitalWrite(LED_BUILTIN, LOW); // will turn the LED off
          delay(500);
          digitalWrite(LED_BUILTIN, HIGH); // will turn the LED on
          delay(500);
          digitalWrite(LED_BUILTIN, LOW); // will turn the LED off
          break;
        case 3:
          Serial.println("Right");
          digitalWrite(LED_BUILTIN, HIGH); // will turn the LED on
          delay(1000);
          digitalWrite(LED_BUILTIN, LOW); // will turn the LED off
          delay(1000);
          digitalWrite(LED_BUILTIN, HIGH); // will turn the LED on
          delay(1000);
          digitalWrite(LED_BUILTIN, LOW); // will turn the LED off
          break;
        default:
          Serial.println(F("Stop"));
          digitalWrite(LED_BUILTIN, LOW); // will turn the LED off
          break;
        }
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, LOW); // will turn the LED off
  }
}



void setup()
{
  Serial.begin(9600);
  ir1.selectBus(0);
  /*pinMode(LED_BUILTIN, OUTPUT);
  if (!BLE.begin()) {
    Serial.println("Could not start BLE!");
    exit(1);
  }
  // set advertised local name and service UUID:
  BLE.setLocalName("The Navigator");
  BLE.setAdvertisedService(controlService);
  controlService.addCharacteristic(directionCharacteristic);
  BLE.addService(controlService);
  // set the initial value for the characteristic:
  directionCharacteristic.writeValue(0);
  // start advertising
  BLE.advertise();
  */
  Serial.println("Started BLE Robot");
}

/*
Main Loop
*/
void loop()
{
  readIRSensor();
  readUltrasonicSensor();
}
