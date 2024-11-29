#include <Arduino.h>
#include <ArduinoBLE.h>
#include <rtos.h>

#include <GPY0E02B.h>
#include <HCSR04.h>
#include <Motor.h>

UltraSonicDistanceSensor us1(7); // D7 on graph
UltraSonicDistanceSensor us2(6); // D6 on graph
GPY0E02B irBus;

// Motor motor(P0_27, P1_2, P0_4, P0_5);
Motor motor(P0_4, P0_5, P0_27, P1_2);
float speed = 0;

#define FIRMWARE_SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define FIRMWARE_CHARACTERISTIC_UUID "abcdefab-cdef-abcd-efab-cdefabcdefab"

// Initialize the BLE services and characteristics
BLEService controlService("180A"); // BLE LED Service
BLEByteCharacteristic directionCharacteristic("1337", BLERead | BLEWrite);
BLEService firmwareService(FIRMWARE_SERVICE_UUID);
BLECharacteristic firmwareCharacteristic(FIRMWARE_CHARACTERISTIC_UUID, BLEWrite, 512);

rtos::Thread bluetoothThread;

void bluetoothSetup() {
    pinMode(LED_BUILTIN, OUTPUT);
    if (!BLE.begin()) {
        Serial.println("Could not start BLE!");
        exit(1);
    }
    // set advertised local name and service UUID:
    BLE.setLocalName("The Navigator");
    BLE.setAdvertisedService(controlService);
    // Add characteristics to the control service
    controlService.addCharacteristic(directionCharacteristic);
    BLE.addService(controlService);

    // Add the firmware update service and characteristic
    firmwareService.addCharacteristic(firmwareCharacteristic);
    BLE.addService(firmwareService);
    // set the initial value for the characteristic:
    directionCharacteristic.writeValue(0);
    // start advertising
    BLE.advertise();

    Serial.println("BLE services initialized, waiting for connections...");
}

/*
Arduino Bluetooh example code
*/
void bluetoothTest() {
    while (true) {
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
                    switch (directionCharacteristic.value()) { // any value other than 0
                    case 1:
                        Serial.println("Forward");
                        digitalWrite(LED_BUILTIN,
                                     HIGH); // will turn the LED on
                        break;
                    case 2:
                        Serial.println("Left");
                        digitalWrite(LED_BUILTIN,
                                     HIGH); // will turn the LED on
                        delay(500);
                        digitalWrite(LED_BUILTIN,
                                     LOW); // will turn the LED off
                        delay(500);
                        digitalWrite(LED_BUILTIN,
                                     HIGH); // will turn the LED on
                        delay(500);
                        digitalWrite(LED_BUILTIN,
                                     LOW); // will turn the LED off
                        break;
                    case 3:
                        Serial.println("Right");
                        digitalWrite(LED_BUILTIN,
                                     HIGH); // will turn the LED on
                        delay(1000);
                        digitalWrite(LED_BUILTIN,
                                     LOW); // will turn the LED off
                        delay(1000);
                        digitalWrite(LED_BUILTIN,
                                     HIGH); // will turn the LED on
                        delay(1000);
                        digitalWrite(LED_BUILTIN,
                                     LOW); // will turn the LED off
                        break;
                    default:
                        Serial.println(F("Stop"));
                        digitalWrite(LED_BUILTIN,
                                     LOW); // will turn the LED off
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
}

/*
Read from ir sensor on i2c
*/
void readIRSensor(GPY0E02B ir) {
    float distance = ir.measureDistanceCm();
    if (distance != -1) {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    } else {
        Serial.println("Error reading sensor data");
    }
}

/*
Read from ultrasonic sensor
*/
void readUltrasonicSensor(UltraSonicDistanceSensor us) {
    float distance = us.measureDistanceCm();
    if (distance != -1) {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    } else {
        Serial.println("Error reading sensor data");
    }
}

void keyboardControls() {
    char input;

    if (Serial.available()) {
        input = Serial.read();
        Serial.println(input, HEX);
    }

    switch (input) {
    case 'w':
        motor.updateMotors(1, 0, speed, speed);
        break;
    case 'a':
        motor.updateMotors(1, 1, speed, speed);
        break;
    case 's':
        motor.updateMotors(0, 1, speed, speed);
        break;
    case 'd':
        motor.updateMotors(0, 0, speed, speed);
        break;
    case '1':
        speed = 0.1f;
        break;
    case '2':
        speed = 0.2f;
        break;
    case '3':
        speed = 0.3f;
        break;
    case '4':
        speed = 0.4f;
        break;
    case '5':
        speed = 0.5f;
        break;
    case '6':
        speed = 0.6f;
        break;
    case '7':
        speed = 0.7f;
        break;
    case '8':
        speed = 0.8f;
        break;
    case '9':
        speed = 0.9f;
        break;
    case '0':
        speed = 1.0f;
        break;
    default:
        break;
    }
}

void moveForward(float s) {
    motor.updateMotors(0, 1, s, s);
}

void moveBackward(float s) {
    motor.updateMotors(0, 1, s, s);
}

void rotateRight(float s) {
    motor.updateMotors(0, 0, s, s);
    // thread_sleep_for(1000); // Adjust the timing for a 90-degree rotation based on your setup
    delay(1000);
    motor.stopMotorA();
    motor.stopMotorB();
}

void rotateLeft(float s) {
    motor.updateMotors(1, 1, s, s);
    // thread_sleep_for(1000); // Adjust the timing for a 90-degree rotation based on your setup
    delay(1000);
    motor.stopMotorA();
    motor.stopMotorB();
}

// turn 90 degrees left (clockwise)
void quarterTurnRight() {
    float quarterCircumference = 0.25 * 13.8 * PI;
    motor.updateMotors(0, 0, 0.5f, 0.5f);
    while (true) {
        float distA = motor.calculateDistanceA();
        float distB = motor.calculateDistanceB();

        if (abs(distA) >= quarterCircumference || abs(distB) >= quarterCircumference) {
            break;
        }
        delay(10);
    }
    motor.stopMotorA();
    motor.stopMotorB();
    motor.resetCount();
}

// turn 90 degrees left (anticlockwise)
void quarterTurnLeft() {
    float quarterCircumference = 0.25 * 13.8 * PI;
    motor.updateMotors(1, 1, 0.5f, 0.5f);
    while (true) {
        float distA = motor.calculateDistanceA();
        float distB = motor.calculateDistanceB();

        if (abs(distA) >= quarterCircumference || abs(distB) >= quarterCircumference) {
            break;
        }
        delay(10);
    }
    motor.stopMotorA();
    motor.stopMotorB();
    motor.resetCount();
}

/*
Main Setup
*/
void setup() {
    delay(1000);
    // Serial.begin(9600);
    Serial.begin(115200);
    bluetoothSetup();

    motor.setup();
    Serial.println("Started BLE Robot");
    motor.startCounting();
    bluetoothThread.start(bluetoothTest);
}

/*
Main Loop
*/
void loop() {
    quarterTurnLeft();
    delay(1000);
    quarterTurnLeft();
    delay(1000);
    quarterTurnLeft();
    delay(1000);
    quarterTurnLeft();
    delay(1000);
    quarterTurnRight();
    delay(1000);
    quarterTurnRight();
    delay(1000);
    quarterTurnRight();
    delay(1000);
    quarterTurnRight();
    delay(1000);
    delay(10000);
    Serial.println("loop end");
}