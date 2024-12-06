#include <Arduino.h>
#include <ArduinoBLE.h>
#include <rtos.h>

#include <GPY0E02B.h>
#include <HCSR04.h>
#include <Motor.h>

UltraSonicDistanceSensor us1(7); // D7 on graph -> left
UltraSonicDistanceSensor us2(6); // D6 on graph -> right
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
rtos::Thread motorSyncThread;

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

void rotateRight(float angle) {
    Serial.print("Attempting to turn ");
    Serial.print(angle);
    Serial.println(" degrees...");

    float quarterCircumference = (angle / 360) * 13.8 * PI;
    motor.resetCount();
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

void rotateLeft(float angle) {
    Serial.print("Attempting to turn ");
    Serial.print(angle);
    Serial.println(" degrees...");

    float quarterCircumference = (angle / 360) * 13.8 * PI;
    motor.resetCount();
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

void moveForwardToWall() {
    float speed = 0.5f;
    motor.updateMotors(0, 1, speed, speed);
    bool wallFound = false;
    irBus.selectBus(0);
    while (!wallFound) {
        if (irBus.measureDistanceCm() <= 5) {
            wallFound = true;
            break;
        } else {
            delay(50);
        }
    }
    motor.stopMotors();
    if (us1.measureDistanceCm() > 6.0f && us2.measureDistanceCm() < 6.0f) {
        quarterTurnLeft();
    } else if (us1.measureDistanceCm() > 6.0f && us2.measureDistanceCm() <= 6.0f) {
        quarterTurnLeft();
    } else if (us1.measureDistanceCm() <= 6.0f && us2.measureDistanceCm() > 6.0f) {
        quarterTurnRight();
    } else {
        quarterTurnRight();
        quarterTurnRight();
    }
    moveForwardToWall();
}

float calculateStraightPathCorrectionAngle() {
    float d1L = us1.measureDistanceCm();
    float d1R = us2.measureDistanceCm();
    if (d1L > 15 && d1R > 15) {
        Serial.println("failed to detect wall");
        return 0.0f;
    }
    motor.resetCount();
    delay(100);
    float d2L = us1.measureDistanceCm();
    float d2R = us2.measureDistanceCm();
    float dmA = motor.calculateDistanceA();
    float dmB = motor.calculateDistanceB();

    float thetaL = 180 / PI * atan((d2L - d1L) / dmA);
    float thetaR = 180 / PI * atan((d2L - d1L) / dmA);
    if (abs(thetaL) >= abs(thetaR))
        return thetaL;
    else
        return -thetaR;
}

/*
Main Setup
*/
void setup() {
    delay(1000);
    Serial.begin(9600); // uncomment when not using bluetooth
    // Serial.begin(115200); // uncomment when using bluetooth
    // bluetoothSetup();
    // bluetoothThread.start(bluetoothTest);
    motor.setup();
    motor.startCounting();
    motorSyncThread.start(mbed::callback(&motor, &Motor::syncMotors));

    Serial.println("Started Robot");
}

/*
Main Loop
*/
void loop() {
    moveForward(0.4f);
    float f = calculateStraightPathCorrectionAngle();
    if (f < 0) {
        rotateRight(f);
    } else if (f > 0) {
        rotateLeft(f);
    }
    delay(1000);
    Serial.println("loop end");
}