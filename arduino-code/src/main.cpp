#include <Arduino.h>
#include <ArduinoBLE.h>
#include <rtos.h>

#include <GPY0E02B.h>
#include <HCSR04.h>
#include <Motor.h>

#include <Coordinate.h>
#include <Map.h>

// US 1 Front
UltraSonicDistanceSensor us1(7);
// US 2 Back
UltraSonicDistanceSensor us2(6);

// ir 0 Left Front
// ir 1 Left Back
// ir 2 Right Front
// ir 3 Right Back
GPY0E02B irBus;

// Motor A is the Left
// Motor B is the Right
Motor motor(P0_4, P0_5, P0_27, P1_2);

#define FIRMWARE_SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define FIRMWARE_CHARACTERISTIC_UUID "abcdefab-cdef-abcd-efab-cdefabcdefab"

// Initialize the BLE services and characteristics
BLEService controlService("180A"); // BLE LED Service
BLEByteCharacteristic directionCharacteristic("1337", BLERead | BLEWrite);
BLEService firmwareService(FIRMWARE_SERVICE_UUID);
BLECharacteristic firmwareCharacteristic(FIRMWARE_CHARACTERISTIC_UUID, BLEWrite, 512);

// Declarning Threads
rtos::Thread bluetoothThread;
rtos::Thread motorSyncThread;
rtos::Thread straightenThread;
rtos::Thread movementThread;

enum State {
    MOVE_FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    STRAIGHTEN,
    U_TURN
};

State currentState = MOVE_FORWARD;

Coordinate coords(1, 1, 0);

Map mapInstance(29, 40, 5); // maze is roughly 145cm x 200cm

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

float kb_speed = 0;
void keyboardControls() {
    char input;

    if (Serial.available()) {
        input = Serial.read();
        Serial.println(input, HEX);
    }

    switch (input) {
    case 'w':
        motor.updateMotors(1, 0, kb_speed, kb_speed);
        break;
    case 'a':
        motor.updateMotors(1, 1, kb_speed, kb_speed);
        break;
    case 's':
        motor.updateMotors(0, 1, kb_speed, kb_speed);
        break;
    case 'd':
        motor.updateMotors(0, 0, kb_speed, kb_speed);
        break;
    case '1':
        kb_speed = 0.1f;
        break;
    case '2':
        kb_speed = 0.2f;
        break;
    case '3':
        kb_speed = 0.3f;
        break;
    case '4':
        kb_speed = 0.4f;
        break;
    case '5':
        kb_speed = 0.5f;
        break;
    case '6':
        kb_speed = 0.6f;
        break;
    case '7':
        kb_speed = 0.7f;
        break;
    case '8':
        kb_speed = 0.8f;
        break;
    case '9':
        kb_speed = 0.9f;
        break;
    case '0':
        kb_speed = 1.0f;
        break;
    default:
        break;
    }
}

void moveForward(float s) {
    motor.updateMotors(0, 1, s, s);
}

void moveBackward(float s) {
    motor.updateMotors(1, 0, s, s);
}

void turnRight(float angle, float speed) {
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
    coords.turnRight(angle);
    motor.stopMotorA();
    motor.stopMotorB();
    motor.resetCount();
}

void turnLeft(float angle, float speed) {
    Serial.print("Attempting to turn ");
    Serial.print(angle);
    Serial.println(" degrees...");

    float quarterCircumference = (angle / 360) * 13.8 * PI;
    motor.resetCount();
    motor.updateMotors(1, 1, speed, speed);
    while (true) {
        float distA = motor.calculateDistanceA();
        float distB = motor.calculateDistanceB();

        if (abs(distA) >= quarterCircumference || abs(distB) >= quarterCircumference) {
            break;
        }
        delay(10);
    }
    coords.turnLeft(angle);
    motor.stopMotorA();
    motor.stopMotorB();
    motor.resetCount();
}

void straighten() {
    irBus.selectBus(0);
    float dLF = irBus.measureDistanceCm();
    irBus.selectBus(1);
    float dLB = irBus.measureDistanceCm();
    irBus.selectBus(2);
    float dRF = irBus.measureDistanceCm();
    irBus.selectBus(3);
    float dRB = irBus.measureDistanceCm();

    // if no walls within limits
    float wallLimit = 15.0f;
    float errorLimit = 0.1f;
    if (dLF > wallLimit && dLB > wallLimit && dRF > wallLimit && dRB > wallLimit) {
        Serial.println("failed to detect wall");
        return;
    }

    if ((dLF + dLB) < (dRF + dRB)) {
        if (fabs(dLF - dLB) < errorLimit) {
            return;
        }

        if (dLF < dLB) {
            Serial.println("Aligning right to follow the left wall");
            float speed = 0.0f;
            while (dLF < dLB) {
                motor.updateMotors(0, 0, speed, speed); // turn right
                speed += 0.01f;

                irBus.selectBus(0);
                dLF = irBus.measureDistanceCm();
                irBus.selectBus(1);
                dLB = irBus.measureDistanceCm();

                if (fabs(dLF - dLB) < errorLimit) {
                    break;
                }
                delay(50);
            }
            motor.stopMotors();
            delay(10);
        } else if (dLF > dLB) {
            Serial.println("Aligning left to follow the left wall");
            float speed = 0.0f;
            while (dLF > dLB) {
                motor.updateMotors(1, 1, speed, speed); // turn left
                speed += 0.01f;

                irBus.selectBus(0);
                dLF = irBus.measureDistanceCm();
                irBus.selectBus(1);
                dLB = irBus.measureDistanceCm();
                if (fabs(dLF - dLB) < 0.5) {
                    break;
                }
                delay(50);
            }
            motor.stopMotors();
            delay(10);
        }
    } else if ((dLF + dLB) > (dRF + dRB)) {
        if (fabs(dRF - dRB) < errorLimit) {
            return;
        }
        if (dRF < dRB) {
            Serial.println("Aligning left to follow the right wall");
            float speed = 0.0f;
            while (dRF < dRB) {
                motor.updateMotors(1, 1, speed, speed); // turn left
                speed += 0.01f;

                irBus.selectBus(2);
                dRF = irBus.measureDistanceCm();
                irBus.selectBus(3);
                dRB = irBus.measureDistanceCm();
                if (fabs(dRF - dRB) < 0.5) {
                    break;
                }
                delay(50);
            }
            motor.stopMotors();
            delay(10);
        } else if (dRF > dRB) {
            Serial.println("Aligning right to follow the right wall");
            float speed = 0.0f;
            while (dRF > dRB) {
                motor.updateMotors(0, 0, speed, speed); // turn right
                speed += 0.01f;

                irBus.selectBus(2);
                dRF = irBus.measureDistanceCm();
                irBus.selectBus(3);
                dRB = irBus.measureDistanceCm();
                if (fabs(dRF - dRB) < 0.5) {
                    break;
                }
                delay(50);
            }
            motor.stopMotors();
            delay(10);
        }
    }
}

void localiseWalls() {
}

void moveForwardToWall() {
    straighten();
    straighten();
    float speed = 0.5f;
    motor.updateMotors(0, 1, speed, speed);
    bool wallFound = false;
    motor.resetCount();
    float lastDistA = motor.calculateDistanceA();
    float lastDistB = motor.calculateDistanceB();
    float lastDistance = us1.measureDistanceCm();

    while (!wallFound) {
        if (lastDistance <= 5) {
            wallFound = true;
            break;
        } else {
            delay(50);
        }
        if (abs(us1.measureDistanceCm() - lastDistance) < 0.1 || abs(motor.calculateDistanceA() - lastDistA) < 0.001 || abs(motor.calculateDistanceB() - lastDistB) < 0.001) {
            motor.updateMotors(1, 0, speed, speed);
            delay(300);
            return;
        } else {
            lastDistA = motor.calculateDistanceA();
            lastDistB = motor.calculateDistanceB();
            lastDistance = us1.measureDistanceCm();
        }
    }
    motor.stopMotors();

    irBus.selectBus(0);
    float dLF = irBus.measureDistanceCm();
    irBus.selectBus(1);
    float dLB = irBus.measureDistanceCm();
    irBus.selectBus(2);
    float dRF = irBus.measureDistanceCm();
    irBus.selectBus(3);
    float dRB = irBus.measureDistanceCm();

    if (dLF > 10.0f && dRF < 10.0f) {
        turnLeft(90, 0.4f);
    } else if (dLF > 10.0f && dRF <= 10.0f) {
        turnLeft(90, 0.4f);
    } else if (dLF <= 10.0f && dRF > 10.0f) {
        turnRight(90, 0.4f);
    } else {
        turnRight(90, 0.4f);
    }
    moveForwardToWall();
}

void movementStateMachine() {
    irBus.selectBus(0);
    float dLF = irBus.measureDistanceCm();
    irBus.selectBus(1);
    float dLB = irBus.measureDistanceCm();
    irBus.selectBus(2);
    float dRF = irBus.measureDistanceCm();
    irBus.selectBus(3);
    float dRB = irBus.measureDistanceCm();
    float dFront = us1.measureDistanceCm();

    switch (currentState) {
    case MOVE_FORWARD:
        if (dFront < 6) {
            if (dLF > 10) {
                currentState = TURN_LEFT;
            } else if (dRF > 10) {
                currentState = TURN_RIGHT;
            } else {
                currentState = U_TURN;
            }
        } else {
            moveForward(0.5f);
            ;
        }
        break;
    case TURN_LEFT:
        turnLeft(90, 0.4f);
        currentState = MOVE_FORWARD;
        break;
    case TURN_RIGHT:
        turnRight(90, 0.4f);
        currentState = MOVE_FORWARD;
        break;
    case U_TURN:
        turnLeft(180, 0.5f);
        currentState = MOVE_FORWARD;
        break;
    default:
        currentState = MOVE_FORWARD;
        break;
    }
    delay(100);
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
    // movementThread.start(movementStateMachine);

    mapInstance.initializeGrid();

    Serial.println("Started Robot");
}

/*
Main Loop
*/
void loop() {

    turnLeft(10, 0.3f);

    irBus.selectBus(0);
    float dLF = irBus.measureDistanceCm();
    irBus.selectBus(1);
    float dLB = irBus.measureDistanceCm();
    irBus.selectBus(2);
    float dRF = irBus.measureDistanceCm();
    irBus.selectBus(3);
    float dRB = irBus.measureDistanceCm();
    float dFront = us1.measureDistanceCm();
    float dBack = us2.measureDistanceCm();

    mapInstance.updateGrid(coords, dLF, dLB, dRF, dRB, dFront, dBack);
    mapInstance.printGrid();
    delay(10000);
    Serial.println("loop end");
}