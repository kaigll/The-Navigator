#include <Arduino.h>
#include <ArduinoBLE.h>
#include <rtos.h>

#include <GPY0E02B.h>
#include <HCSR04.h>
#include <Motor.h>

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
float speed = 0;

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

void turnRight(float angle) {
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

void turnLeft(float angle) {
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

void moveForwardToWall() {
    float speed = 0.5f;
    motor.updateMotors(0, 1, speed, speed);
    bool wallFound = false;
    while (!wallFound) {
        if (us1.measureDistanceCm() <= 5) {
            wallFound = true;
            break;
        } else {
            delay(50);
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
        turnLeft(90);
    } else if (dLF > 10.0f && dRF <= 10.0f) {
        turnLeft(90);
    } else if (dLF <= 10.0f && dRF > 10.0f) {
        turnRight(90);
    } else {
        turnRight(90);
    }
    moveForwardToWall();
}

float calculateStraightPathCorrectionAngle() {
    irBus.selectBus(0);
    float dLF = irBus.measureDistanceCm();
    irBus.selectBus(1);
    float dLB = irBus.measureDistanceCm();
    irBus.selectBus(2);
    float dRF = irBus.measureDistanceCm();
    irBus.selectBus(3);
    float dRB = irBus.measureDistanceCm();

    if (dLF > 15 && dLB > 15 && dRF > 15 && dRB > 15) {
        Serial.println("failed to detect wall");
        return 0.0f;
    }
    if (dLF < 15 && dLB < 15) {
        return -180 / PI * atan((dLB - dLF) / 9.6f);
    } else if (dRF < 15 && dRB < 15) {
        return 180 / PI * atan((dRB - dRF) / 9.6f);
    }
}

void straightenPath() {
    float angle = calculateStraightPathCorrectionAngle();
    while (abs(angle) > 3.0f) {
        Serial.println(angle);
        motor.stopMotors();
        delay(100);
        if (angle > 3.0f) {
            turnLeft(angle);
        } else if (angle < -3.0f) {
            turnRight(-angle);
        }
        angle = calculateStraightPathCorrectionAngle();
    }
    moveForward(0.5f);
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
            straightenPath();
        }
        break;
    case TURN_LEFT:
        turnLeft(90);
        currentState = MOVE_FORWARD;
        break;
    case TURN_RIGHT:
        turnRight(90);
        currentState = MOVE_FORWARD;
        break;
    case U_TURN:
        turnLeft(180);
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
    //movementThread.start(movementStateMachine);

    Serial.println("Started Robot");
}

/*
Main Loop
*/
void loop() {
    moveForward(0.5f);
    straightenPath();
    delay(10);
    Serial.println("loop end");
}