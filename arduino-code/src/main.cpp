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

// Declarning Threads
rtos::Thread motorSyncThread;
rtos::Thread updateThread;

enum RobotState {
    IDLE,
    TURNING_LEFT,
    TURNING_RIGHT,
    MOVING_FORWARD
};

RobotState robotState = IDLE;
float targetDistance;
float currentDistance;
float moveSpeed;

Map mapInstance(29, 40, 5); // maze is roughly 145cm x 200cm

struct Action {
    RobotState state;
    float value; // Angle for turning, distance for moving forward
    float speed;
};

#define MAX_ACTIONS 10

Action actionQueue[MAX_ACTIONS];
int actionCount = 0;
int currentActionIndex = 0;

void enqueueAction(RobotState state, float value, float speed) {
    if (actionCount < MAX_ACTIONS) {
        actionQueue[actionCount++] = {state, value, speed};
        Serial.println("Action added to queue.");
    } else {
        Serial.println("Action queue is full.");
    }
}

void turnLeft(float angle, float speed) {
    Serial.print("Attempting to turn left ");
    Serial.print(angle);
    Serial.println(" degrees...");

    float partialCircumference = (angle / 360) * 13.8 * PI;
    targetDistance = partialCircumference;
    currentDistance = 0;
    moveSpeed = speed;
    robotState = TURNING_LEFT;
    motor.resetCount();
    motor.updateMotors(1, 1, speed, speed);
}

void turnRight(float angle, float speed) {
    Serial.print("Attempting to turn right ");
    Serial.print(angle);
    Serial.println(" degrees...");

    float partialCircumference = (angle / 360) * 13.8 * PI;
    targetDistance = partialCircumference;
    currentDistance = 0;
    moveSpeed = speed;
    robotState = TURNING_RIGHT;
    motor.resetCount();
    motor.updateMotors(0, 0, speed, speed);
}

void moveForward(float distance, float speed) {
    Serial.print("Attempting to move forward ");
    Serial.print(distance);
    Serial.println(" cm...");

    targetDistance = distance;
    currentDistance = 0;
    moveSpeed = speed;
    robotState = MOVING_FORWARD;
    motor.resetCount();
    motor.updateMotors(0, 1, speed, speed);
}

void update() {
    while (true) {
        float distA = motor.calculateDistanceA();
        float distB = motor.calculateDistanceB();

        switch (robotState) {
        case TURNING_LEFT:
        case TURNING_RIGHT:
            currentDistance = (abs(distA) + abs(distB)) / 2;

            if (currentDistance >= targetDistance) {
                motor.stopMotors();
                thread_sleep_for(10);
                motor.resetCount();
                Serial.println("Turn complete.");
                currentActionIndex++;
                robotState = IDLE;
            }
            break;

        case MOVING_FORWARD:
            currentDistance = (abs(distA) + abs(distB)) / 2;

            if (currentDistance >= targetDistance) {
                motor.stopMotors();
                thread_sleep_for(10);
                motor.resetCount();
                Serial.println("Move complete.");
                currentActionIndex++;
                robotState = IDLE;
            }
            break;

        case IDLE:
        default:
            if (currentActionIndex < actionCount) {
                Action &action = actionQueue[currentActionIndex];
                switch (action.state) {
                case TURNING_LEFT:
                    turnLeft(action.value, action.speed);
                    break;
                case TURNING_RIGHT:
                    turnRight(action.value, action.speed);
                    break;
                case MOVING_FORWARD:
                    moveForward(action.value, action.speed);
                    break;
                default:
                    break;
                }
                // Do nothing
                break;
            }
            thread_sleep_for(10);
        }
    }
}

void alignRight(float &frontDistance, float &backDistance, int frontBus, int backBus, float errorLimit) {
    float speed = 0.0f;
    while (frontDistance < backDistance) {
        motor.updateMotors(0, 0, speed, speed); // turn right
        speed += 0.01f;
        irBus.selectBus(frontBus);
        frontDistance = irBus.measureDistanceCm();
        irBus.selectBus(backBus);
        backDistance = irBus.measureDistanceCm();
        if (fabs(frontDistance - backDistance) < errorLimit) {
            break;
        }
        thread_sleep_for(50);
    }
    motor.stopMotors();
    thread_sleep_for(10);
}

void alignLeft(float &frontDistance, float &backDistance, int frontBus, int backBus, float errorLimit) {
    float speed = 0.0f;
    while (frontDistance > backDistance) {
        motor.updateMotors(1, 1, speed, speed); // turn left
        speed += 0.01f;
        irBus.selectBus(frontBus);
        frontDistance = irBus.measureDistanceCm();
        irBus.selectBus(backBus);
        backDistance = irBus.measureDistanceCm();
        if (fabs(frontDistance - backDistance) < errorLimit) {
            break;
        }
        thread_sleep_for(50);
    }
    motor.stopMotors();
    thread_sleep_for(10);
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

    // if right is closer to wall
    if ((dLF + dLB) < (dRF + dRB)) {
        if (fabs(dLF - dLB) < errorLimit) {
            return;
        }
        if (dLF < dLB) {
            Serial.println("Aligning right to follow the left wall");
            alignRight(dLF, dLB, 0, 1, errorLimit);
        } else if (dLF > dLB) {
            Serial.println("Aligning left to follow the left wall");
            alignLeft(dLF, dLB, 0, 1, errorLimit);
        }
        // if left is closer to wall
    } else if ((dLF + dLB) > (dRF + dRB)) {
        if (fabs(dRF - dRB) < errorLimit) {
            return;
        }
        if (dRF < dRB) {
            Serial.println("Aligning left to follow the right wall");
            alignLeft(dRF, dRB, 2, 3, errorLimit);
        } else if (dRF > dRB) {
            Serial.println("Aligning right to follow the right wall");
            alignRight(dRF, dRB, 2, 3, errorLimit);
        }
    }
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
            thread_sleep_for(50);
        }
        if (abs(us1.measureDistanceCm() - lastDistance) < 0.1 || abs(motor.calculateDistanceA() - lastDistA) < 0.001 || abs(motor.calculateDistanceB() - lastDistB) < 0.001) {
            motor.updateMotors(1, 0, speed, speed);
            thread_sleep_for(300);
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

void setup() {
    Serial.begin(9600);

    delay(100);

    motor.setup();
    motor.startCounting();
    // motorSyncThread.start(mbed::callback(&motor, &Motor::syncMotors));

    mapInstance.setRobotPosition(1, 1, 0);

    updateThread.start(update);

    Serial.println("Started Robot");

    thread_sleep_for(1000);

    enqueueAction(TURNING_LEFT, 90, 0.5f);
    enqueueAction(MOVING_FORWARD, 10, 0.5f);
    enqueueAction(TURNING_RIGHT, 45, 0.5f);
    enqueueAction(MOVING_FORWARD, 5, 0.5f);

}

void loop() {
    // nothing
}