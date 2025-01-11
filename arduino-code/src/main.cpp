#include <Arduino.h>
#include <ArduinoBLE.h>
#include <rtos.h>

#include <GPY0E02B.h>
#include <HCSR04.h>
#include <Map.h>
#include <Motor.h>
#include <queue>

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
rtos::Thread syncMotorsThread;
rtos::Thread updateThread;

// Initialise map
// Map mapInstance(29, 40, 5); // maze is roughly 145cm x 200cm
Map mapInstance(150, 200, 1);

// Define all possible states
enum RobotState {
    IDLE,
    TURNING_LEFT,
    TURNING_RIGHT,
    MOVING_FORWARD,
    STRAIGHTEN
};

RobotState robotState = IDLE;
float targetDistance;
float currentDistance;

// Define an action struct to allow for movement planning
struct Action {
    RobotState state;
    float value; // for turning: angle, for moving: distance
    float speed;
};

std::queue<Action> actionQueue;

void enqueueAction(RobotState state, float value, float speed) {
    actionQueue.push({state, value, speed});
    Serial.println((String) "Addeed:" + state + " to action queue");
}

void turnLeft(float angle, float speed) {
    Serial.println((String) "Attempting to turn left " + angle + " degrees...");

    float partialCircumference = (angle / 360) * 13.8 * PI;
    targetDistance = partialCircumference;
    currentDistance = 0;
    robotState = TURNING_LEFT;
    motor.resetCount();
    mapInstance.rotateRobotLeft(angle);
    motor.updateMotors(1, 1, speed, speed);
}

void turnRight(float angle, float speed) {
    Serial.println((String) "Attempting to turn right " + angle + " degrees...");

    float partialCircumference = (angle / 360) * 13.8 * PI;
    targetDistance = partialCircumference;
    currentDistance = 0;
    robotState = TURNING_RIGHT;
    motor.resetCount();
    mapInstance.rotateRobotRight(angle);
    motor.updateMotors(0, 0, speed, speed);
}

void moveForward(float distance, float speed) {
    Serial.println((String) "Attempting to move forward " + distance + " cm...");

    targetDistance = distance;
    currentDistance = 0;
    robotState = MOVING_FORWARD;
    motor.resetCount();
    mapInstance.moveRobotForward(distance);
    motor.updateMotors(0, 1, speed, speed);
}

void alignLeft() {
    float const ERROR = 0.3;

    while (irBus.measureDistanceCm(0) != irBus.measureDistanceCm(1)) {
        float dLF = irBus.measureDistanceCm(0);
        float dLB = irBus.measureDistanceCm(1);
        if (dLF == dLB) {
            break;
        }
        if (dLF > dLB + ERROR) {
            turnLeft(1, 0.3f);
        } else if (dLB > dLF + ERROR) {
            turnRight(1, 0.3f);
        }
    }

    motor.stopMotors();
}

void alignRight() {
    float const ERROR = 0.3; // as the IRsensors will have natural variance between eachother this constant represents how much they can differ and still be accepted as "aligned"

    while (irBus.measureDistanceCm(2) != irBus.measureDistanceCm(3)) {
        float dRF = irBus.measureDistanceCm(2);
        float dRB = irBus.measureDistanceCm(3);
        if (dRF == dRB) {
            break;
        }
        if (dRF > dRB + ERROR) {
            turnRight(1, 0.3f);
        } else if (dRB > dRF + ERROR) {
            turnLeft(1, 0.3f);
        }
    }
}

/**
 * The main movement controlling thread is within this function,
 * this is simply a state machine for the current state, and will
 * follow the current queue of moves assigned to the robot.
 */
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
                robotState = IDLE;
            }
            break;
        case STRAIGHTEN:

        case IDLE:
        default:
            if (!actionQueue.empty()) {
                Action action = actionQueue.front();
                actionQueue.pop();

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
                // Do nothing when no actions are queued
            }
            thread_sleep_for(10);
        }
    }
}

void mapUpdate() {
    float dF = us1.measureDistanceCm();
    float dB = us2.measureDistanceCm();
    float dLF = irBus.measureDistanceCm(0);
    float dLB = irBus.measureDistanceCm(1);
    float dRF = irBus.measureDistanceCm(2);
    float dRB = irBus.measureDistanceCm(3);

    mapInstance.updateGrid(dLF, dLB, dRF, dRB, dF, dB);
}

void syncMotors() {
    Serial.println("SYNCING");
    motor.resetCount();
    float encoder_difference_multiplier1;
    float encoder_difference_multiplier2;
    float circumference = 13.8 * PI;
    motor.updateMotors(1, 1, 0.5f, 0.5f);
    while ((fabs(motor.calculateDistanceA()) + fabs(motor.calculateDistanceB())) * 0.5 < circumference) {
        encoder_difference_multiplier1 = fabs((float)motor.encoderCountA) / fabs((float)motor.encoderCountB);
        Serial.println(encoder_difference_multiplier1);
    }
    motor.stopMotors();
    motor.resetCount();
    motor.updateMotors(0, 0, 0.5f, 0.5f);
    while ((fabs(motor.calculateDistanceA()) + fabs(motor.calculateDistanceB())) * 0.5 < circumference) {
        encoder_difference_multiplier2 = fabs((float)motor.encoderCountA) / fabs((float)motor.encoderCountB);
        Serial.println(encoder_difference_multiplier2);
    }
    motor.stopMotors();
    motor.speed_difference_fix = encoder_difference_multiplier1;
    motor.resetCount();
}

void setup() {
    Serial.begin(9600);

    delay(100);

    Serial.println("Started The Navigator");

    motor.setup();
    motor.startCounting();

    thread_sleep_for(1000);
    float dB = us2.measureDistanceCm();
    float dRB = irBus.measureDistanceCm(3);
    mapInstance.identifyStartPosition(dB, dRB);

    updateThread.start(update);

    Serial.println("Setup complete");

    thread_sleep_for(1000);
}

void loop() {
    mapUpdate();
    if (actionQueue.empty()) {
        if (mapInstance.isFrontBlocked()) {
            if (mapInstance.isLeftBlocked() && !mapInstance.isRightBlocked()) {
                enqueueAction(TURNING_RIGHT, 90, 0.5f);
            } else if (!mapInstance.isLeftBlocked() && mapInstance.isRightBlocked()) {
                enqueueAction(TURNING_LEFT, 90, 0.5f);
            } else {
                enqueueAction(TURNING_LEFT, 180, 0.5f);
            }
        } else if (!mapInstance.isLeftBlocked()) {
            enqueueAction(MOVING_FORWARD, 6, 0.5f);
            enqueueAction(TURNING_LEFT, 90, 0.5f);
        }
        enqueueAction(MOVING_FORWARD, 2, 0.5f);
    }
}