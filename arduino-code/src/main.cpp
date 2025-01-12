#include <Arduino.h>
#include <ArduinoBLE.h>
#include <rtos.h>

#include <GPY0E02B.h>
#include <HCSR04.h>
#include <Map.h>
#include <Motor.h>
#include <queue>

UltraSonicDistanceSensor us1(7); // Ultrasonic sensor 1, located on the front
UltraSonicDistanceSensor us2(6); // Ultrasonic sensor 2, located on the back

/*
 * IR 0 - left front sensor
 * IR 1 - left back sensor
 * IR 2 - right front sensor
 * IR 3 - right back sensor
 **/
IRSensor irBus;

/**
 * Motor A is the left side wheel
 * Motor B is the right side wheel
 */
Motor motor(P0_4, P0_5, P0_27, P1_2);

// Declarning Threads
rtos::Thread syncMotorsThread;
rtos::Thread updateThread;

Map mapInstance(150, 200, 1); // Initialise map

// Define all possible states
enum RobotState {
    IDLE,
    TURNING_LEFT,
    TURNING_RIGHT,
    MOVING_FORWARD,
    ALIGN
};

RobotState robotState = IDLE;
float targetDistance;
float currentDistance;

// Define an action struct to allow for movement planning
struct Action {
    RobotState state; // The type of action to perform
    float value;      // For turning this is the angle. For moving this is the distance
    float speed;      // Speed to perform action at, range 0.0f -> 1.0f
};

std::queue<Action> actionQueue; // Queue of actions to be performed sequentially.
bool useActionQueue = true;     // boolean to pause following the action queue, used for multi-step action routines (e.g. alignLeft, alignRight)

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
    useActionQueue = false;  // Pause queued actions
    const float ERROR = 0.3; // Provide a margain of error to allow, accounting for sensor varience
    float distanceLeftFront = irBus.measureDistanceCm(0);
    float distanceLeftBack = irBus.measureDistanceCm(1);

    while (distanceLeftFront != distanceLeftBack) {
        distanceLeftFront = irBus.measureDistanceCm(0);
        distanceLeftBack = irBus.measureDistanceCm(1);
        if (distanceLeftFront == distanceLeftBack) {
            break;
        }
        if (distanceLeftFront > distanceLeftBack + ERROR) {
            turnLeft(1, 0.25f); // correct for left front being too far away
        } else if (distanceLeftBack > distanceLeftFront + ERROR) {
            turnRight(1, 0.25f); // correct for left back being too far away
        }
        while (robotState == TURNING_LEFT || robotState == TURNING_RIGHT) {
            // wait for move to finish
            thread_sleep_for(1);
        }
    }
    Serial.println("Alignment complete. Now aligned to left side wall");
    useActionQueue = true; // Resume queued actions
}

void alignRight() {
    useActionQueue = false;  // Pause queued actions
    const float ERROR = 0.3; // Provide a margain of error to allow, accounting for sensor varience
    float distanceRightFront = irBus.measureDistanceCm(2);
    float distanceRightBack = irBus.measureDistanceCm(3);

    while (distanceRightFront != distanceRightBack) {
        distanceRightFront = irBus.measureDistanceCm(2);
        distanceRightBack = irBus.measureDistanceCm(3);
        if (distanceRightFront == distanceRightBack) {
            break;
        }
        if (distanceRightFront > distanceRightBack + ERROR) {
            turnRight(1, 0.25f); // correct for right front being too far away
        } else if (distanceRightBack > distanceRightFront + ERROR) {
            turnLeft(1, 0.25f); // correct for right back being too far away
        }
        while (robotState == TURNING_LEFT || robotState == TURNING_RIGHT) {
            // wait for move to finish
            thread_sleep_for(1);
        }
    }
    Serial.println("Alignment complete. Now aligned to right side wall");
    useActionQueue = true; // Resume queued actions
}

/**
 * @brief Controls the state machine responsible for movement.
 * Intended to run within a thread as this runs a continuous loop.
 */
void update() {
    while (true) {
        // update encoder distance measurements
        float distanceMotorA = motor.calculateDistanceA();
        float distanceMotorB = motor.calculateDistanceB();

        switch (robotState) {
        case TURNING_LEFT: // Fall-through due to both having identical logic
        case TURNING_RIGHT:
            currentDistance = (abs(distanceMotorA) + abs(distanceMotorB)) / 2;
            if (currentDistance >= targetDistance) {
                motor.stopMotors();
                thread_sleep_for(10);
                motor.resetCount();
                Serial.println("Turn complete.");
                robotState = IDLE;
            } else {
                motor.syncMotors();
            }
            break;

        case MOVING_FORWARD:
            currentDistance = (abs(distanceMotorA) + abs(distanceMotorB)) / 2;
            if (currentDistance >= targetDistance) {
                motor.stopMotors();
                thread_sleep_for(10);
                motor.resetCount();
                Serial.println("Move complete.");
                robotState = IDLE;
            } else {
                motor.syncMotors();
            }
            break;

        case ALIGN:

        case IDLE:
        default:
            if (!actionQueue.empty() && useActionQueue) {
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
            }
            // Do nothing when no actions are queued
        }
        thread_sleep_for(10); // short delay before looping
    }
}

/**
 * @brief Check all sensors, and update map based upon values
 */
void mapUpdate() {
    float distanceFront = us1.measureDistanceCm();
    float distanceBack = us2.measureDistanceCm();
    float distanceLeftFront = irBus.measureDistanceCm(0);
    float distanceLeftBack = irBus.measureDistanceCm(1);
    float distanceRightFront = irBus.measureDistanceCm(2);
    float distanceRightBack = irBus.measureDistanceCm(3);
    mapInstance.updateGrid(distanceLeftFront, distanceLeftBack, distanceRightFront, distanceRightBack, distanceFront, distanceBack);
}

/**
 * @brief Built in arduino setup function
 */
void setup() {
    Serial.begin(9600);

    delay(100);

    Serial.println("Started The Navigator");

    motor.setup();
    motor.startCounting();

    thread_sleep_for(1000);
    float distanceBack = us2.measureDistanceCm();
    float distanceRightBack = irBus.measureDistanceCm(3);
    mapInstance.identifyStartPosition(distanceBack, distanceRightBack);

    updateThread.start(update);

    Serial.println("Setup complete");

    thread_sleep_for(1000);
}

/**
 * @brief Built in arduino loop function
 */
void loop() {
    /*mapUpdate();
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
    }*/
    mapInstance.setRobotPosition(50, mapInstance.getRobotY()+5, 180);
    Serial.println(mapInstance.isRobotAtFinish());
    thread_sleep_for(100);
}