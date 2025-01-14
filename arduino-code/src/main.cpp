#include <Arduino.h>
#include <ArduinoBLE.h>
#include <rtos.h>

#include <GPY0E02B.h>
#include <HCSR04.h>
#include <Map.h>
#include <Motor.h>
#include <list>
#include <queue>

UltraSonicDistanceSensor us1(7); // Ultrasonic sensor 1, located on the front
UltraSonicDistanceSensor us2(6); // Ultrasonic sensor 2, located on the back

/**
 * IR 0 - left front sensor
 * IR 1 - left back sensor
 * IR 2 - right front sensor
 * IR 3 - right back sensor
 */
IRSensor irBus;

/**
 * Motor A is the left side wheel
 * Motor B is the right side wheel
 */
Motor motor(P0_4, P0_5, P0_27, P1_2);

rtos::Thread updateThread; // Dedicated update thread for

Map mapInstance(150, 200, 1); // Initialise map

// Define all possible states
enum RobotState {
    IDLE,
    MOVING_FORWARD,
    MOVING_BACKWARD,
    TURNING_LEFT,
    TURNING_RIGHT
};

RobotState robotState = IDLE;

// Define an action struct to allow for movement planning
struct Action {
    RobotState state; // The type of action to perform
    float value;      // For turning this is the angle. For moving this is the distance
    float speed;      // The speed to perform action at, range 0.0f -> 1.0f
};

std::queue<Action> actionQueue; // Queue of actions to be performed sequentially.
std::list<Action> actionList;   // List of all actions to hopefully repeat.
bool useActionQueue = true;     // boolean to pause following the action queue, used for multi-step action routines (e.g. alignLeft, alignRight)

rtos::Thread timoutThread;
volatile bool timeout_occurred = false;
float timeout_duration;
void timeoutTask() {
    thread_sleep_for(int(timeout_duration * 1000));
    timeout_occurred = true;
}
void beginTimeout(float time) {
    timeout_duration = time;
    timoutThread.terminate();
    timoutThread.start(mbed::callback(timeoutTask));
}

// --- Prototype functions ---
void moveForward(float distance, float speed);
void moveBackward(float distance, float speed);
void align();

/**
 * @brief
 *
 * @param state The type of action to perform
 * @param value For turning this is the angle. For moving this is the distance
 * @param speed The speed to perform action at, range 0.0f -> 1.0f
 */
void enqueueAction(RobotState state, float value, float speed) {
    actionQueue.push({state, value, speed});
    Serial.println((String) "Addeed:" + state + " to action queue");
}

/**
 * @brief Turn left a set amount of degrees
 * @param angle in degrees (0-359)
 * @param speed The speed to perform action at, range 0.0f -> 1.0f
 */
void turnLeft(float angle, float speed) {
    Serial.println((String) "Attempting to turn left " + angle + " degrees...");

    float partialCircumference = (angle / 360) * 13.8 * PI;
    float currentDistance = 0;
    float previousDistance = 0;
    float distanceMotorA = motor.calculateDistanceA();
    float distanceMotorB = motor.calculateDistanceB();

    motor.resetCount();
    motor.updateMotors(1, 1, speed, speed);

    beginTimeout(5.0);
    while (currentDistance < partialCircumference) {
        if (timeout_occurred) {
            Serial.println("Timeout reached, exiting the loop.");
            break;
        }
        previousDistance = currentDistance;
        thread_sleep_for(10);
        distanceMotorA = motor.calculateDistanceA();
        distanceMotorB = motor.calculateDistanceB();
        currentDistance = (abs(distanceMotorA) + abs(distanceMotorB)) / 2;
        if (previousDistance == currentDistance) {
            break;
        }
        motor.syncMotors();
        thread_sleep_for(10);
    }
    motor.stopMotors();
    thread_sleep_for(10);
    motor.resetCount();
    if (abs(angle) != 1) {
        mapInstance.rotateRobotRight(angle);
    }
    Serial.println("Turn complete.");
    robotState = IDLE;
}

/**
 * @brief Turn right a set amount of degrees
 * @param angle in degrees (0-359)
 * @param speed The speed to perform action at, range 0.0f -> 1.0f
 */
void turnRight(float angle, float speed) {
    Serial.println((String) "Attempting to turn right " + angle + " degrees...");

    float partialCircumference = (angle / 360) * 13.8 * PI;
    float currentDistance = 0;
    float previousDistance = 0;
    float distanceMotorA = motor.calculateDistanceA();
    float distanceMotorB = motor.calculateDistanceB();

    motor.resetCount();
    motor.updateMotors(0, 0, speed, speed);

    beginTimeout(5.0);
    while (currentDistance < partialCircumference) {
        if (timeout_occurred) {
            Serial.println("Timeout reached, exiting the loop.");
            break;
        }
        previousDistance = currentDistance;
        thread_sleep_for(10);
        distanceMotorA = motor.calculateDistanceA();
        distanceMotorB = motor.calculateDistanceB();
        currentDistance = (abs(distanceMotorA) + abs(distanceMotorB)) / 2;
        if (previousDistance == currentDistance) {
            break;
        }
        motor.syncMotors();
        thread_sleep_for(10);
    }

    motor.stopMotors();
    thread_sleep_for(10);
    motor.resetCount();
    if (abs(angle) != 1) {
        mapInstance.rotateRobotRight(angle);
    }
    Serial.println("Turn complete.");
    robotState = IDLE;
}

/**
 * @brief Move forward a set distance and speed
 * @param distance in cm
 * @param speed The speed to perform action at, range 0.0f -> 1.0f
 */
void moveForward(float distance, float speed) {
    Serial.println((String) "Attempting to move forward " + distance + " cm...");

    float targetDistance = distance;
    float currentDistance = 0;
    float previousDistance = 0;
    float distanceMotorA = motor.calculateDistanceA();
    float distanceMotorB = motor.calculateDistanceB();

    motor.resetCount();
    motor.updateMotors(0, 1, speed, speed);

    beginTimeout(5.0);
    while (currentDistance < targetDistance) {
        if (timeout_occurred) {
            Serial.println("Timeout reached, exiting the loop.");
            break;
        }
        previousDistance = currentDistance;
        thread_sleep_for(10);
        distanceMotorA = motor.calculateDistanceA();
        distanceMotorB = motor.calculateDistanceB();
        currentDistance = (abs(distanceMotorA) + abs(distanceMotorB)) / 2;
        if (currentDistance == previousDistance) {
            break;
        }
        motor.syncMotors();
    }

    motor.stopMotors();
    thread_sleep_for(10);
    motor.resetCount();
    mapInstance.moveRobotForward(currentDistance);
    Serial.println("Move complete.");
    robotState = IDLE;
}

/**
 * @brief Move backwards a set distance and speed
 * @param distance To travel in cm
 * @param speed The speed to perform action at, range 0.0f -> 1.0f
 */
void moveBackward(float distance, float speed) {
    Serial.println((String) "Attempting to move forward " + distance + " cm...");

    float targetDistance = distance;
    float currentDistance = 0;
    float previousDistance = 0;
    float distanceMotorA = motor.calculateDistanceA();
    float distanceMotorB = motor.calculateDistanceB();

    motor.resetCount();
    motor.updateMotors(1, 0, speed, speed);

    beginTimeout(5.0);
    while (currentDistance < targetDistance) {
        if (timeout_occurred) {
            Serial.println("Timeout reached, exiting the loop.");
            break;
        }
        previousDistance = currentDistance;
        thread_sleep_for(10);
        distanceMotorA = motor.calculateDistanceA();
        distanceMotorB = motor.calculateDistanceB();
        currentDistance = (abs(distanceMotorA) + abs(distanceMotorB)) / 2;
        if (previousDistance == currentDistance) {
            break;
        }
        motor.syncMotors();
    }

    motor.stopMotors();
    thread_sleep_for(10);
    motor.resetCount();
    mapInstance.moveRobotForward(currentDistance);
    Serial.println("Move complete.");
    robotState = IDLE;
}

/**
 * @brief Align to left side wall
 */
void alignLeft() {
    useActionQueue = false;  // Pause queued actions
    const float ERROR = 0.3; // Provide a margain of error to allow, accounting for sensor varience
    float distanceLeftFront = irBus.measureDistanceCm(0);
    float distanceLeftBack = irBus.measureDistanceCm(1);
    digitalWrite(LEDB, LOW); // Blue LED to show that its aligning
    beginTimeout(5.0);
    // while the back and front are not equal,
    while (distanceLeftFront != distanceLeftBack) {
        if (timeout_occurred) {
            Serial.println("Timeout reached, exiting the loop.");
            break;
        }

        distanceLeftFront = irBus.measureDistanceCm(0);
        distanceLeftBack = irBus.measureDistanceCm(1);
        if (abs(abs(distanceLeftFront) - abs(distanceLeftBack)) < ERROR) {
            break;
        }
        if (distanceLeftFront > distanceLeftBack + ERROR) {
            turnLeft(1, 0.25f); // correct for left front being too far away
        } else if (distanceLeftBack > distanceLeftFront + ERROR) {
            turnRight(1, 0.25f); // correct for left back being too far away
        }
    }

    digitalWrite(LEDB, HIGH);
    Serial.println("Alignment complete. Now aligned to left side wall");
    useActionQueue = true; // Resume queued actions
}

/**
 * @brief Align to right side wall
 */
void alignRight() {
    useActionQueue = false;  // Pause queued actions
    const float ERROR = 0.3; // Provide a margain of error to allow, accounting for sensor varience
    float distanceRightFront = irBus.measureDistanceCm(2);
    float distanceRightBack = irBus.measureDistanceCm(3);
    digitalWrite(LEDB, LOW); // Blue LED to show that its aligning
    beginTimeout(5.0);
    // while the back and front are not equal,
    while (distanceRightFront != distanceRightBack) {
        if (timeout_occurred) {
            Serial.println("Timeout reached, exiting the loop.");
            break;
        }
        distanceRightFront = irBus.measureDistanceCm(2);
        distanceRightBack = irBus.measureDistanceCm(3);
        if (abs(abs(distanceRightFront) - abs(distanceRightBack)) < ERROR) {
            break;
        }
        if (distanceRightFront > distanceRightBack + ERROR) {
            turnRight(1, 0.25f); // correct for right front being too far away
        } else if (distanceRightBack > distanceRightFront + ERROR) {
            turnLeft(1, 0.25f); // correct for right back being too far away
        }
    }

    digitalWrite(LEDB, HIGH);
    Serial.println("Alignment complete. Now aligned to right side wall");
    useActionQueue = true; // Resume queued actions
}

/**
 * @brief Chose the closest viable wall to align to.
 */
void align() {
    float distanceLeftFront = irBus.measureDistanceCm(0);
    float distanceLeftBack = irBus.measureDistanceCm(1);
    float distanceRightFront = irBus.measureDistanceCm(2);
    float distanceRightBack = irBus.measureDistanceCm(3);

    const float significantError = 20;

    if (distanceLeftFront < distanceRightFront && (abs(distanceLeftFront - distanceLeftBack) < significantError && distanceLeftFront < significantError && distanceLeftBack < significantError)) {
        // if left side is closer AND if left side doesn't have too large of a difference.
        alignLeft();
    } else if (distanceLeftFront > distanceRightFront && (abs(distanceRightFront - distanceRightBack) < significantError && distanceRightFront < significantError && distanceRightBack < significantError)) {
        // if right side is closer AND if right side doesn't have too large of a difference.
        alignRight();
    }
}

/**
 * @brief The state machine (of sorts) handling the movement for when using the queue. Intended to be ran on a dedicated thread.
 */
void update() {
    while (true) {
        if (robotState == IDLE && !actionQueue.empty() && useActionQueue) {
            Action action = actionQueue.front();
            actionQueue.pop();

            switch (action.state) {
            case MOVING_FORWARD:
                moveForward(action.value, action.speed);
                break;
            case MOVING_BACKWARD:
                moveBackward(action.value, action.speed);
                break;
            case TURNING_LEFT:
                turnLeft(action.value, action.speed);
                align();
                break;
            case TURNING_RIGHT:
                turnRight(action.value, action.speed);
                align();
                break;
            default:
                break;
            }
        }
        thread_sleep_for(10); // Short delay before looping
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
    mapInstance.isRobotAtFinish();
}

/**
 * @brief Check all sensors, and update map based upon values
 */
void explore() {
    float distanceFront, distanceBack, distanceLeftFront, distanceLeftBack, distanceRightFront, distanceRightBack;
    const float CLOSE_TO_SIDE_WALL_DISTANCE = 22;
    const float CLOSE_TO_FRONT_WALL_DISTANCE = CLOSE_TO_SIDE_WALL_DISTANCE + 2.5;
    const float FAR_TOO_CLOSE_DISTANCE = 3.0;
    const float US_FAIL = -1;
    bool frontNotTouchingWall, leftFrontClear, leftBackClear, rightFrontClear, rightBackClear;

    while (true) {
        distanceFront = us1.measureDistanceCm();
        distanceBack = us2.measureDistanceCm();
        distanceLeftFront = irBus.measureDistanceCm(0);
        distanceLeftBack = irBus.measureDistanceCm(1);
        distanceRightFront = irBus.measureDistanceCm(2);
        distanceRightBack = irBus.measureDistanceCm(3);
        frontNotTouchingWall = !(distanceFront < FAR_TOO_CLOSE_DISTANCE || distanceFront == US_FAIL);
        bool frontClear = distanceFront > CLOSE_TO_FRONT_WALL_DISTANCE;
        leftFrontClear = !(distanceLeftFront < CLOSE_TO_SIDE_WALL_DISTANCE);
        leftBackClear = !(distanceLeftBack < CLOSE_TO_SIDE_WALL_DISTANCE);
        rightFrontClear = !(distanceRightFront < CLOSE_TO_SIDE_WALL_DISTANCE);
        rightBackClear = !(distanceRightBack < CLOSE_TO_SIDE_WALL_DISTANCE);
        bool isLeftDiffBlockWidth = abs(distanceLeftFront - distanceLeftBack) < 10;
        bool isRightDiffBlockWidth = abs(distanceRightFront - distanceRightBack) < 10;

        float robotAngleDeg = mapInstance.getRobotAngle();
        int robotX = mapInstance.getRobotX();
        int robotY = mapInstance.getRobotY();
        bool facingTowardsGoal = robotAngleDeg == 90;
        bool leftTurnToOrientToGoal = robotAngleDeg == 180;
        bool onLeftSide = robotX > 100; // relative height of map

        mapInstance.updateGrid(distanceLeftFront, distanceLeftBack, distanceRightFront, distanceRightBack, distanceFront, distanceBack);
        mapInstance.isRobotAtFinish();

        if (!frontNotTouchingWall) {
            // Blocked on front -> move backwards
            moveBackward(4, 0.5f);
            align();
        } else {
            if (facingTowardsGoal && frontClear) {
                // Clear in front -> move forward
                align();
                moveForward(10, 0.5f);
            } else {
                if (leftFrontClear && leftBackClear && (!onLeftSide || leftTurnToOrientToGoal)) {
                    // Clear on left -> turn left
                    turnLeft(90, 0.5f);
                    distanceFront = us1.measureDistanceCm();
                    if (frontClear) {
                        // Clear in front -> move forward
                        moveForward(25, 0.5f);
                    }
                    align();
                } else if (leftFrontClear && !leftBackClear && !isLeftDiffBlockWidth && (!onLeftSide || leftTurnToOrientToGoal)) {
                    // Close to clearing left wall -> move forward enought to fully clear -> turn left
                    moveForward(18, 0.5f);
                    turnLeft(90, 0.5f);
                    distanceFront = us1.measureDistanceCm();
                    if (frontClear) {
                        // Clear in front -> move forward
                        moveForward(25, 0.5f);
                    }
                } else if (!leftFrontClear && leftBackClear) {
                    // Close to being parallel with new left side wall -> move forward to be beside it
                    moveForward(16, 0.5f);
                } else {
                    // Blocked on left
                    // Check if the front is clear enough to move forward
                    if (frontClear) {
                        // Clear in front -> move forward
                        align();
                        moveForward(10, 0.5f);
                    } else {
                        if (rightFrontClear && rightBackClear) {
                            // Clear on right -> turn right
                            turnRight(90, 0.5f);
                            distanceFront = us1.measureDistanceCm();
                            if (frontClear) {
                                // Clear in front -> move forward
                                moveForward(25, 0.5f);
                            }
                            align();
                        } else if (rightFrontClear && !rightBackClear) {
                            // Close to clearing right wall -> move forward
                            moveForward(18, 0.5f);
                        } else {
                            // Fully blocked on front, left and right -> turn 180
                            turnRight(180, 0.5f);
                            align();
                        }
                    }
                }
            }
        }
        thread_sleep_for(100);
    }
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
    align();
    mapInstance.identifyStartPosition(distanceBack, distanceRightBack);

    updateThread.start(update);

    Serial.println("Setup complete");

    thread_sleep_for(1000);
}

/**
 * @brief Built in arduino loop function
 */
void loop() {
    explore();
    thread_sleep_for(10);
}