#include <Arduino.h>
#include <ArduinoBLE.h>
#include <rtos.h>

#include <GPY0E02B.h>
#include <HCSR04.h>
#include <Map.h>
#include <Motor.h>
#include <Util.h>
#include <queue>
#include <stack>

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

// Util util;

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
std::stack<Action> actionStack; // List of all actions to hopefully repeat.
bool useActionQueue = true;     // boolean to pause following the action queue, used for multi-step action routines (e.g. alignLeft, alignRight)

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
 * @brief
 *
 * @param state The type of action to perform
 * @param value For turning this is the angle. For moving this is the distance
 * @param speed The speed to perform action at, range 0.0f -> 1.0f
 */
void pushActionToStack(RobotState state, float value, float speed) {
    actionStack.push({state, value, speed});
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

    mbed::Timeout timeout;
    bool timeoutOccurred;
    Util::beginTimeout(timeout, timeoutOccurred, 5.0);
    while (currentDistance < partialCircumference) {
        if (timeoutOccurred) {
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
        mapInstance.rotateRobotLeft(angle);
        pushActionToStack(TURNING_LEFT, angle, speed);
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

    mbed::Timeout timeout;
    bool timeoutOccurred;
    Util::beginTimeout(timeout, timeoutOccurred, 5.0);
    while (currentDistance < partialCircumference) {
        if (timeoutOccurred) {
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
        pushActionToStack(TURNING_RIGHT, angle, speed);
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

    mbed::Timeout timeout;
    bool timeoutOccurred;
    Util::beginTimeout(timeout, timeoutOccurred, 5.0);
    while (currentDistance < targetDistance) {
        if (timeoutOccurred) {
            moveBackward(2, 0.5f);
            align();
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
    pushActionToStack(MOVING_FORWARD, currentDistance, speed);
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

    mbed::Timeout timeout;
    bool timeoutOccurred;
    Util::beginTimeout(timeout, timeoutOccurred, 5.0);
    while (currentDistance < targetDistance) {
        if (timeoutOccurred) {
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
    pushActionToStack(MOVING_BACKWARD, currentDistance, speed);
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
    mbed::Timeout timeout;
    bool timeoutOccurred;
    Util::beginTimeout(timeout, timeoutOccurred, 5.0);
    // while the back and front are not equal,
    while (distanceLeftFront != distanceLeftBack) {
        if (timeoutOccurred) {
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
    mbed::Timeout timeout;
    bool timeoutOccurred;
    Util::beginTimeout(timeout, timeoutOccurred, 5.0);
    // while the back and front are not equal,
    while (distanceRightFront != distanceRightBack) {
        if (timeoutOccurred) {
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
                align();
                break;
            case MOVING_BACKWARD:
                moveBackward(action.value, action.speed);
                align();
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

void reverseStack() {
    turnRight(180, 0.5f);
    while (true) {
        if (robotState == IDLE && !actionStack.empty()) {
            Action action = actionStack.top();
            actionQueue.pop();

            switch (action.state) {
            case MOVING_FORWARD:
                moveForward(action.value, action.speed);
                align();
                break;
            case MOVING_BACKWARD:
                moveBackward(action.value, action.speed);
                align();
                break;
            case TURNING_LEFT:
                turnRight(action.value, action.speed);
                align();
                break;
            case TURNING_RIGHT:
                turnLeft(action.value, action.speed);
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
    bool frontTouchingWall, leftFrontClear, leftBackClear, rightFrontClear, rightBackClear;

    while (true) {
        distanceFront = us1.measureDistanceCm();
        distanceBack = us2.measureDistanceCm();
        distanceLeftFront = irBus.measureDistanceCm(0);
        distanceLeftBack = irBus.measureDistanceCm(1);
        distanceRightFront = irBus.measureDistanceCm(2);
        distanceRightBack = irBus.measureDistanceCm(3);
        frontTouchingWall = distanceFront < FAR_TOO_CLOSE_DISTANCE || distanceFront == US_FAIL;
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
        bool rightTurnToOrientToGoal = robotAngleDeg == 0;
        bool onLeftSide = robotX > 100; // relative height of map

        mapInstance.updateGrid(distanceLeftFront, distanceLeftBack, distanceRightFront, distanceRightBack, distanceFront, distanceBack);
        if (mapInstance.isRobotAtFinish()) {
            reverseStack();
            break;
        }
        if (frontTouchingWall) {
            moveBackward(4, 0.5f);
            align();
        } else {
            if (facingTowardsGoal) {
                if (frontClear) {
                    // front clear towards goal -> move forward towards goal
                    align();
                    moveForward(8, 0.5f);
                } else if (leftFrontClear && leftBackClear) {
                    // front not clear -> turn to find a left side wall
                    turnLeft(90, 0.5f);
                    distanceFront = us1.measureDistanceCm();
                    if (frontClear) {
                        // Clear in front -> move forward
                        moveForward(26, 0.5f);
                    }
                    align();
                } else if (rightFrontClear && rightBackClear) {
                    // front not clear and too close to left side -> turn right
                    turnRight(90, 0.5f);
                    distanceFront = us1.measureDistanceCm();
                    if (frontClear) {
                        // Clear in front -> move forward
                        moveForward(26, 0.5f);
                    }
                } else {
                    // Fully blocked on front, left and right -> turn 180
                    turnRight(180, 0.5f);
                    align();
                }
            } else if (leftTurnToOrientToGoal) {
                if (leftFrontClear && leftBackClear) {
                    // able to rotate to face goal on left -> turn left
                    turnLeft(90, 0.5f);
                    distanceFront = us1.measureDistanceCm();
                    if (frontClear) {
                        // Clear in front -> move forward
                        moveForward(26, 0.5f);
                    }
                } else if (leftFrontClear && !leftBackClear) {
                    // Close to clearing left wall -> move forward enought to fully clear -> turn left
                    moveForward(18, 0.5f);
                    turnLeft(90, 0.5f);
                    distanceFront = us1.measureDistanceCm();
                    if (frontClear) {
                        // Clear in front -> move forward
                        moveForward(26, 0.5f);
                    }
                    align();
                } else if (!leftFrontClear && leftBackClear) {
                    moveForward(18, 0.5f);
                    align();
                } else {
                    if (frontClear) {
                        moveForward(8, 0.5f);
                        align();
                    } else {
                        if (rightFrontClear && rightBackClear) {
                            turnRight(90, 0.5f);
                            distanceFront = us1.measureDistanceCm();
                            if (frontClear) {
                                // Clear in front -> move forward
                                moveForward(26, 0.5f);
                            }
                        } else if (rightFrontClear && !rightBackClear) {
                            moveForward(18, 0.5f);
                            align();
                        } else {
                            // Fully blocked on front, left and right -> turn 180
                            turnRight(180, 0.5f);
                            align();
                        }
                    }
                }
            } else if (rightTurnToOrientToGoal) {
                if (frontClear) {
                    moveForward(8, 0.5f);
                } else {
                    if (rightFrontClear && rightBackClear) {
                        turnRight(90, 0.5f);
                        distanceFront = us1.measureDistanceCm();
                        if (frontClear) {
                            // Clear in front -> move forward
                            moveForward(26, 0.5f);
                        }
                    } else if (rightFrontClear && !rightBackClear) {
                        moveForward(18, 0.5f);
                    } else {
                        // Fully blocked on front, left and right -> turn 180
                        turnRight(180, 0.5f);
                        align();
                    }
                }
            } else {
                if (rightFrontClear && rightBackClear) {
                    turnRight(90, 0.5f);
                    distanceFront = us1.measureDistanceCm();
                    if (frontClear) {
                        // Clear in front -> move forward
                        moveForward(26, 0.5f);
                    }
                    align();
                } else if (rightFrontClear && !rightBackClear) {
                    moveForward(18, 0.5f);
                    align();
                } else {
                    if (leftFrontClear && leftBackClear) {
                        // able to rotate to face goal on left -> turn left
                        turnLeft(90, 0.5f);
                        distanceFront = us1.measureDistanceCm();
                        if (frontClear) {
                            // Clear in front -> move forward
                            moveForward(26, 0.5f);
                        }
                        align();
                    } else if (leftFrontClear && !leftBackClear) {
                        // Close to clearing left wall -> move forward enought to fully clear -> turn left
                        moveForward(18, 0.5f);
                        turnLeft(90, 0.5f);
                        distanceFront = us1.measureDistanceCm();
                        if (frontClear) {
                            // Clear in front -> move forward
                            moveForward(26, 0.5f);
                        }
                        align();
                    } else if (!leftFrontClear && leftBackClear) {
                        moveForward(18, 0.5f);
                        align();
                    } else {
                        if (frontClear) {
                            moveForward(8, 0.5f);
                            align();
                        } else {
                            turnRight(180, 0.5f);
                            align();
                        }
                    }
                }
            }
        }
    }

    /*if (frontTouchingWall) {
        // Blocked on front -> move backwards
        moveBackward(4, 0.5f);
        align();
    } else {
        if (facingTowardsGoal && frontClear) {
            // Clear in front -> move forward
            align();
            moveForward(10, 0.5f);
        } else {
            if (leftFrontClear && leftBackClear && (leftTurnToOrientToGoal || !onLeftSide) && !rightTurnToOrientToGoal) {
                // Clear on left -> turn left
                turnLeft(90, 0.5f);
                distanceFront = us1.measureDistanceCm();
                if (frontClear) {
                    // Clear in front -> move forward
                    moveForward(26, 0.5f);
                }
                align();
            } else if (leftFrontClear && !leftBackClear && (leftTurnToOrientToGoal || !onLeftSide || !rightTurnToOrientToGoal)) {
                // Close to clearing left wall -> move forward enought to fully clear -> turn left
                moveForward(18, 0.5f);
                turnLeft(90, 0.5f);
                distanceFront = us1.measureDistanceCm();
                if (frontClear) {
                    // Clear in front -> move forward
                    moveForward(26, 0.5f);
                }
            } else if (!leftFrontClear && leftBackClear && (leftTurnToOrientToGoal || !onLeftSide || !rightTurnToOrientToGoal)) {
                // Close to being parallel with new left side wall -> move forward to be beside it
                moveForward(16, 0.5f);
            } else {
                // Blocked on left
                // Check if the front is clear enough to move forward
                if (frontClear) {
                    // Clear in front -> move forward
                    align();
                    moveForward(8, 0.5f);
                } else {
                    if (rightFrontClear && rightBackClear) {
                        // Clear on right -> turn right
                        turnRight(90, 0.5f);
                        distanceFront = us1.measureDistanceCm();
                        if (frontClear) {
                            // Clear in front -> move forward
                            moveForward(26, 0.5f);
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
    }*/
    thread_sleep_for(10);
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

    // updateThread.start(update);

    Serial.println("Setup complete");

    thread_sleep_for(1000);
}

/**
 * @brief Built in arduino loop function
 */
void loop() {
    explore();
    thread_sleep_for(1000);
}