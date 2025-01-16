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

rtos::Thread performQueueThread; // Dedicated thread for when the performQueue method is being used

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

    bool operator==(const Action &other) const {
        return state == other.state && value == other.value && speed == other.speed;
    }
};

std::queue<Action> actionQueue; // Queue of actions to be performed sequentially.
std::stack<Action> actionStack; // List of all actions to hopefully repeat.
bool useActionQueue = true;     // Boolean to pause following the action queue, used for multi-step action routines (e.g. alignLeft, alignRight)
int actionStackLength = 0;      // Variable to keep track of the stack length

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
    actionStackLength++;
    actionStack.push({state, value, speed});
    Serial.println((String) "Addeed:" + state + " to action queue");
}

/**
 * @brief Combine back-to-back actions of the same type.
 *
 * @param actionStack the stack to perform the operation upon
 */
std::stack<Action> combineActions(std::stack<Action> actionStack) {
    std::stack<Action> tempStack;
    // Transfer elements from stack to tempStack
    while (!actionStack.empty()) {
        tempStack.push(actionStack.top());
        actionStack.pop();
    }
    // Combine consecutive actions of the same state
    while (!tempStack.empty()) {
        Action currentAction = tempStack.top();
        tempStack.pop();
        while (!tempStack.empty() && tempStack.top().state == currentAction.state) {
            Action nextAction = tempStack.top();
            tempStack.pop();
            currentAction.value += nextAction.value;
            currentAction.speed = nextAction.speed;
        }
        actionStack.push(currentAction);
    }
    return actionStack;
}

/**
 * @brief Turn left a set amount of degrees
 * @param angle in degrees (0-359)
 * @param speed The speed to perform action at, range 0.0f -> 1.0f
 */
void turnLeft(float angle, float speed) {
    // Serial.println((String) "Attempting to turn left " + angle + " degrees...");

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
            motor.stopMotors();
            motor.resetCount();
            robotState = IDLE;
            return;
        }
        previousDistance = currentDistance;
        thread_sleep_for(10);
        distanceMotorA = motor.calculateDistanceA();
        distanceMotorB = motor.calculateDistanceB();
        currentDistance = (abs(distanceMotorA) + abs(distanceMotorB)) / 2;
        if (previousDistance == currentDistance) {
            break; // Break if not moving, probably stuck
        }
        motor.syncMotors();
        thread_sleep_for(10);
    }
    motor.stopMotors();
    thread_sleep_for(10);
    motor.resetCount();
    if (abs(angle) != 1) {
        // Discount all mapping that is for wall alignment
        mapInstance.rotateRobotLeft(angle);
        pushActionToStack(TURNING_LEFT, angle, speed);
    }
    // Serial.println("Turn complete.");
    robotState = IDLE;
}

/**
 * @brief Turn right a set amount of degrees
 * @param angle in degrees (0-359)
 * @param speed The speed to perform action at, range 0.0f -> 1.0f
 */
void turnRight(float angle, float speed) {
    // Serial.println((String) "Attempting to turn right " + angle + " degrees...");

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
            motor.stopMotors();
            motor.resetCount();
            robotState = IDLE;
            return;
        }
        previousDistance = currentDistance;
        thread_sleep_for(10);
        distanceMotorA = motor.calculateDistanceA();
        distanceMotorB = motor.calculateDistanceB();
        currentDistance = (abs(distanceMotorA) + abs(distanceMotorB)) / 2;
        if (previousDistance == currentDistance) {
            break; // Break if not moving, probably stuck
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
    // Serial.println("Turn complete.");
    robotState = IDLE;
}

/**
 * @brief Move forward a set distance and speed
 * @param distance in cm
 * @param speed The speed to perform action at, range 0.0f -> 1.0f
 */
void moveForward(float distance, float speed) {
    // Serial.println((String) "Attempting to move forward " + distance + " cm...");

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
            // Stuck -> move backwards -> check sensors and turn & move to avoid the obstacle blocking
            moveBackward(2, 0.5f);
            align();
            if (irBus.measureDistanceCm(0) > 10) {
                turnLeft(90, 0.5f);
                moveForward(2, 0.5f);
            } else if (irBus.measureDistanceCm(2) > 10) {
                turnRight(90, 0.5f);
                moveForward(2, 0.5f);
            } else {
                turnRight(180, 0.5f);
            }
            break;
        }
        previousDistance = currentDistance;
        thread_sleep_for(10);
        distanceMotorA = motor.calculateDistanceA();
        distanceMotorB = motor.calculateDistanceB();
        currentDistance = (abs(distanceMotorA) + abs(distanceMotorB)) / 2;
        if (currentDistance == previousDistance) {
            break; // Break if not moving, probably stuck
        }
        motor.syncMotors();
    }

    motor.stopMotors();
    thread_sleep_for(10);
    motor.resetCount();
    mapInstance.moveRobotForward(currentDistance);
    pushActionToStack(MOVING_FORWARD, currentDistance, speed);
    // Serial.println("Move complete.");
    robotState = IDLE;
}

/**
 * @brief Move backwards a set distance and speed
 * @param distance To travel in cm
 * @param speed The speed to perform action at, range 0.0f -> 1.0f
 */
void moveBackward(float distance, float speed) {
    // Serial.println((String) "Attempting to move forward " + distance + " cm...");

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
            break; // Break if not moving, probably stuck
        }
        motor.syncMotors();
    }

    motor.stopMotors();
    thread_sleep_for(10);
    motor.resetCount();
    mapInstance.moveRobotForward(currentDistance);
    pushActionToStack(MOVING_BACKWARD, currentDistance, speed);
    // Serial.println("Move complete.");
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
        if (distanceLeftFront > distanceLeftBack) {
            turnLeft(1, 0.25f); // correct for left front being too far away
        } else if (distanceLeftBack > distanceLeftFront) {
            turnRight(1, 0.25f); // correct for left back being too far away
        }
    }

    digitalWrite(LEDB, HIGH);
    // Serial.println("Alignment complete. Now aligned to left side wall");
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
        if (distanceRightFront > distanceRightBack) {
            turnRight(1, 0.25f); // correct for right front being too far away
        } else if (distanceRightBack > distanceRightFront) {
            turnLeft(1, 0.25f); // correct for right back being too far away
        }
    }

    digitalWrite(LEDB, HIGH);
    // Serial.println("Alignment complete. Now aligned to right side wall");
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

    const float SIGNIFICANT_ERROR = 20;

    if (distanceLeftFront < distanceRightFront && (abs(distanceLeftFront - distanceLeftBack) < SIGNIFICANT_ERROR &&
                                                   distanceLeftFront < SIGNIFICANT_ERROR && distanceLeftBack < SIGNIFICANT_ERROR)) {
        // if left side is closer AND if left side doesn't have too large of a difference.
        alignLeft();
    } else if (distanceLeftFront > distanceRightFront && (abs(distanceRightFront - distanceRightBack) < SIGNIFICANT_ERROR &&
                                                          distanceRightFront < SIGNIFICANT_ERROR && distanceRightBack < SIGNIFICANT_ERROR)) {
        // if right side is closer AND if right side doesn't have too large of a difference.
        alignRight();
    }
}

/**
 * @brief The state machine (of sorts) handling the movement for when using the queue. Intended to be ran on a dedicated thread.
 * This is designed for queing moves, and planning routes.
 *
 * Due to my logic pivoting the explore() function this is just here to show what I intended to do with the use of the map.
 */
void performQueue() {
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

/**
 * @brief Perform the move stack in reverse, (e.g. turnLeft ---becomes---> turnRight)
 *
 * Similar in concept to performQueue, but in this case it takes over from explore to reverse the actions done.
 */
void reverseStack() {
    turnRight(180, 0.5f);
    actionStack = combineActions(actionStack);
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
 * @brief The main logic for navigation, this is a left wall following algorithm, with a preference for orienting towards the goal to prevent the common wall following pitfalls.
 */
void explore() {
    // constants for evaluating sensor data
    const float CLOSE_TO_SIDE_WALL_DISTANCE = 20;
    const float CLOSE_TO_FRONT_WALL_DISTANCE = CLOSE_TO_SIDE_WALL_DISTANCE + 2.5;
    const float FAR_TOO_CLOSE_DISTANCE = 3.0;
    const float US_FAIL = -1;
    // pre-defining variables
    float distanceFront, distanceBack, distanceLeftFront, distanceLeftBack, distanceRightFront, distanceRightBack;
    bool frontTouchingWall, frontClear, leftFrontClear, leftBackClear, rightFrontClear, rightBackClear;
    bool goalInFront, goalToLeft, goalToRight, goalBehind;
    bool onLeftSideOfMap;

    while (true) {
        // Take distance measurements
        distanceFront = us1.measureDistanceCm();
        distanceBack = us2.measureDistanceCm();
        distanceLeftFront = irBus.measureDistanceCm(0);
        distanceLeftBack = irBus.measureDistanceCm(1);
        distanceRightFront = irBus.measureDistanceCm(2);
        distanceRightBack = irBus.measureDistanceCm(3);

        // evaluate booleans for the navigation logic
        frontTouchingWall = distanceFront < FAR_TOO_CLOSE_DISTANCE || distanceFront == US_FAIL;
        frontClear = distanceFront > CLOSE_TO_FRONT_WALL_DISTANCE;
        leftFrontClear = distanceLeftFront >= CLOSE_TO_SIDE_WALL_DISTANCE;
        leftBackClear = distanceLeftBack >= CLOSE_TO_SIDE_WALL_DISTANCE;
        rightFrontClear = distanceRightFront >= CLOSE_TO_SIDE_WALL_DISTANCE;
        rightBackClear = distanceRightBack >= CLOSE_TO_SIDE_WALL_DISTANCE;

        goalInFront = mapInstance.getRobotAngle() == 90; // 90 degrees is facing the goal
        goalToLeft = mapInstance.getRobotAngle() == 180; // clockwise 90 degrees from goal
        goalToRight = mapInstance.getRobotAngle() == 0;  // anticlockwise 90 degrees from goal
        goalBehind = mapInstance.getRobotAngle() == 270; // 180 degrees from goal
        onLeftSideOfMap = mapInstance.getRobotX() > 90;  // true when the robot is following on the left of the map

        mapInstance.updateGrid(distanceLeftFront, distanceLeftBack, distanceRightFront, distanceRightBack, distanceFront, distanceBack);
        if (mapInstance.isRobotAtFinish()) {
            reverseStack();
            break;
        }

        if (frontTouchingWall) {
            // Blocked on front -> move backwards
            moveBackward(4, 0.5f);
            align();
        } else if ((goalInFront || goalToRight) && frontClear) {
            // Clear in front and facing goal -> move forward
            align();
            moveForward(10, 0.5f);
        } else if ((!goalToRight || !onLeftSideOfMap) && leftFrontClear && leftBackClear) {
            // Turning left will result in facing the goal
            // OR not currently on left side of map (suggesting that the left wall is not being followed)
            // Left turn okay
            turnLeft(90, 0.5f);
            align();
            distanceFront = us1.measureDistanceCm(); // retake measurement on front for new orientation
            frontClear = distanceFront > CLOSE_TO_FRONT_WALL_DISTANCE;
            if (frontClear) {
                // Clear in front -> move forward
                moveForward(20, 0.5f);
            }
            align();
        } else if ((!goalToRight || !onLeftSideOfMap) && leftFrontClear && !leftBackClear) {
            // Need to move forwards for left turn
            moveForward(18, 0.5f);
            turnLeft(90, 0.5f);
            align();
            distanceFront = us1.measureDistanceCm(); // retake measurement on front for new orientation
            frontClear = distanceFront > CLOSE_TO_FRONT_WALL_DISTANCE;
            if (frontClear) {
                // Clear in front -> move forward
                moveForward(distanceLeftBack + 20, 0.5f);
            }
            align();
        } else if ((!goalToRight || !onLeftSideOfMap) && !leftFrontClear && leftBackClear) {
            // Found a new left side wall to align to
            moveForward(18, 0.5f);
            align();
        } else if (frontClear) {
            // left blocked, not facing goal, front clear -> move forward
            align();
            moveForward(10, 0.5f);
        } else if (rightFrontClear && rightBackClear) {
            // left & front blocked, right clear -> turn right
            turnRight(90, 0.5f);
            align();
            distanceFront = us1.measureDistanceCm(); // retake measurement on front for new orientation
            frontClear = distanceFront > CLOSE_TO_FRONT_WALL_DISTANCE;
            if (frontClear) {
                // Clear in front -> move forward
                moveForward(distanceLeftBack + 20, 0.5f);
            }
            align();
        } else if (rightFrontClear && !rightBackClear) {
            // left & front blocked, appreaching clearing on right
            moveForward(18, 0.5f);
            turnRight(90, 0.5f);
            align();
            distanceFront = us1.measureDistanceCm(); // retake measurement on front for new orientation
            frontClear = distanceFront > CLOSE_TO_FRONT_WALL_DISTANCE;
            if (frontClear) {
                // Clear in front -> move forward
                moveForward(distanceLeftBack + 20, 0.5f);
            }
            align();
        } else {
            // left & front & right blocked -> turn around
            turnRight(180, 0.5f);
            align();
        }
        thread_sleep_for(10);
    }
}

/**
 * @brief Built in arduino setup function
 */
void setup() {
    Serial.begin(9600);
    delay(100); // Delay so serial monitor can pick up debugging messages within the setup
    Serial.println("Started The Navigator");

    motor.setup();
    motor.startCounting();

    thread_sleep_for(1000);
    align();
    float distanceBack = us2.measureDistanceCm();
    float distanceRightBack = irBus.measureDistanceCm(3);
    mapInstance.identifyStartPosition(distanceBack, distanceRightBack);
    // performQueueThread.start(performQueue); // start the thread for performing queue (commented because it is not in use for this version of the software)
    Serial.println("Setup complete");
    thread_sleep_for(1000); // delay before start loop
}

/**
 * @brief Built in arduino loop function
 */
void loop() {
    explore();
    thread_sleep_for(1000);
}