#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();


// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed); 
}



void vexcodeInit() {

  // Initializing random seed.
  initializeRandomSeed(); 
}

#pragma endregion VEXcode Generated Robot Configuration
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       yunhao, tao, matt, richard                                                     */
/*    Created:      10/31/2026, 8:25:58 PM                                    */
/*    Description:  IQ2 project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;
using namespace std;

// A global instance of vex::brain used for printing to the IQ2 brain screen
//vex::brain Brain;

// define your global instances of motors and other devices here
motor RM(PORT12, bool(reverse));
motor LM(PORT7, bool(forward));
bumper BUMPER(PORT3);
distance DISTANCE(PORT1); //left side cliff sensor
motor_group motors(RM, LM);
optical OPTICAL(PORT6); //right side cliff sensor
controller CONTROLLER;
//inertial BrainInertial = inertial();
touchled TOUCHLED(PORT8);

// Structs
const double MAX_SPEED = 50.0;

// Struct to hold PID controller values and perform calculations
struct PIDcontroller
{
    float Kp;
    float Ki;
    float Kd;
    float integralZone; //only apply Ki in this zone
    float error;
    float previousError;
    float integral;
    float output;
    timer PIDtimer;
    float previousTime;

    // Constructor - sets up K values
    PIDcontroller(float Kp, float Ki, float Kd, float integralZone)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->integralZone = integralZone;
        error = 0;
        previousError = 0;
        integral = 0; // Tracks accumulated error if not responsive enough
        output = 0;
        this->PIDtimer.reset();
        this->previousTime = this->PIDtimer.time(msec);
    }

    ~PIDcontroller()
    {
        // Destructor
    }

    // PID calculation function
    // Params: target and current position
    // Output: Positive output for motorspeed in percentage (Max 100%)
    float calculatePID(float error)
    {
        //Calculate time difference
        float currentTime = this->PIDtimer.time(msec);
        float dt = currentTime - this->previousTime;
        this->previousTime = currentTime;

        if (dt <= 0) { return 0; } //Don't divide by zero

        // Take in error
        this->error = error;

        // Porportional Term
        float porportionalTerm = this->error * this->Kp;

        // Integral Term
        float integralTerm = 0;
        // Add the error to the integral only if in integral zone
        if (fabs(this->error) < this->integralZone){
            this->integral = this->integral + this->error * dt;
            integralTerm = this->integral * this->Ki;
        }
        
        // Derivative Term
        float derivative = (this->error - this->previousError) / dt;
        float derivativeTerm = derivative * this->Kd;
        this->previousError = this->error;

        // Make output term
        this->output = fabs(porportionalTerm + integralTerm + derivativeTerm);
        //Slew Output to prevent sudden changes in motor speed for first half a second
        float slewFactor = 1.0; // Default to no slew
        if (this->PIDtimer.time(msec) < 500)
        {
            // Calculate the number of 25ms intervals in the first 500ms
            float timeDivision = this->PIDtimer.time(msec) / 25.0; // 0 to 20
            for (int i = 0; i<timeDivision; i++){
                //Output will be full output by half a second
                //Scaled up by 5% each time
                slewFactor = slewFactor * (0.05 + 0.05*i); // 0.05, 0.1, 0.15, ... 1.0
            }
        }

        //Apply slew factor to output
        this->output = this->output * slewFactor;
        // Max motor speed is 100%
        if (this->output > 100)
        {
            this->output = 100;
        }

        return this->output; // output
    }

    //Ki setter method
    void setKi(float Ki)
    {
        this->Ki = Ki;
    }

};

// Struct to track robot position and heading
struct pose
{
    float x;
    float y;
    float heading;

    pose()
    {
        x = 0;
        y = 0;
        heading = 0;
    }
    ~pose()
    {
        // Destructor
    }
};


// Struct for robot
// Contains movement functions
struct robot
{
    pose robot_pose;
    float absoluteHeading;

    // 1=red, 2=green, 3=blue, 4=yellow
    int LED_color;

    // Default constructor
    robot()
    {
        robot_pose = pose();
        absoluteHeading = 0;
        LED_color = 0;
    }

    // Destructor
    ~robot() {}



};



// Function decalarations

void turn_heading(float targetHeading);
void find_absolute_heading();

// Calibrate all sensors
void calibrateSensors();

void drivingHeading();
void drivingHeading(float targetHeading, float expectedMM = 0, bool perimeter = false);
bool driveDistance(float targetDistance, float targetSpeed);
void automaticCleaning();
float pauseForObstacle();

// Settings used by automaticCleaning and its existing movement functions.
const float BACK_MM = 50.0f;
const float EDGE_APPROACH_MM = 150.0f;
bool cleaningFailed = false;

float driveEncoderMM() {
    return (LM.position(degrees) + RM.position(degrees)) / 2.0f * 200.0f / 360.0f;
}

float headingError(float targetHeading) {
    float error = fmod(targetHeading - BrainInertial.rotation(degrees), 360.0f);
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    return error;
}

void stopCleaning(const char *message) {
    cleaningFailed = true;
    motors.stop(brake);
    TOUCHLED.setColor(color::red);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("%s", message);
}

// Test functions----------------------------------------------------------------------------------
void testPIDTurning()
{
    turn_heading(40);
    wait(1, seconds);
    turn_heading(20);
    wait(1, seconds);
    turn_heading(90);
    wait(1,seconds);
    turn_heading(-150);
    wait(1, seconds);
    turn_heading(-40);
    wait(1, seconds);
    turn_heading(-20);
    
}

void test_find_absolute_heading()
{
    find_absolute_heading();
}

// Event functions
void endAutonomous(bool &end_autonomous)
{
    end_autonomous = true;
    TOUCHLED.setColor(color::red);
    motors.stop(brake);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("End Auto");
}

// Button functions
void handle_TOUCLED(bool &end_autonomous, color &TOUCLED_COLOR)
{
    if (TOUCLED_COLOR == color::red)
    {
        endAutonomous(end_autonomous);
    }
}

// -------------------------------------- MAIN ---------------------------------------------------
int main()
{
  automaticCleaning();
  return 0;
}

// Function definitions

// Yunhao -----------------------------------------------------------------------------------------
void calibrateSensors()
{
    Brain.Screen.printAt(2, 50, "Calibrating Sensors");
    BrainInertial.calibrate();
    while (BrainInertial.isCalibrating())
    {
        wait(10, msec);
    } // Allow time for calibration
    
    BrainInertial.resetRotation();
}

// Purpose: Finds the absolute heading of the robot
// Outputs: None
void find_absolute_heading() {
    if (cleaningFailed) return;
    // Start each alignment attempt with both front sensors over the table.
    if (!DISTANCE.isObjectDetected() || DISTANCE.objectDistance(mm) >= 50 ||
        !OPTICAL.isNearObject()) {
        if (!driveDistance(-BACK_MM, 15)) return;
    }
    // Declare timers
    timer leftTimer;
    timer rightTimer;
    int attempts = 0;

    float left_time = 0;
    float right_time = 0;
    
    const float ACCEPTABLE_TIME_ERROR = 50; // Milliseconds at the 15% probing speed
    const float SENSOR_HEIGHT = 50; 
    float error = 0; //In milliseconds

    // Instantiate PID outside the loop so it retains memory (Integral/Derivative)
    float kp = 0.04;
    float ki = 0.0; 
    float kd = 0.0; 
    float integralZone = 500; // only use ki when error is within this zone (msec)
    PIDcontroller turnPID = PIDcontroller(kp, ki, kd, integralZone);

    do {
        if (++attempts > 8) {
            stopCleaning("Alignment not settled");
            return;
        }
        // Reset flags and variables for this attempt
        bool left_edge_found = false;
        bool right_edge_found = false;
        left_time = 0;
        right_time = 0;

        leftTimer.clear();
        rightTimer.clear();
        float pausedMS = 0;
        float firstEdgeMM = -1;
        const float startMM = driveEncoderMM();

        // Drive forward until BOTH sensors have detected the edge
        while (!left_edge_found || !right_edge_found) {
            pausedMS += pauseForObstacle();
            const float travelledMM = driveEncoderMM() - startMM;
            if (leftTimer.time(msec) - pausedMS > 65000 || travelledMM > 4000 ||
                (firstEdgeMM >= 0 && travelledMM - firstEdgeMM > 45)) {
                stopCleaning("Alignment edge mismatch");
                return;
            }
            
            // If right sensor is still on the table, keep driving right motor
            if (OPTICAL.isNearObject() && !right_edge_found) {
                // You can replace this with your straight PID driving logic
                motors.spin(forward, 15, percent);
            } else if (!right_edge_found) {
                // Right sensor just hit the edge! Record time and mark true
                right_time = rightTimer.time(msec) - pausedMS;
                right_edge_found = true;
                if (firstEdgeMM < 0) firstEdgeMM = travelledMM;
            }

            // If left sensor is still on the table, keep driving left motor
            if (DISTANCE.isObjectDetected() && DISTANCE.objectDistance(mm) < SENSOR_HEIGHT &&
                !left_edge_found) {
                motors.spin(forward, 15, percent);
            } else if (!left_edge_found) {
                // Left sensor just hit the edge! Record time and mark true
                left_time = leftTimer.time(msec) - pausedMS;
                left_edge_found = true;
                if (firstEdgeMM < 0) firstEdgeMM = travelledMM;
            }
            
            // Small sleep to prevent CPU hogging
            if (left_edge_found && right_edge_found) motors.stop(brake);
            task::sleep(10);
        }

        // Stop completely once both edges are found
        motors.stop(brake);
        task::sleep(100); // Allow physical momentum to settle

        // Every measured corner uses the same encoder-based setback.
        if (!driveDistance(-BACK_MM, 15)) return;
        if (!DISTANCE.isObjectDetected() || DISTANCE.objectDistance(mm) >= SENSOR_HEIGHT ||
            !OPTICAL.isNearObject()) {
            stopCleaning("Alignment backoff failed");
            return;
        }

        // Calculate alignment error
        error = left_time - right_time; 

        // If error is outside acceptable limits, execute correction turn
        if (fabs(error) > ACCEPTABLE_TIME_ERROR) {
            //float turnAmount = turnPID.calculatePID(error); //Degrees turned (MAX output is 100)
            float turnAmount = fmin(8.0f, fmax(1.5f, fabs(error) * .014f));

            // If error > 0, left_time was larger, right sensor hit the edge first
            // The right side is too far forward
            if (error > 0) {
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(3, 1);
                Brain.Screen.print("Error: %.2f", error); 
                Brain.Screen.setCursor(4, 1);
                Brain.Screen.print("turnAmount: %.2f", turnAmount);
                Brain.Screen.setCursor(5,1);
                Brain.Screen.print("turning right");
                turn_heading(turnAmount); // Turn Right
            } else { 
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(3, 1);
                Brain.Screen.print("Error: %.2f", error); 
                Brain.Screen.setCursor(4, 1);
                Brain.Screen.print("turnAmount: %.2f", turnAmount);
                Brain.Screen.setCursor(5,1);
                Brain.Screen.print("turning left");
                turn_heading(-turnAmount);  // Turn Left
            }
            if (cleaningFailed) return;
        }

        // Debugging output
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Left Time: %.2f", left_time);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Right Time: %.2f", right_time);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Error: %.2f", error);

    } while (fabs(error) > ACCEPTABLE_TIME_ERROR); // FIX: Repeat while error is LARGER than acceptable bounds

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Absolute Heading found");
    return;
}


// Params: Heading (+ or -)
// Purpose: Turns by a relative angle while keeping a continuous gyro reference
// Outputs: None
void turn_heading(float targetHeading)
{
    // Stop all motors
    motors.stop(brake);
    if (cleaningFailed) return;

    float error = targetHeading;  // Initial error is just heading
    targetHeading += BrainInertial.rotation(degrees);
    float acceptableError = 1.0; // degrees
    float currentHeading = 0;
    float motorSpeed = 0; // In percentage

    // Construct PID controller object
    float Kp = 0.8;
    float Ki = 0.00028;
    float Kd = 0.8;

    //Changing Ki based on error

    float integralZone = 90; //only apply Ki in this zone (degrees of a turn)
    PIDcontroller turnPID = PIDcontroller(Kp, Ki, Kd, integralZone);
    bool settled = false;
    timer turnTimer;
    turnTimer.reset();
    float pausedMS = 0;
    // Perform movement until error is acceptable
    while (!settled)
    {
        float pauseMS = pauseForObstacle();
        if (pauseMS > 0) {
            pausedMS += pauseMS;
            turnPID.integral = 0;
            turnPID.previousError = 0;
            turnPID.previousTime = turnPID.PIDtimer.time(msec);
        }
        if (turnTimer.time(msec) - pausedMS > 8000) {
            stopCleaning("Turn timeout");
            return;
        }
        // continuously find current heading and error
        currentHeading = BrainInertial.rotation(degrees);
        error = targetHeading - currentHeading;
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("%f", currentHeading);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("%f", error);

        // Determine which direction to turn
        if (fabs(error) <= acceptableError) {
            motors.stop(brake);
        }
        else if (error > 0)
        {                                             // right turn
            motorSpeed = turnPID.calculatePID(error); // Pass in error to PIDcontroller for output
            motorSpeed = fmin(25.0f, fmax(5.0f, motorSpeed));
            motorSpeed = fmin(motorSpeed, fmax(5.0f, fabs(error) * 1.5f));

            RM.spin(reverse, motorSpeed, percent);
            LM.spin(forward, motorSpeed, percent);
        }
        else
        {                                             // left turn
            motorSpeed = turnPID.calculatePID(error); // Pass in error to PIDcontroller for output
            motorSpeed = fmin(25.0f, fmax(5.0f, motorSpeed));
            motorSpeed = fmin(motorSpeed, fmax(5.0f, fabs(error) * 1.5f));

            RM.spin(forward, motorSpeed, percent);
            LM.spin(reverse, motorSpeed, percent);
        }
        task:: sleep(20); // Small delay to prevent CPU hogging

        //Calculate rotational acceleration (gyroscope)
        float gxNew = BrainInertial.gyroRate(xaxis, dps);
        float gyNew = BrainInertial.gyroRate(yaxis, dps);
        float gzNew = BrainInertial.gyroRate(zaxis, dps);
        float rotationRate = sqrt(gxNew * gxNew + gyNew * gyNew + gzNew * gzNew);

        // Check if the robot has settled after the turn
        if (fabs(error) <= acceptableError && !settled && rotationRate < 1) {
            // Wait for a short duration to ensure the robot has settled
            task::sleep(200); // Adjust this duration as needed
            settled = fabs(targetHeading - BrainInertial.rotation(degrees)) <= acceptableError;
        }
    }
    motors.stop(brake);
    return;
}

// Riley ------------------------------------------------------------------------------------------

// Params: Velocity, heading
// Purpose: Drives forward using gyro sensor
// Outputs: None

void drivingHeading(float targetHeading, float expectedMM, bool perimeter)
{
    if (cleaningFailed) return;
    motors.stop(brake);
    // Use the original 40% for initial straight driving; survey the perimeter at 20%.
    float velocity = perimeter ? 20 : 40;
    const float startMM = driveEncoderMM();
    float pausedMS = 0;
    int sideCorrections = 0;

    float Kp = 0.88;
    float Ki = 0.0;
    float Kd = 0.3;
    float integralZone = 20;
    PIDcontroller drivePID(Kp, Ki, Kd, integralZone);
    // Let the original PID's 500 ms startup interval finish while stationary.
    // Otherwise its small correction output lets each new row drift sideways.
    wait(500, msec);
    const float MAX_CORRECTION = 10;

    timer driveTimer;
    driveTimer.clear();
    while (true) {
        if (BUMPER.pressing()) {
            pausedMS += pauseForObstacle();
            drivePID.previousError = 0;
            drivePID.previousTime = drivePID.PIDtimer.time(msec);
        }

        const bool leftOn = DISTANCE.isObjectDetected() && DISTANCE.objectDistance(mm) < 50;
        const bool rightOn = OPTICAL.isNearObject();
        const float travelledMM = driveEncoderMM() - startMM;
        const float elapsedMS = driveTimer.time(msec) - pausedMS;
        if (!leftOn || !rightOn) {
            motors.stop(brake);

            // On the clockwise perimeter the outside edge is on the left.
            if (perimeter && !leftOn && rightOn) {
                if (++sideCorrections > 4) {
                    stopCleaning("Side correction limit");
                    return;
                }
                const float recoveryStart = driveTimer.time(msec);
                turn_heading(headingError(targetHeading + 8));
                if (cleaningFailed) return;

                // If the pivot restores both sensors, move a little inward.
                if (DISTANCE.isObjectDetected() && DISTANCE.objectDistance(mm) < 50 &&
                    OPTICAL.isNearObject()) {
                    if (!driveDistance(40, 12)) return;
                    turn_heading(headingError(targetHeading));
                    if (cleaningFailed) return;
                    pausedMS += driveTimer.time(msec) - recoveryStart;
                    drivePID.previousError = 0;
                    drivePID.previousTime = drivePID.PIDtimer.time(msec);
                    continue;
                }

                // A front corner can also trigger the left sensor first.
                turn_heading(headingError(targetHeading));
                if (cleaningFailed) return;
            }

            if (expectedMM > 0 && travelledMM < expectedMM - EDGE_APPROACH_MM) {
                stopCleaning("Unexpected early edge");
                return;
            }
            break;
        }
        const float limitMM = expectedMM > 0 ? expectedMM + 120 : 4000;
        if (elapsedMS > 65000 || travelledMM >= limitMM) {
            stopCleaning("Straight travel limit");
            return;
        }

        float speed = velocity;
        if (expectedMM > 0) {
            const float remainingMM = expectedMM - travelledMM;
            const float ramp = fmin(1.0f, fmax(0.0f, (remainingMM - EDGE_APPROACH_MM) / 60.0f));
            speed = 12 + (velocity - 12) * ramp;
        }

        float error = headingError(targetHeading);
        float correctionMagnitude = drivePID.calculatePID(error);
        if (correctionMagnitude > MAX_CORRECTION) correctionMagnitude = MAX_CORRECTION;
        if (error < 0) correctionMagnitude = -correctionMagnitude;
        // Correct small errors too, so repeated rows do not drift sideways.
        float correction = correctionMagnitude;
        float leftSpeed = speed + correction;
        float rightSpeed = speed - correction;
        LM.spin(forward, leftSpeed, percent);
        RM.spin(forward, rightSpeed, percent);
        task::sleep(20);
    }
    motors.stop(brake);
}

// Richard ----------------------------------------------------------------------------------------
void driveMotor(motor &m, double speed) { //mannual control
    if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
    } else if (speed < -MAX_SPEED) {
        speed = -MAX_SPEED;
    }

    if (fabs(speed) < 3) {
        m.stop(brake);
    } else {
        m.spin(forward, speed, percent);
    }
}

void Manual_Control() {
  while (!CONTROLLER.ButtonEUp.pressing()) {

    double forwardInput = CONTROLLER.AxisA.position();
    double turnInput = CONTROLLER.AxisB.position();

    if (fabs(forwardInput) < 5) {
      forwardInput = 0;
    }
    if (fabs(turnInput) < 5) {
      turnInput = 0;
    }

    double leftSpeed = forwardInput + turnInput;
    double rightSpeed = forwardInput - turnInput;


    double largest = fabs(leftSpeed);
    if (fabs(rightSpeed) > largest) {
      largest = fabs(rightSpeed);
    }

    if (largest > 100) {
      leftSpeed = leftSpeed / largest * 100;
      rightSpeed = rightSpeed / largest * 100;
    }
    leftSpeed *= MAX_SPEED / 100.0;
    rightSpeed *= MAX_SPEED / 100.0;

    driveMotor(LM, leftSpeed);
    driveMotor(RM, rightSpeed);
    
    wait(10, msec);
  }
  motors.stop();
}

void automaticCleaning() {
    const float MAX_STEP_MM = 80;
    int rows = 0;
    float stepMM = 0;
    cleaningFailed = false;

    motors.stop(brake);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Place robot, tap Touch");
    while (TOUCHLED.pressing()) wait(10, msec);
    while (!TOUCHLED.pressing()) wait(10, msec);
    while (TOUCHLED.pressing()) wait(10, msec);

    wait(300, msec);
    calibrateSensors();
    TOUCHLED.setColor(color::green);

    // Find the first edge and keep its normal as the reference for the run.
    drivingHeading(BrainInertial.rotation(degrees));
    if (cleaningFailed) return;
    find_absolute_heading();
    if (cleaningFailed) return;
    const float edgeHeading = BrainInertial.rotation(degrees);

    // First reach a corner, then clean and measure four complete sides.
    float sidesMM[4];
    for (int side = 0; side < 5; ++side) {
        const float heading = edgeHeading + 90 * (side + 1);
        turn_heading(headingError(heading));
        if (cleaningFailed) return;
        const float startMM = driveEncoderMM();
        drivingHeading(heading, 0, true);
        if (cleaningFailed) return;
        find_absolute_heading();
        if (cleaningFailed) return;
        if (side > 0) {
            sidesMM[side - 1] = driveEncoderMM() - startMM;
            if (sidesMM[side - 1] <= 10) {
                stopCleaning("Table span too short");
                return;
            }
        }
    }

    // Compare opposite sides before using the measured spans for the rows.
    for (int axis = 0; axis < 2; ++axis) {
        const float longer = fmax(sidesMM[axis], sidesMM[axis + 2]);
        if (fabs(sidesMM[axis] - sidesMM[axis + 2]) > fmax(40.0f, longer * 0.10f)) {
            stopCleaning("Perimeter mismatch");
            return;
        }
    }
    const float lengthMM = (sidesMM[0] + sidesMM[2]) / 2;
    const float widthMM = fmin(sidesMM[1], sidesMM[3]) - 10;
    rows = (int)ceil(widthMM / MAX_STEP_MM) + 1;
    stepMM = widthMM / (rows - 1);

    // Keep the original row loop. Each row follows one of two fixed headings.
    float shiftedMM = 0;
    for (int row = 0; row < rows; ++row) {
        const float heading = edgeHeading + 180 + (row % 2) * 180;
        turn_heading(headingError(heading));
        if (cleaningFailed) return;
        drivingHeading(heading, lengthMM + BACK_MM);
        if (cleaningFailed) return;

        // Retreat and turn into the next lane.
        if (!driveDistance(-BACK_MM, 15)) return;
        if (row == rows - 1) break;

        turn_heading(headingError(edgeHeading + 270));
        if (cleaningFailed) return;
        const float startMM = driveEncoderMM();
        if (!driveDistance((row + 1) * stepMM - shiftedMM, 15)) return;
        shiftedMM += driveEncoderMM() - startMM;
        wait(200, msec);
    }

    motors.stop(brake);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Rows finished: %d", rows);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Span L %.0f W %.0f", lengthMM, widthMM + 10);
}

float pauseForObstacle() {
    if (!BUMPER.pressing()) return 0;

    motors.stop(brake);
    timer pauseTimer;
    pauseTimer.reset();
    TOUCHLED.setColor(color::red);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Remove obstacle");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Then tap Touch");

    // Require the obstacle to be removed, then a NEW Touch press and release.
    while (true) {
        while (BUMPER.pressing() || TOUCHLED.pressing()) wait(20, msec);
        while (!TOUCHLED.pressing() && !BUMPER.pressing()) wait(20, msec);
        if (BUMPER.pressing()) continue;
        while (TOUCHLED.pressing() && !BUMPER.pressing()) wait(20, msec);
        if (!BUMPER.pressing()) break;
    }

    TOUCHLED.setColor(color::green);
    return pauseTimer.time(msec);
}


bool driveDistance(float targetDistance, float targetSpeed) {
    motors.stop(brake);
    if (cleaningFailed) return false;
    if (targetSpeed <= 0 || targetSpeed > MAX_SPEED) return false;

    float targetDegree = fabs(targetDistance) / 200.0 * 360.0;
    float leftStart = LM.position(degrees);
    float rightStart = RM.position(degrees);
    float travelled = 0;
    float pausedMS = 0;
    float timeoutMS = 5000 + fabs(targetDistance) * 50;
    directionType direction = targetDistance >= 0 ? forward : reverse;
    const float targetHeading = BrainInertial.rotation(degrees);
    PIDcontroller drivePID(0.88, 0, 0.3, 20);
    timer moveTimer;
    moveTimer.reset();

    while (travelled < targetDegree &&
           moveTimer.time(msec) - pausedMS < timeoutMS) {
        float pauseMS = pauseForObstacle();
        if (pauseMS > 0) {
            pausedMS += pauseMS;
            drivePID.previousError = 0;
            drivePID.previousTime = drivePID.PIDtimer.time(msec);
        }
        if (direction == forward &&
            (!DISTANCE.isObjectDetected() || DISTANCE.objectDistance(mm) >= 50 ||
             !OPTICAL.isNearObject())) {
            stopCleaning("Edge during distance move");
            return false;
        }
        float error = headingError(targetHeading);
        float correction = fmin(6.0f, drivePID.calculatePID(error));
        if (error < 0) correction = -correction;
        const float speed = direction == forward ? targetSpeed : -targetSpeed;
        LM.spin(forward, speed + correction, percent);
        RM.spin(forward, speed - correction, percent);
        wait(10, msec);
        travelled = (
            fabs(LM.position(degrees) - leftStart) +
            fabs(RM.position(degrees) - rightStart)
        ) / 2.0;
    }

    motors.stop(brake);
    if (travelled < targetDegree) {
        stopCleaning("Move timeout");
    } else if (direction == forward &&
               (!DISTANCE.isObjectDetected() || DISTANCE.objectDistance(mm) >= 50 ||
                !OPTICAL.isNearObject())) {
        stopCleaning("Edge during distance move");
        return false;
    }
    return travelled >= targetDegree;
}
// Matt -------------------------------------------------------------------------------------------

void test_edge_detection(){
  for(int i = 0; i < 4; i++){
    find_absolute_heading();

    turn_heading(90);

    drivingHeading(BrainInertial.rotation(degrees));
  }
}
