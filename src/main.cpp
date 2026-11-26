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
// vex::brain Brain;

// define your global instances of motors and other devices here
motor RM(PORT12, bool(reverse));
motor LM(PORT7, bool(forward));
motor BRUSH(PORT4, bool(forward));
bumper BUMPER(PORT3);
distance DISTANCE(PORT1); // left side cliff sensor
motor_group motors(RM, LM);
optical OPTICAL(PORT6); // right side cliff sensor
controller CONTROLLER;
motor COLLECTOR(PORT2, false); // seesaw collector
touchled TOUCHLED(PORT8);

// Structs

// Struct to hold PID controller values and perform calculations
struct PIDcontroller
{
    float Kp;
    float Ki;
    float Kd;
    float integralZone; // only apply Ki in this zone
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
        // Calculate time difference
        float currentTime = this->PIDtimer.time(msec);
        float dt = currentTime - this->previousTime;
        this->previousTime = currentTime;

        if (dt <= 0)
        {
            return 0;
        } // Don't divide by zero

        // Take in error
        this->error = error;

        // Porportional Term
        float porportionalTerm = this->error * this->Kp;

        // Integral Term
        float integralTerm = 0;
        // Add the error to the integral only if in integral zone
        if (fabs(this->error) < this->integralZone)
        {
            this->integral = this->integral + this->error * dt;
            integralTerm = this->integral * this->Ki;
        }

        // Derivative Term
        float derivative = (this->error - this->previousError) / dt;
        float derivativeTerm = derivative * this->Kd;
        this->previousError = this->error;

        // Make output term
        this->output = fabs(porportionalTerm + integralTerm + derivativeTerm);
        // Slew Output to prevent sudden changes in motor speed for first half a second
        float slewFactor = 1.0; // Default to no slew
        if (this->PIDtimer.time(msec) < 500)
        {
            // Calculate the number of 25ms intervals in the first 500ms
            float timeDivision = this->PIDtimer.time(msec) / 25.0; // 0 to 20
            for (int i = 0; i < timeDivision; i++)
            {
                // Output will be full output by half a second
                // Scaled up by 5% each time
                slewFactor = slewFactor * (0.05 + 0.05 * i); // 0.05, 0.1, 0.15, ... 1.0
            }
        }

        // Apply slew factor to output
        this->output = this->output * slewFactor;
        // Max motor speed is 100%
        if (this->output > 100)
        {
            this->output = 100;
        }

        return this->output; // output
    }

    // Ki setter method
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

struct collisionMonitor{
    bool collisionDetected;
    int collisionCount;
    timer collisionTimer;
    float previousTime;
    float collisionWindowStart;
    float collisionDetectedAt;

    //Records acceleration values from the last loop to calculate jerk
    float prevAx;
    float prevAy;
    float prevAz;

    //Records rotational values from last loop to find angular acceleration
    float prevGx;
    float prevGy;
    float prevGz;


    collisionMonitor()
    {
        collisionDetected = false;
        collisionCount = 0;
        collisionTimer.reset();
        previousTime = 0;
        collisionWindowStart = 0;
        collisionDetectedAt = 0;
        prevGx = BrainInertial.gyroRate(xaxis, dps);
        prevGy = BrainInertial.gyroRate(yaxis, dps);
        prevGz = BrainInertial.gyroRate(zaxis, dps);
        prevAx = BrainInertial.acceleration(xaxis);
        prevAy = BrainInertial.acceleration(yaxis);
        prevAz = BrainInertial.acceleration(zaxis);
    }
    ~collisionMonitor()
    {
        // Destructor
    }

    void reset()
    {
        collisionDetected = false;
        collisionCount = 0;
        collisionTimer.reset();
        previousTime = 0;
        collisionWindowStart = 0;
        collisionDetectedAt = 0;
        prevAx = BrainInertial.acceleration(xaxis);
        prevAy = BrainInertial.acceleration(yaxis);
        prevAz = BrainInertial.acceleration(zaxis);
        prevGx = BrainInertial.gyroRate(xaxis, dps);
        prevGy = BrainInertial.gyroRate(yaxis, dps);
        prevGz = BrainInertial.gyroRate(zaxis, dps);
    }

    bool detectCollisionLin(){
        const float JERK_THRESHOLD = 10.0; //Collision threshold in m/s^3 
        const float COUNT_WINDOW_S = 0.15f;
        const float FLAG_HOLD_S = 0.5f;

        //Calculate time difference since last reading
        float currentTime = collisionTimer.time(msec) / 1000.0; // Convert to seconds
        float dt = currentTime - previousTime;

        if (collisionDetected && currentTime - collisionDetectedAt >= FLAG_HOLD_S){
            collisionDetected = false;
            collisionCount = 0;
        }
        if (!collisionDetected && collisionCount > 0 &&
            currentTime - collisionWindowStart >= COUNT_WINDOW_S){
            collisionCount = 0;
        }

        float currAx = BrainInertial.acceleration(xaxis);
        float currAy = BrainInertial.acceleration(yaxis);    
        float currAz = BrainInertial.acceleration(zaxis);

        // Do not divide by zero
        if (dt > 0){
            float jerkX = (currAx - prevAx) / dt;
            float jerkY = (currAy - prevAy) / dt;
            float jerkZ = (currAz - prevAz) / dt;

            //Calculate jerk magnitude
            float jerk = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);

            if (!collisionDetected && jerk > JERK_THRESHOLD){
                if (collisionCount == 0)
                    collisionWindowStart = currentTime;
                collisionCount++;
                if (collisionCount >= 3){
                    collisionDetected = true;
                    collisionDetectedAt = currentTime;
                    Brain.Screen.clearScreen();
                    Brain.Screen.setCursor(1, 1);
                    Brain.Screen.print("Collision Detected");
                }
            }
            
        }
        //Update member variables
        previousTime = currentTime;
        prevAx = currAx;
        prevAy = currAy;
        prevAz = currAz;
        return collisionDetected;
    }

    // Detects collisions based on angular acceleration
    bool detectCollisionAng(){
        const float ANGULAR_ACCEL_THRESHOLD = 3200.0; //Collision threshold in degrees/s^2

        //Calculate time difference since last reading
        float currentTime = collisionTimer.time(msec) / 1000.0; // Convert to seconds
        float dt = currentTime - previousTime;

        float currGx = BrainInertial.gyroRate(xaxis, dps);
        float currGy = BrainInertial.gyroRate(yaxis, dps);
        float currGz = BrainInertial.gyroRate(zaxis, dps);

        if (dt > 0){ // Do not divide by zero
            //Calculate angular acceleration for each axis
            float dGX = (currGx - prevGx) / dt;
            float dGY = (currGy - prevGy) / dt;
            float dGZ = (currGz - prevGz) / dt;

            //Calculate angular acceleration magnitude
            float angAccel = sqrt(dGX * dGX + dGY * dGY + dGZ * dGZ);

     
            //If angular acceleration exceeds threshold, increment collision count
            if (angAccel > ANGULAR_ACCEL_THRESHOLD){
                collisionCount++;
                collisionDetected = true;
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("Collision Detected");

            }

            /*
            //Only register collison if loop happens 2 times in a row
            if (collisionCount > 1 && !collisionDetected){
                collisionDetected = true;
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("Collision Detected");
            }*/
            
        }

        //Update member variables
        previousTime = currentTime;
        prevGx = currGx;
        prevGy = currGy;
        prevGz = currGz;

        return collisionDetected;
    }
};


// Struct for robot
// Contains movement functions
struct robot
{
    const double MAX_SPEED = 50.0; // Maximum speed of the robot
    pose robot_pose;
    collisionMonitor collisionMonitor;
    float absoluteHeading;
    const float BACK_MM = 100.0;           // Distance to back up when finding absolute heading
    const float EDGE_APPROACH_MM = 150.0; // How far before the expected edge a detection is still accepted
    bool cleaningFailed;

    // 1=red, 2=green, 3=blue, 4=yellow
    int LED_color;

    // Default constructor
    robot()
    {
        robot_pose = pose();
        absoluteHeading = 0;
        LED_color = 0;
        cleaningFailed = false;
    }

    // Destructor
    ~robot() {}

    // Methods

    void calibrateSensors()
    {
        Brain.Screen.printAt(2, 50, "Calibrating Sensors");
        BrainInertial.calibrate();
        while (BrainInertial.isCalibrating())
        {
            wait(10, msec);
        } // Allow time for calibration

        BrainInertial.resetRotation();
        // Full brightness so every colour is clearly visible
        TOUCHLED.setBrightness(100);
    }

    float driveEncoderMM()
    {
        // Average the left and right motor encoders to get the distance traveled in mm
        return (LM.position(degrees) + RM.position(degrees)) / 2.0f * 200.0f / 360.0f;
    }

    float headingError(float targetHeading)
    {
        float error = fmod(targetHeading - BrainInertial.rotation(degrees), 360.0f);
        if (error > 180)
            error -= 360;
        if (error < -180)
            error += 360;
        return error;
    }

    void stopCleaning(const char *message)
    {
        cleaningFailed = true;
        motors.stop(brake);
        TOUCHLED.setColor(color::red);
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("%s", message);
    }
    // Yunhao -----------------------------------------------------------------------------------------
    // Purpose: Finds the absolute heading of the robot
    // Outputs: None
    void find_absolute_heading()
    {
        if (cleaningFailed)
            return;
        // Start each alignment attempt with both front sensors over the table.
        if (!DISTANCE.isObjectDetected() || DISTANCE.objectDistance(mm) >= 100 ||
            !OPTICAL.isNearObject())
        {
            if (!driveDistance(-BACK_MM, 15))
                return;
        }
        // Declare timers
        timer leftTimer;
        timer rightTimer;
        int attempts = 0;

        float left_time = 0;
        float right_time = 0;

        const float ACCEPTABLE_TIME_ERROR = 150; // Milliseconds at the 15% probing speed
        const float SENSOR_HEIGHT = 100;
        float error = 0; // In milliseconds

        // Instantiate PID outside the loop so it retains memory (Integral/Derivative)
        float kp = 0.04;
        float ki = 0.0;
        float kd = 0.0;
        float integralZone = 500; // only use ki when error is within this zone (msec)
        PIDcontroller turnPID = PIDcontroller(kp, ki, kd, integralZone);

        do
        {
            if (++attempts > 8)
            {
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
            float startMM = driveEncoderMM();

            // Drive forward until BOTH sensors have detected the edge
            while (!left_edge_found || !right_edge_found)
            {
                pausedMS += pauseForObstacle();
                const float travelledMM = driveEncoderMM() - startMM;
                if (firstEdgeMM >= 0 && travelledMM - firstEdgeMM > 67)
                {
                    motors.stop(brake);
                    const float correctionAngle = left_edge_found ? -10.0f : 10.0f;
                    const float retreatMM = travelledMM - firstEdgeMM + BACK_MM;
                    if (!driveDistance(-retreatMM, 15))
                        return;
                    turn_heading(correctionAngle);
                    // begin fresh measurement after repositioning
                    left_edge_found = false;
                    right_edge_found = false;
                    left_time = 0;
                    right_time = 0;
                    firstEdgeMM = -1;
                    startMM = driveEncoderMM();
                    pausedMS = 0;
                    leftTimer.clear();
                    rightTimer.clear();
                    continue;
                }

                // If right sensor is still on the table, keep driving right motor
                if (OPTICAL.isNearObject() && !right_edge_found)
                {
                    // You can replace this with your straight PID driving logic
                    motors.spin(forward, 15, percent);
                }
                else if (!right_edge_found)
                {
                    // Right sensor just hit the edge! Record time and mark true
                    right_time = rightTimer.time(msec) - pausedMS;
                    right_edge_found = true;
                    if (firstEdgeMM < 0)
                        firstEdgeMM = travelledMM;
                }

                // If left sensor is still on the table, keep driving left motor
                if (DISTANCE.isObjectDetected() && DISTANCE.objectDistance(mm) < SENSOR_HEIGHT &&
                    !left_edge_found)
                {
                    motors.spin(forward, 15, percent);
                }
                else if (!left_edge_found)
                {
                    // Left sensor just hit the edge! Record time and mark true
                    left_time = leftTimer.time(msec) - pausedMS;
                    left_edge_found = true;
                    if (firstEdgeMM < 0)
                        firstEdgeMM = travelledMM;
                }

                // Small sleep to prevent CPU hogging
                if (left_edge_found && right_edge_found)
                    motors.stop(brake);
                task::sleep(10);
            }

            // Stop completely once both edges are found
            motors.stop(brake);
            task::sleep(100); // Allow physical momentum to settle

            // Every measured corner uses the same encoder-based setback.
            if (!driveDistance(-BACK_MM, 15))
                return;
            if (!DISTANCE.isObjectDetected() || DISTANCE.objectDistance(mm) >= SENSOR_HEIGHT ||
                !OPTICAL.isNearObject())
            {
                stopCleaning("Alignment backoff failed");
                return;
            }

            // Calculate alignment error
            error = left_time - right_time;

            // If error is outside acceptable limits, execute correction turn
            if (fabs(error) > ACCEPTABLE_TIME_ERROR)
            {
                //float turnAmount = turnPID.calculatePID(error); //Degrees turned (MAX output is 100)
                float turnAmount = fmin(8.0f, fmax(0.67f, fabs(error) * .014f));

                // If error > 0, left_time was larger, right sensor hit the edge first
                // The right side is too far forward
                if (error > 0)
                {
                    Brain.Screen.clearScreen();
                    Brain.Screen.setCursor(3, 1);
                    Brain.Screen.print("Error: %.2f", error);
                    Brain.Screen.setCursor(4, 1);
                    Brain.Screen.print("turnAmount: %.2f", turnAmount);
                    Brain.Screen.setCursor(5, 1);
                    Brain.Screen.print("turning right");
                    turn_heading(turnAmount); // Turn Right
                }
                else
                {
                    Brain.Screen.clearScreen();
                    Brain.Screen.setCursor(3, 1);
                    Brain.Screen.print("Error: %.2f", error);
                    Brain.Screen.setCursor(4, 1);
                    Brain.Screen.print("turnAmount: %.2f", turnAmount);
                    Brain.Screen.setCursor(5, 1);
                    Brain.Screen.print("turning left");
                    turn_heading(-turnAmount); // Turn Left
                }
                if (cleaningFailed)
                    return;
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
        if (cleaningFailed)
            return;

        // Collector follows the turn direction (straight driving leaves it as it is):
        //   right turn -> LEFT side down, left turn -> RIGHT side down.
        // Small correction turns (squaring up to an edge, side corrections) don't move it.
        const float COLLECTOR_TURN_DEG = 20.0;
        if (fabs(targetHeading) >= COLLECTOR_TURN_DEG)
        {
            if (!setCollector(targetHeading > 0 ? -1 : 1))
                return;
        }

        float error = targetHeading; // Initial error is just heading
        targetHeading += BrainInertial.rotation(degrees);
        float acceptableError = 1.0; // degrees
        float currentHeading = 0;
        float motorSpeed = 0; // In percentage

        // Construct PID controller object
        float Kp = 0.8;
        float Ki = 0.00028;
        float Kd = 0.8;

        // Changing Ki based on error

        float integralZone = 90; // only apply Ki in this zone (degrees of a turn)
        PIDcontroller turnPID = PIDcontroller(Kp, Ki, Kd, integralZone);
        bool settled = false;
        timer turnTimer;
        turnTimer.reset();
        float pausedMS = 0;
        // Perform movement until error is acceptable
        while (!settled)
        {
            pauseForObstacle();
            float pauseMS = pauseForObstacle();
            if (pauseMS > 0)
            {
                pausedMS += pauseMS;
                turnPID.integral = 0;
                turnPID.previousError = 0;
                turnPID.previousTime = turnPID.PIDtimer.time(msec);
            }
            if (turnTimer.time(msec) - pausedMS > 8000)
            {
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
            if (fabs(error) <= acceptableError)
            {
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
            task::sleep(20); // Small delay to prevent CPU hogging

            // Calculate rotational acceleration (gyroscope)
            float gxNew = BrainInertial.gyroRate(xaxis, dps);
            float gyNew = BrainInertial.gyroRate(yaxis, dps);
            float gzNew = BrainInertial.gyroRate(zaxis, dps);
            float rotationRate = sqrt(gxNew * gxNew + gyNew * gyNew + gzNew * gzNew);

            // Check if the robot has settled after the turn
            if (fabs(error) <= acceptableError && !settled && rotationRate < 1)
            {
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
    void drivingHeading()
    {
        drivingHeading(BrainInertial.rotation(degrees), 0, false);
    }

    void drivingHeading(float targetHeading, float expectedMM, bool perimeter)
    {
        if (cleaningFailed)
            return;
        motors.stop(brake);
        // Use the original 40% for initial straight driving; survey the perimeter at 20%.
        float velocity = perimeter ? 30 : 40;
        const float startMM = driveEncoderMM();
        float pausedMS = 0;
        int sideCorrections = 0;

        float Kp = 0.88;
        float Ki = 0.0;
        float Kd = 0.3;
        float integralZone = 20;
        PIDcontroller drivePID(Kp, Ki, Kd, integralZone);
        // Let the original PID's 500 ms startup interval finish while stationary
        // Otherwise its small correction output lets each new row drift sideway
        wait(500, msec);
        collisionMonitor.reset();
        const float MAX_CORRECTION = 10;

        timer driveTimer;
        driveTimer.clear();
        bool reachedEdge = false;
        while (!reachedEdge )
        {   
            const bool leftOn = DISTANCE.isObjectDetected() && DISTANCE.objectDistance(mm) < 100;
            const bool rightOn = OPTICAL.isNearObject();
            bool collidetect = collisionMonitor.detectCollisionLin();
            const float travelledMM = driveEncoderMM() - startMM;
            const float elapsedMS = driveTimer.time(msec) - pausedMS;
            if(leftOn && rightOn){
            if (BUMPER.pressing() || collidetect) //RICHARD LOOK HERE on this line
            {
                pausedMS += pauseForObstacle_heading();
                drivePID.previousError = 0;
                drivePID.previousTime = drivePID.PIDtimer.time(msec);
            }}
            if (!leftOn || !rightOn)
            {
                motors.stop(brake);

                // On the clockwise perimeter the outside edge is on the left.
                if (perimeter && !leftOn && rightOn)
                {
                    if (++sideCorrections > 4)
                    {
                        stopCleaning("Side correction limit");
                        return;
                    }
                    const float recoveryStart = driveTimer.time(msec);
                    turn_heading(headingError(targetHeading + 8));
                    if (cleaningFailed)
                        return;

                    // If the pivot restores both sensors, move a little inward.
                    if (DISTANCE.isObjectDetected() && DISTANCE.objectDistance(mm) < 100 &&
                        OPTICAL.isNearObject())
                    {
                        if (!driveDistance(40, 12))
                            return;
                        turn_heading(headingError(targetHeading));
                        if (cleaningFailed)
                            return;
                        pausedMS += driveTimer.time(msec) - recoveryStart;
                        drivePID.previousError = 0;
                        drivePID.previousTime = drivePID.PIDtimer.time(msec);
                        continue;
                    }

                    // A front corner can also trigger the left sensor first.
                    turn_heading(headingError(targetHeading));
                    if (cleaningFailed)
                        return;
                }

                if (expectedMM > 0 && travelledMM < expectedMM - EDGE_APPROACH_MM)
                {
                    stopCleaning("Unexpected early edge");
                    return;
                }
                reachedEdge = true;
                continue; // still need testing
            }
            const float limitMM = expectedMM > 0 ? expectedMM + 120 : 4000;
            if (elapsedMS > 65000 || travelledMM >= limitMM)
            {
                stopCleaning("Straight travel limit");
                return;
            }
            float speed = velocity;
            float error = headingError(targetHeading);
            float correctionMagnitude = drivePID.calculatePID(error);
            if (correctionMagnitude > MAX_CORRECTION)
                correctionMagnitude = MAX_CORRECTION;
            if (error < 0)
                correctionMagnitude = -correctionMagnitude;
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
    // Matt -------------------------------------------------------------------------------------------
    // Purpose: Moves the seesaw collector (only one side can touch the table at a time)
    // Params: side = -1 left side down, 0 level (both sides up), +1 right side down
    // Setup: at the start the user lowers the collector onto its RIGHT side and
    //        COLLECTOR.resetPosition() is called, so 0 deg = right down,
    //        -8.25 deg = level, -16.5 deg = left down (measured range is +-9.5 deg from level)
    // Outputs: true if it got there (going down, touching the table early counts)
    bool setCollector(int side)
    {
        if (cleaningFailed)
            return false;

        const float TILT_DEG = 8.25;
        const int RIGHT_TO_LEFT = -1;
        const float TOLERANCE_DEG = 2.0;
        const float SPEED_PERCENT = 15.0;

        // +1 = right down (0°), 0 = level (-9.5°), -1 = left down (-19°).
        const float target = RIGHT_TO_LEFT * TILT_DEG * (1 - side);

        motors.stop(brake);
        COLLECTOR.setMaxTorque(50, percent);

        timer collectorTimer;
        collectorTimer.clear();

        float error = target - COLLECTOR.position(degrees);
        bool touchedTable = false;

        while (fabs(error) > TOLERANCE_DEG &&
               !touchedTable &&
               collectorTimer.time(msec) < 1500)
        {
            // Recheck the direction each time in case the motor overshoots.
            COLLECTOR.spin(error > 0 ? forward : reverse, SPEED_PERCENT, percent);
            wait(10, msec);

            error = target - COLLECTOR.position(degrees);

            // Low motor speed after 300 ms can indicate contact while lowering.
            touchedTable = side != 0 &&
                           collectorTimer.time(msec) >= 300 &&
                           fabs(COLLECTOR.velocity(rpm)) < 2;
        }

        COLLECTOR.stop(hold);

        if (fabs(error) <= TOLERANCE_DEG || touchedTable)
            return true;

        stopCleaning(side == 0 ? "Collector not level" : "Collector timeout");
        return false;
    }

    // Richard ----------------------------------------------------------------------------------------
    void automaticCleaning()
    {
        const float MAX_STEP_MM = 80;
        int rows = 0;
        float stepMM = 0;
        cleaningFailed = false;

        motors.stop(brake);
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Collector RIGHT down");
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("then tap Touch");
        while (TOUCHLED.pressing())
            wait(10, msec);
        while (!TOUCHLED.pressing())
            wait(10, msec);
        while (TOUCHLED.pressing())
            wait(10, msec);

        COLLECTOR.resetPosition(); // user put the collector RIGHT side down before tapping
        if (!setCollector(0))
            return; // lift to level; fails right away if RIGHT_TO_LEFT is wrong

        wait(300, msec);
        calibrateSensors();
        TOUCHLED.setColor(color::green);
        BRUSH.spin(reverse, 50, percent);

        // Find the first edge and keep its normal as the reference for the run.
        drivingHeading();
        if (cleaningFailed)
            return;
        find_absolute_heading();
        if (cleaningFailed)
            return;
        const float edgeHeading = BrainInertial.rotation(degrees);

        // First reach a corner, then clean and measure four complete sides.
        float sidesMM[4];
        for (int side = 0; side < 5; ++side)
        {
            const float heading = edgeHeading + 90 * (side + 1);
            turn_heading(headingError(heading));
            if (cleaningFailed)
                return;
            const float startMM = driveEncoderMM();
            drivingHeading(heading, 0, true);
            if (cleaningFailed)
                return;
            find_absolute_heading();
            if (cleaningFailed)
                return;
            if (side > 0)
            {
                sidesMM[side - 1] = driveEncoderMM() - startMM;
                if (sidesMM[side - 1] <= 10)
                {
                    stopCleaning("Table span too short");
                    return;
                }
            }
        }

        // Compare opposite sides before using the measured spans for the rows.
        for (int axis = 0; axis < 2; ++axis)
        {
            const float longer = fmax(sidesMM[axis], sidesMM[axis + 2]);
            if (fabs(sidesMM[axis] - sidesMM[axis + 2]) > fmax(40.0f, longer * 0.10f))
            {
                stopCleaning("Perimeter mismatch");
                return;
            }
        }
        const float lengthMM = (sidesMM[0] + sidesMM[2]) / 2;
        const float widthMM = fmin(sidesMM[1], sidesMM[3]) - 10;
        rows = (int)ceil(widthMM / MAX_STEP_MM) + 1; // Add a row to eliminate dead zone
        stepMM = widthMM / (rows - 1);

        // Keep the original row loop. Each row follows one of two fixed headings.
        float shiftedMM = 0;
        bool finished = false;
        for (int row = 0; row < rows; ++row)
        {
            const float heading = edgeHeading + 180 + (row % 2) * 180;
            turn_heading(headingError(heading));
            if (cleaningFailed)
                return;
            drivingHeading(heading, lengthMM + BACK_MM, false);
            if (cleaningFailed)
                return;
            // Retreat and turn into the next lane.
            if (!driveDistance(-BACK_MM, 15))
                return;
            if (row + 1 < rows)
            {
                turn_heading(headingError(edgeHeading + 270));
                if (cleaningFailed)
                    return;
                const float startMM = driveEncoderMM();
                if (!driveDistance((row + 1) * stepMM - shiftedMM, 15))
                    return;
                shiftedMM += driveEncoderMM() - startMM;
                wait(200, msec);
            }
        }

        motors.stop(brake);
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Rows finished: %d", rows);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Span L %.0f W %.0f", lengthMM, widthMM + 10);
    }
    void driveMotor(motor &m, double speed)
    { // mannual control
        if (speed > MAX_SPEED)
        {
            speed = MAX_SPEED;
        }
        else if (speed < -MAX_SPEED)
        {
            speed = -MAX_SPEED;
        }

        if (fabs(speed) < 3)
        {
            m.stop(brake);
        }
        else
        {
            m.spin(forward, speed, percent);
        }
    }

    void Manual_Control()
    {
        motors.stop(brake);

        bool running = setCollector(0);
        int collectorState = 0;

        if (running)
            BRUSH.spin(reverse, 50, percent);

        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Manual: E Up to exit");

        while (running && !cleaningFailed &&
               CONTROLLER.installed() &&
               !CONTROLLER.ButtonEUp.pressing())
        {
            double forwardInput = CONTROLLER.AxisA.position();
            double turnInput = CONTROLLER.AxisC.position();

            // Ignore small joystick movements.
            if (fabs(forwardInput) < 5)
                forwardInput = 0;

            if (fabs(turnInput) < 10)
                turnInput = 0;

            // Left turn: -1, right turn: +1, straight/stopped: 0.
            int desiredCollector = 0;

            if (turnInput < 0)
                desiredCollector = -1;
            else if (turnInput > 0)
                desiredCollector = 1;

            // Adjust only when the requested collector position changes.
            if (desiredCollector != collectorState)
            {
                running = setCollector(desiredCollector);
                collectorState = desiredCollector;
                continue;
            }

            double leftSpeed = forwardInput + turnInput;
            double rightSpeed = forwardInput - turnInput;

            // Scale both speeds together to preserve the turning ratio.
            const double largest =
                fmax(fabs(leftSpeed), fabs(rightSpeed));

            if (largest > 100)
            {
                leftSpeed = leftSpeed / largest * 100;
                rightSpeed = rightSpeed / largest * 100;
            }

            driveMotor(LM, leftSpeed * MAX_SPEED / 100.0);
            driveMotor(RM, rightSpeed * MAX_SPEED / 100.0);

            wait(10, msec);
        }

        motors.stop(brake);
        BRUSH.stop(coast);
        COLLECTOR.stop(hold);
    }

    bool driveDistance(float targetDistance, float targetSpeed)
    {
        motors.stop(brake);
        if (cleaningFailed)
            return false;
        if (targetSpeed <= 0 || targetSpeed > MAX_SPEED)
            return false;

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

        //RICHARD LOOK HERE
        float pauseMS = 0;
        collisionMonitor.reset();

        while (travelled < targetDegree &&
               moveTimer.time(msec) - pausedMS < timeoutMS) //ADD COLLISION DETECTION HERE
        {
            if (BUMPER.pressing() || collisionMonitor.detectCollisionLin())
            {
                pausedMS += pauseForObstacle();
                drivePID.previousError = 0;
                drivePID.previousTime = drivePID.PIDtimer.time(msec);
            }
            
            if (direction == forward &&
                (!DISTANCE.isObjectDetected() || DISTANCE.objectDistance(mm) >= 100 ||
                 !OPTICAL.isNearObject()))
            {
                stopCleaning("Edge during distance move");
                return false;
            }
            float error = headingError(targetHeading);
            float correction = fmin(6.0f, drivePID.calculatePID(error));
            if (error < 0)
                correction = -correction;
            const float speed = direction == forward ? targetSpeed : -targetSpeed;
            LM.spin(forward, speed + correction, percent);
            RM.spin(forward, speed - correction, percent);
            wait(10, msec);
            travelled = (fabs(LM.position(degrees) - leftStart) +
                         fabs(RM.position(degrees) - rightStart)) /
                        2.0;
        }

        motors.stop(brake);
        if (travelled < targetDegree)
        {
            stopCleaning("Move timeout");
        }
        else if (direction == forward &&
                 (!DISTANCE.isObjectDetected() || DISTANCE.objectDistance(mm) >= 100 ||
                  !OPTICAL.isNearObject()))
        {
            stopCleaning("Edge during distance move");
            return false;
        }
        return travelled >= targetDegree;
    }
    float pauseForObstacle_heading()
    {
        if (!collisionMonitor.detectCollisionLin())
            return 0;

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
        bool pause = true;
        while (pause)
        {
            while (BUMPER.pressing() || TOUCHLED.pressing())  
                wait(20, msec);
            while (!TOUCHLED.pressing() && !BUMPER.pressing())
                wait(20, msec);
            if (BUMPER.pressing())
                continue;
            while (TOUCHLED.pressing() && !BUMPER.pressing())
                wait(20, msec);
            if (!BUMPER.pressing()){
                collisionMonitor.collisionCount = 0;
                collisionMonitor.collisionDetected = false;
                pause = false;
            }
        }

        TOUCHLED.setColor(color::green);
        return pauseTimer.time(msec);
    }
    float pauseForObstacle()
    {
        if (!BUMPER.pressing())
            return 0;

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
        bool pause = true;
        while (pause)
        {
            while (BUMPER.pressing() || TOUCHLED.pressing())  
                wait(20, msec);
            while (!TOUCHLED.pressing() && !BUMPER.pressing())
                wait(20, msec);
            if (BUMPER.pressing())
                continue;
            while (TOUCHLED.pressing() && !BUMPER.pressing())
                wait(20, msec);
            if (!BUMPER.pressing()){
                pause = false;
            }
        }

        TOUCHLED.setColor(color::green);
        return pauseTimer.time(msec);
    }
    void circle_algo()
    {
        const float CLEANING_WIDTH_MM = 210.0f;
        const float RADIUS_COMPENSATION_MM = 310.0f;
        const float SPOKE_CLEARANCE_MM = 35.0f;
        const float SPEED_PERCENT = 40.0f;
        const float RIM_START_TURN_DEG = 90.0f; // Right turn after reaching the rim after spoke cleaning
        const float RIM_SPEED_PERCENT = 12.0f;
        const float RIM_SLOWDOWN_PERCENT = 15.0f;
        const int RIM_TIMEOUT_MS = 240000;
        const float PI = 3.14159265f;
        const bool MEASURE_ONLY = false;

        // Set the collector's zero position and wait for a fresh tap
        cleaningFailed = false;
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Circular cleaning");
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Collector RIGHT down");
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Then tap Touch");
        while (TOUCHLED.pressing())
            wait(10, msec);
        while (!TOUCHLED.pressing())
            wait(10, msec);
        while (TOUCHLED.pressing())
            wait(10, msec);

        COLLECTOR.resetPosition();
        if (!setCollector(0))
            return;
        wait(300, msec);
        calibrateSensors();
        TOUCHLED.setColor(color::green);
        BRUSH.spin(reverse, 50, percent);

        // Face the first rim. The alignment function backs up by BACK_MM
        drivingHeading();
        if (cleaningFailed)
            return;
        find_absolute_heading();
        if (cleaningFailed)
            return;

        float nearBackoffMM = BACK_MM;
        float measuredRadiusMM = 0.0f;
        float outwardHeading = 0.0f;
    
        for (int crossing = 0; crossing <1; ++crossing)
        {

            turn_heading(180.0f);
            if (cleaningFailed || !setCollector(0))
                return;
            outwardHeading = BrainInertial.rotation(degrees);
            const float startMM = driveEncoderMM();
            drivingHeading(outwardHeading, 0, false);
            if (cleaningFailed)
                return;

            // Start was one backoff inside the rim. Add that distance to the span
            const float spanMM = driveEncoderMM() - startMM;
            measuredRadiusMM = (spanMM) / 2.0f;
            if (measuredRadiusMM <= BACK_MM)
            {
                stopCleaning("Invalid circular span");
                return;
            }
            // The robot is at the far edge. Reverse half the measured span
            if (!driveDistance(-measuredRadiusMM-60, SPEED_PERCENT))
                return;
            if (!DISTANCE.isObjectDetected() || DISTANCE.objectDistance(mm) >= 100 ||
                !OPTICAL.isNearObject())
            {
                stopCleaning("Center check failed");
                return;
            }
        }

        // Use the compensated radius for spacing and the measured radius for driving
        const float tableRadiusMM = measuredRadiusMM + RADIUS_COMPENSATION_MM;
        const float spokeLengthMM = measuredRadiusMM - SPOKE_CLEARANCE_MM;//deleted BACK_MM
        if (!isfinite(tableRadiusMM) || tableRadiusMM <= 0.0f || spokeLengthMM <= 0.0f)
        {
            stopCleaning("Invalid circular span");
            return;
        }

        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Radius est %.0f mm", tableRadiusMM);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Measured diam %.0f mm", 2.0f * measuredRadiusMM);
        if (MEASURE_ONLY)
            return;

        // Original pizza-slice formula. The angle from acos is in radians.
        const float width = CLEANING_WIDTH_MM;
        const float radius = tableRadiusMM;
        float cosine = 1.0f - (width * width) / (2.0f * radius * radius);
        if (cosine < -1.0f)
            cosine = -1.0f;
        const float maxTurnDeg = acosf(cosine) * 180.0f / PI;
        if (maxTurnDeg <= 0.0f)
        {
            stopCleaning("Invalid spoke angle");
            return;
        }

        int spokeCount = (int)ceilf(360.0f / maxTurnDeg);
        if (spokeCount < 3)
            spokeCount = 3;
        const float turnDeg = 360.0f / spokeCount;

        // For each spoke, clean outward and reverse back with the collector up.
        for (int spoke = 0; spoke < spokeCount; ++spoke)
        {
            const float target = outwardHeading + spoke * turnDeg;
            turn_heading(headingError(target));
            if (cleaningFailed || !setCollector(-1))
                return;
            const float startMM = driveEncoderMM();
            drivingHeading();
            if (!setCollector(0) || !driveDistance(-spokeLengthMM, SPEED_PERCENT))
                return;
        }

        // The last spoke returned to the center. Approach the rim and finish last touches
        drivingHeading();
        if (cleaningFailed)
            return;
        turn_heading(RIM_START_TURN_DEG);
        if (cleaningFailed || !setCollector(-1))
            return;

        // Follow the edge on the left until the gyro records one clockwise lap (heading == initial heading)
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Cleaning rim");
        const float rimStartRotation = BrainInertial.rotation(degrees);
        timer rimTimer;
        rimTimer.reset();
        float pausedMS = 0.0f;
        bool sawLeftOnTable = false;
        bool sawLeftOffTable = false;
        while (BrainInertial.rotation(degrees) - rimStartRotation < 360.0f)
        {
            pausedMS += pauseForObstacle();
            if (!OPTICAL.isNearObject())
            {
                stopCleaning("Right sensor off table");
                return;
            }
            if (rimTimer.time(msec) - pausedMS >= RIM_TIMEOUT_MS)
            {
                stopCleaning("Rim lap timeout");
                return;
            }

            const bool leftOnTable = DISTANCE.isObjectDetected() && DISTANCE.objectDistance(mm) < 100;
            float leftSpeed = RIM_SPEED_PERCENT;
            float rightSpeed = RIM_SPEED_PERCENT;
            if (leftOnTable)
            {
                sawLeftOnTable = true;
                leftSpeed -= RIM_SLOWDOWN_PERCENT; // Steer left toward the rim
            }
            else
            {
                sawLeftOffTable = true;
                rightSpeed -= RIM_SLOWDOWN_PERCENT; // Steer right toward the table
            }
            LM.spin(forward, leftSpeed, percent);
            RM.spin(forward, rightSpeed, percent);
            wait(20, msec);
        }
        motors.stop();
        if (!OPTICAL.isNearObject())
        {
            stopCleaning("Right sensor off table");
            return;
        }
        if (!sawLeftOnTable || !sawLeftOffTable)
        {
            stopCleaning("Rim edge not tracked");
            return;
        }
        if (!setCollector(0))
            return;

        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Finished Cleaning");
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Radius %.0f mm", tableRadiusMM);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Spokes %d + rim", spokeCount);
    }

    // Allow users to select cleaning mode
    static int cleaningSelection()
    {
        // 1 for rectangle, 2 for circle, 3 for manual
        int selection = 1;

        do
        {
            // find the right display
            if (selection == 1)
            {
                
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("Selection:");
                Brain.Screen.setCursor(2, 1);
                Brain.Screen.print("-Rectangle-");
                Brain.Screen.setCursor(3, 1);
                Brain.Screen.print("> Circle");
                Brain.Screen.setCursor(5, 2);
                Brain.Screen.print("Tap LED to select");
            }
            else if (selection == 2)
            {
                
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("Selection:");
                Brain.Screen.setCursor(2, 1);
                Brain.Screen.print("-Circle-");
                Brain.Screen.setCursor(3, 1);
                Brain.Screen.print("> Manual");
                Brain.Screen.setCursor(5, 2);
                Brain.Screen.print("Tap LED to select");
            }
            else if (selection == 3)
            {
                
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("Selection:");
                Brain.Screen.setCursor(2, 1);
                Brain.Screen.print("-Manual-");
                Brain.Screen.setCursor(3, 1);
                Brain.Screen.print("> Rectangle");
                Brain.Screen.setCursor(5, 2);
                Brain.Screen.print("Tap LED to select");
            }

            if (Brain.buttonRight.pressing())
            {
                Brain.Screen.clearScreen();
                selection++;
                if (selection > 3)
                {
                    selection = 1;
                }

                wait(500, msec);
            }
            else if (Brain.buttonLeft.pressing())
            {
                Brain.Screen.clearScreen();
                selection--;
                if (selection < 1)
                {
                    selection = 3;
                }
                wait(500, msec);
            }

        } while (!TOUCHLED.pressing());
        // wait for release
        TOUCHLED.setColor(color::red);
        while (TOUCHLED.pressing())
        {
            wait(10, msec);
        }
        return selection;
    }
};
/*
// Test functions----------------------------------------------------------------------------------
void testPIDTurning()
{
    turn_heading(40);
    wait(1, seconds);
    turn_heading(20);
    wait(1, seconds);
    turn_heading(90);
    wait(1, seconds);
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
*/
// -------------------------------------- MAIN ---------------------------------------------------
// int main()
// {
//     //Declare robot object
//     robot SweepBot;
//     SweepBot.automaticCleaning();
//     return 0;
// }

// NEW int main

int main()
{
    robot SweepBot;

    int selection = 0;
    // Get use to select the cleaning mode
    selection = robot::cleaningSelection();
    if (selection == 1)
    {
        TOUCHLED.setColor(color::green);
        TOUCHLED.setColor(color::yellow);
        SweepBot.automaticCleaning();
        Brain.programStop();
    }
    else if (selection == 2)
    {
        TOUCHLED.setColor(color::green);
        TOUCHLED.setColor(color::yellow);
        SweepBot.circle_algo();
        Brain.programStop();
    }
    else if (selection == 3)
    {
        TOUCHLED.setColor(color::green);
        TOUCHLED.setColor(color::yellow);
        SweepBot.Manual_Control();
    }

    return 0;
}
