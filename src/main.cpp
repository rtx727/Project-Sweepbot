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
motor motor2(PORT2, true); //claw left
motor motor5(PORT5, false); //claw right

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
bool driveDistance(float targetDistance, float targetSpeed);
void automaticCleaning();
float pauseForObstacle();

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
    } // Allow time for calibration
    
    BrainInertial.resetRotation();
}

// Purpose: Finds the absolute heading of the robot
// Outputs: None
void find_absolute_heading() {
    // Declare timers
    timer leftTimer;
    timer rightTimer;
    timer reverseTimer;

    float left_time = 0;
    float right_time = 0;
    
    const float ACCEPTABLE_TIME_ERROR = 50; // Milliseconds (Note: check if your PID output scales well to this)
    const float SENSOR_HEIGHT = 50; 
    float error = 0; //In milliseconds

    // Instantiate PID outside the loop so it retains memory (Integral/Derivative)
    float kp = 0.04;
    float ki = 0.0; 
    float kd = 0.0; 
    float integralZone = 500; // only use ki when error is within this zone (msec)
    PIDcontroller turnPID = PIDcontroller(kp, ki, kd, integralZone);

    do {
        // Reset flags and variables for this attempt
        bool left_edge_found = false;
        bool right_edge_found = false;
        left_time = 0;
        right_time = 0;

        leftTimer.clear();
        rightTimer.clear();

        // Drive forward until BOTH sensors have detected the edge
        while (!left_edge_found || !right_edge_found) {
            
            // If right sensor is still on the table, keep driving right motor
            if (OPTICAL.isNearObject() && !right_edge_found) {
                // You can replace this with your straight PID driving logic
                motors.spin(forward, 30, percent); 
            } else if (!right_edge_found) {
                // Right sensor just hit the edge! Record time and mark true
                right_time = rightTimer.time(msec);
                right_edge_found = true;
            }

            // If left sensor is still on the table, keep driving left motor
            if (DISTANCE.objectDistance(mm) < SENSOR_HEIGHT && !left_edge_found) {
                motors.spin(forward, 30, percent);
            } else if (!left_edge_found) {
                // Left sensor just hit the edge! Record time and mark true
                left_time = leftTimer.time(msec);
                left_edge_found = true;
            }
            
            // Small sleep to prevent CPU hogging
            task::sleep(10); 
        }

        // Stop completely once both edges are found
        motors.stop(brake);
        task::sleep(100); // Allow physical momentum to settle

        // Calculate recovery time based on whichever sensor drove furthest off the table
        //float reverse_time = (left_time > right_time) ? left_time : right_time;
        //reverse_time += 15; // Extra padding to clear the edge safely
        float reverse_time = 300;
        // Pull back onto the table safely
        reverseTimer.clear();
        while (reverseTimer.time(msec) < reverse_time) {
            motors.spin(reverse, 30, percent);
        }
        motors.stop(brake);

        // Calculate alignment error
        error = left_time - right_time; 

        // If error is outside acceptable limits, execute correction turn
        if (fabs(error) > ACCEPTABLE_TIME_ERROR) {
            //float turnAmount = turnPID.calculatePID(error); //Degrees turned (MAX output is 100)
            float turnAmount = fabs(error)*.014;

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
// Purpose: Turns using gyro sensor, assumes current heading is zero
// Outputs: None
void turn_heading(float targetHeading)
{
    // Stop all motors
    motors.stop(brake);

    // Set current heading to 0;
    BrainInertial.resetRotation();
    float error = targetHeading;  // Initial error is just heading
    float acceptableError = 0.25; // degrees
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
    // Perform movement until error is acceptable
    while (fabs(error) > acceptableError)
    {
        // continuously find current heading and error
        currentHeading = BrainInertial.rotation(degrees);
        error = targetHeading - currentHeading;
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("%f", currentHeading);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("%f", error);

        // Determine which direction to turn
        if (error > 0)
        {                                             // right turn
            motorSpeed = turnPID.calculatePID(error); // Pass in error to PIDcontroller for output

            RM.spin(reverse, motorSpeed, percent);
            LM.spin(forward, motorSpeed, percent);
        }
        else
        {                                             // left turn
            motorSpeed = turnPID.calculatePID(error); // Pass in error to PIDcontroller for output

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
            settled = true;
        }
    }
    motors.stop();
    return;
}

// Riley ------------------------------------------------------------------------------------------

// Params: Velocity, heading
// Purpose: Drives forward using gyro sensor
// Outputs: None
void drivingHeading()
{ 
    float targetHeading = BrainInertial.rotation(degrees);
    float velocity = 40;

      // Record encoder positions AFTER the turn is finished
      //const float leftStart = LM.position(degrees);
      //const float rightStart = RM.position(degrees);
    
      float Kp = 0.88;
      float Ki = 0.0;
      float Kd = 0.3;
      float integralZone = 20;
    
      PIDcontroller drivePID(Kp, Ki, Kd, integralZone);
    
      //const float SENSOR_HEIGHT = 50; // mm, calibrate this
      const float MAX_CORRECTION = 10; // motor speed percentage
    
      timer driveTimer;
      driveTimer.clear();
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1,1);
    
      // Safety timeout -- adjust for your table dimensions
      while (DISTANCE.objectDistance(mm)<50 && !BUMPER.pressing() && OPTICAL.isNearObject()){// added optical detection
        // Continuous inertial heading feedback

        float currentHeading = BrainInertial.rotation(degrees);
        Brain.Screen.print("%f", targetHeading);
        Brain.Screen.setCursor(2,1);
        Brain.Screen.print("%f", currentHeading);
        float error = targetHeading - currentHeading;
        Brain.Screen.setCursor(3,1);
        Brain.Screen.print("%f",error);
        float correctionMagnitude = drivePID.calculatePID(error);
    
        if (correctionMagnitude > MAX_CORRECTION){
          correctionMagnitude = MAX_CORRECTION;
        }
    
        // Restore the direction of the correction
        if (error < 0){
          correctionMagnitude = -correctionMagnitude;
        }
        float correction = correctionMagnitude;
    
        // Ignore very small heading errors
        if (fabs(error) < 0.5){
            correction = 0;
        }
    
        float leftSpeed = velocity + correction;
        float rightSpeed = velocity - correction;
    
        LM.spin(forward, leftSpeed, percent);
        RM.spin(forward, rightSpeed, percent);
    
        task::sleep(20);
    }
    motors.stop();
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

bool controlClaw(motor &m, bool open) {
  double target = open ? 30 : 0;
  m.spinToPosition(target, degrees,true);
  bool reached = fabs(m.position(degrees) - target) <= 3;
  if (reached) {
    m.stop(hold);
  } else {
    m.stop(brake);
  }
  return reached;
}

void Manual_Control() {


    motor2.stop(hold);
    motor5.stop(hold);


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


    double clawInput = CONTROLLER.AxisC.position();

    bool right_release = clawInput > 20;
    bool left_release = clawInput < -20;


    controlClaw(motor2, left_release);
    controlClaw(motor5, right_release);
    
    wait(10, msec);
  }
  motors.stop();
}

void automaticCleaning() {
    const float MAX_STEP_MM = 80;
    const float BACK_MM = 50;
    int rows = 0;
    float stepMM = 0;

    motors.stop(brake);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Place robot, tap Touch");
    while (TOUCHLED.pressing()) wait(10, msec);
    while (!TOUCHLED.pressing()) wait(10, msec);
    while (TOUCHLED.pressing()) wait(10, msec);

    motor2.setPosition(0, degrees);
    motor5.setPosition(0, degrees);
    motor2.setVelocity(15, percent);
    motor5.setVelocity(15, percent);
    motor2.setTimeout(3, seconds);
    motor5.setTimeout(3, seconds);
    motor2.stop(hold);
    motor5.stop(hold);
    wait(300, msec);
    calibrateSensors();

    for (int row = 0; ; row++) {
        drivingHeading();
        if (rows > 0 && row == rows - 1) break;

        if (!driveDistance(-BACK_MM, 15)) return;
        if (row == 0) find_absolute_heading();

        float angle = row % 2 == 0 ? 90 : -90;
        motor &claw = angle < 0? motor2 : motor5;
        if (!controlClaw(claw, true)) return;
        turn_heading(angle);
        if (!controlClaw(claw, false)) return;
        wait(200, msec);

        if (row == 0) {
            // First right turn: measure the available sideways travel.
            float leftStart = LM.position(degrees);
            float rightStart = RM.position(degrees);
            drivingHeading();
            float measuredMM = (
                LM.position(degrees) - leftStart +
                RM.position(degrees) - rightStart
            ) / 2.0 * 200.0 / 360.0 + 100;

            // Leave 10 mm before the measured side stop for the last lane.
            float widthMM = fmax(0.0, measuredMM - 10.0);
            rows = (int)ceil(widthMM / MAX_STEP_MM) + 1;
            if (rows == 1) break;
            stepMM = widthMM / (rows - 1);

            // Return directly to lane two, not all the way to lane one.
            if (!driveDistance(-(measuredMM - stepMM), 15)) return;
        } else {
            if (!driveDistance(stepMM, 15)) return;
        }
        if (!controlClaw(claw, true)) return;
        turn_heading(angle);
        if (!controlClaw(claw, false)) return;
        wait(200, msec);
    }

    motors.stop(brake);
    motor2.stop(hold);
    motor5.stop(hold);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Rows finished: %d", rows);
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
    if (targetSpeed <= 0 || targetSpeed > MAX_SPEED) return false;

    float targetDegree = fabs(targetDistance) / 200.0 * 360.0;
    float leftStart = LM.position(degrees);
    float rightStart = RM.position(degrees);
    float travelled = 0;
    float pausedMS = 0;
    float timeoutMS = 5000 + fabs(targetDistance) * 50;
    directionType direction = targetDistance >= 0 ? forward : reverse;
    timer moveTimer;
    moveTimer.reset();

    while (travelled < targetDegree &&
           moveTimer.time(msec) - pausedMS < timeoutMS) {
        pausedMS += pauseForObstacle();
        motors.spin(direction, targetSpeed, percent);
        wait(10, msec);
        travelled = (
            fabs(LM.position(degrees) - leftStart) +
            fabs(RM.position(degrees) - rightStart)
        ) / 2.0;
    }

    motors.stop(brake);
    if (travelled < targetDegree) {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Move timeout");
    }
    return travelled >= targetDegree;
}
// Matt -------------------------------------------------------------------------------------------
/*bool driveDistance(float targetDistance, float targetSpeed){
  const float wheelCirc = 200.0;//In mm
  motors.stop(brake);

  float targetDegree = targetDistance / wheelCirc * 360.0;

  //float leftStart = LM.position(degrees); //Modified by Richard, so that it can drive forward or backward
  //float rightStart = RM.position(degrees);
  float travelled = 0;
  directionType direction = targetDistance >= 0 ? forward : reverse;
  timer moveTimer;
  moveTimer.reset();
  motors.spin(direction,targetSpeed,percent);
  while(travelled<targetDistance){
    wait(10,msec);
  }
  motors.stop(brake);
  return true;
}*/

void test_edge_detection(){
  for(int i = 0; i < 4; i++){
    find_absolute_heading();

    turn_heading(90);

    drivingHeading();
  }
}
