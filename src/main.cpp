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
vex::brain Brain;

// define your global instances of motors and other devices here
motor RM(PORT12, bool(reverse));
motor LM(PORT7, bool(forward));
bumper BUMPER(PORT3);
distance DISTANCE(PORT1); //left side
motor_group motors(RM, LM);
optical OPTICAL(PORT6); //right side
controller CONTROLLER;
inertial BrainInertial = inertial();
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
    float error;
    float previousError;
    float integral;
    float output;

    // Constructor - sets up K values
    PIDcontroller(float Kp, float Ki, float Kd)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        error = 0;
        previousError = 0;
        integral = 0; // Tracks accumulated error if not responsive enough
        output = 0;
    }

    ~PIDcontroller()
    {
        // Destructor
    }

    // PID calculation function
    // Params: target and current position
    // Output: Positive output for motorspeed in percentage
    float calculatePID(float error)
    {
        // Take in error
        this->error = error;

        // Porportional Term
        float porportionalTerm = this->error * this->Kp;

        // Integral Term
        // Add the error to the integral
        this->integral = this->integral + this->error;
        float integralTerm = this->integral * this->Ki;

        // Derivative Term
        float derivative = this->error - this->previousError;
        float derivativeTerm = derivative * this->Kd;
        this->previousError = this->error;

        // Make output term
        this->output = fabs(porportionalTerm + integralTerm + derivativeTerm);

        // Max motor speed is 100%
        if (this->output > 100)
        {
            this->output = 100;
        }

        return this->output; // outputs motor speed
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

void driveHeading(float velocity, float targetHeading);

// Test functions----------------------------------------------------------------------------------
void testPIDTurning()
{
    turn_heading(90);
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
    // bool end_autonomous = false;
    color TOUCLED_COLOR = color::green;

    // Calibration
    calibrateSensors();
    Brain.Screen.printAt(2, 30, "Hello IQ2");
    Brain.Screen.clearScreen();

    // Cosntruct robot object
    // robot table_cleaner;

    // Setup events
    // TOUCHLED.pressed(handle_TOUCLED,end_autonomous, TOUCLED_COLOR));
    TOUCHLED.setColor(color::red); // Start autonomous program

    // Tests
    test_find_absolute_heading();
    //testPIDTurning();

    // End program
    // Brain.programStop();
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
    Brain.Screen.printAt(2, 70, "Calibration Complete");
    Brain.Screen.clearScreen();
}

// Purpose: Finds the absolute heading of the robot
// Outputs: None
void find_absolute_heading()
{
    // Declare timers
    timer leftTimer;
    timer rightTimer;
    timer reverseTimer;
    float left_time;
    float right_time;
    const float ACCEPTABLE_TIME_ERROR = 0.5; // Based on tuning
    const float SENSOR_HEIGHT = 50; //mm, based on sensor placement
    float error = 0; // Error between the two times

    do
    {
        leftTimer.clear();
        rightTimer.clear();

        // Drive forward slowly until each sensor goes over the edge
        while(OPTICAL.isNearObject() && DISTANCE.objectDistance(mm) < 50){
            
            //Replace with PID controller for straight driving
            motors.spin(forward, 10, percent);

            //Optical sensor is on the left side of the robot, distance sensor is on the right side
            if (OPTICAL.isNearObject() == false)
            {
                left_time = leftTimer.time(msec); // Record time when left sensor goes over the edge
            }
            if (DISTANCE.objectDistance(mm) > 50)
            {
                right_time = rightTimer.time(msec); // Record time when right sensor goes over the edge
            }
        }
        // stop when both sensors are over the edge
        motors.stop(brake);

        //reverse a little to get both sensors back on the table
        //reverse the amount of time for the sensor that went over last, plus a little extra to ensure both sensors are back on the table
        float reverse_time = 0;
        //Reverse for the same amount of time as the sensor that went over last
        reverse_time = (left_time > right_time) ? left_time : right_time;
        reverse_time += 100; // Add 100ms to ensure both sensors are back on the table
        reverseTimer.clear();
        motors.spin(reverse, 10, percent); // Replace with PID controller for straight driving
        while (reverseTimer.time(msec) < reverse_time)
        {
        }
        motors.stop(brake);

        // make correct turn based on error between the two sensors, repeat until error is acceptable
        error = left_time - right_time;

        //Make PID controller
        float kp = 0.5; // Proportional gain, based on tuning
        float ki = 0.0; // Integral gain, based on tuning
        float kd = 0.0; // Derivative gain, based on tuning
        PIDcontroller turnPID = PIDcontroller(kp, ki, kd);
        float turnAmount = 0; // In percentage

        // Call turn function if there is error between the two sensors
        if (fabs(error) > ACCEPTABLE_TIME_ERROR){
            turnAmount = turnPID.calculatePID(error); // Pass in error to PIDcontroller for output

            //Determine which sensor went over the edge first and turn in the same direction
            if (error > 0)
            { // right sensor went over first, turn right
                turn_heading(turnAmount);
            }
            else
            { // left sensor went over first, turn left
                turn_heading(-(turnAmount));
            }
        }
        else{
            motors.stop(brake);
        }

        // Print times to brain screen for debugging
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Left Time: %f", left_time);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Right Time: %f", right_time);

    } while (fabs(left_time - right_time) < ACCEPTABLE_TIME_ERROR);
    //Repeat until both sensors reach edge at roughly same time

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Absolute Heading found");

    return;
}

// Params: Heading (+ or -)
// Purpose: Turns gyro sensor, assumes current heading is zero
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
    float Kp = 0.7;
    float Ki = 0.00003;
    float Kd = 0.8;
    PIDcontroller turnPID = PIDcontroller(Kp, Ki, Kd);

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
    }
    return;
}

// Riley ------------------------------------------------------------------------------------------

// Params: Velocity, heading
// Purpose: Drives forward using gyro sensor
// Outputs: None
void driveHeading(float velocity, float targetHeading)
{
    // Maybe use PID controller to subtract output from one wheel and add to another?
}

// Richard ----------------------------------------------------------------------------------------

// Matt -------------------------------------------------------------------------------------------
