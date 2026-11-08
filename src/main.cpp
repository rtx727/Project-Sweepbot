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
distance DISTANCE(PORT1); //left side cliff sensor
motor_group motors(RM, LM);
optical OPTICAL(PORT6); //right side cliff sensor
controller CONTROLLER;
inertial BrainInertial = inertial();
touchled TOUCHLED(PORT8);
motor motor2(PORT2, true); //claw left
motor motor5(PORT5, false); //claw right

// Structs
const double MAX_SPEED = 50.0;

//Holds variable for collision boolean
struct collisionMonitor{
    bool collisionDetected;
    int collisionCount;
    timer jerkTimer;
    float previousTime;
    //Records acceleration values from the last loop to calculate jerk
    float prevAx;
    float prevAy;
    float prevAz;


    collisionMonitor()
    {
        collisionDetected = false;
        collisionCount = 0;
        jerkTimer.reset();
        previousTime = 0;
        prevAx = BrainInertial.acceleration(xaxis);
        prevAy = BrainInertial.acceleration(yaxis);
        prevAz = BrainInertial.acceleration(zaxis);
    }
    ~collisionMonitor()
    {
        // Destructor
    }

    bool detectCollisionLin(){
        const float JERK_THRESHOLD = 6.0; //Collision threshold in m/s^3 
        
        //Calculate time difference since last reading
        float currentTime = jerkTimer.time(msec) / 1000.0; // Convert to seconds
        float dt = currentTime - previousTime;

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

            //If jerk exceeds threshold, increment collision count
            if (jerk > JERK_THRESHOLD){
                collisionCount++;
            }

            //Only register collison if loop happens more than 2 times but less than 5 times
            if (collisionCount > 2 && collisionCount < 5 && !collisionDetected){
                collisionDetected = true;
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("Collision Detected");
            }
            //Probably not a collision if it happens more than 5 times in a row, reset the counter
            else if (collisionCount >= 5){
                collisionDetected = false;
                collisionCount = 0;
            }
            
        }
        //Update member variables
        previousTime = currentTime;
        prevAx = currAx;
        prevAy = currAy;
        prevAz = currAz;
        return collisionDetected;
    }
    
    //
    bool detectCollisionAng(){

        return collisionDetected;
    }

};


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
void turn_heading(float targetHeading, collisionMonitor &collision_monitor);
void find_absolute_heading();

// Calibrate all sensors
void calibrateSensors();

void driveHeading(float velocity, float targetHeading);

// Test functions----------------------------------------------------------------------------------
void testPIDTurning(collisionMonitor &collision_monitor)
{
    turn_heading(40, collision_monitor);
    wait(1, seconds);
    turn_heading(20, collision_monitor);
    wait(1, seconds);
    turn_heading(90, collision_monitor);
    wait(1,seconds);
    turn_heading(-150, collision_monitor);
    wait(1, seconds);
    turn_heading(-40, collision_monitor);
    wait(1, seconds);
    turn_heading(-20, collision_monitor);
    
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

    //Construct collision monitor object
    collisionMonitor collision_monitor;

    // Setup events
    // TOUCHLED.pressed(handle_TOUCLED,end_autonomous, TOUCLED_COLOR));
    TOUCHLED.setColor(color::red); // Start autonomous program

    // Tests
    test_find_absolute_heading(); // call a PID that is more simple with no ki
    //testPIDTurning(collision_monitor););

    TOUCHLED.setColor(color::blue); // End autonomous program
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
}

// Purpose: Finds the absolute heading of the robot
// Inputs: None
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
        float reverse_time = (left_time > right_time) ? left_time : right_time;
        reverse_time += 50; // Extra padding to clear the edge safely

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


// Params: Heading (+ or -), collision monitor object
// Purpose: Turns using gyro sensor, assumes current heading is zero
// Outputs: None
void turn_heading(float targetHeading, collisionMonitor &collision_monitor)
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

    float integralZone = 90; //only apply Ki in this zone (degrees of a turn)
    PIDcontroller turnPID = PIDcontroller(Kp, Ki, Kd, integralZone);

    task::sleep(20); // Allow time or timer to become not zero for first loop iteration

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

        //check for collision
        if (collision_monitor.detectCollisionLin()){
            motors.stop(brake);
            while (true){
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(5, 1);
                Brain.Screen.print("Waiting for user to clear collision");
            }
        }
    }
    motors.stop();
    return;
}

// Params: Heading (+ or -)
// Purpose: Turns using gyro sensor, assumes current heading is zero
// Outputs: None
//OVERLOADED FUNCTION FOR HEADING DETECTION, NO COLLISION MONITORING
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

    float integralZone = 90; //only apply Ki in this zone (degrees of a turn)
    PIDcontroller turnPID = PIDcontroller(Kp, Ki, Kd, integralZone);

    task::sleep(20); // Allow time or timer to become not zero for first loop iteration

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
    }
    motors.stop();
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

void controlClaw(motor &m, bool collect, bool release) {
    double position = m.position(degrees);
    double speed = MAX_SPEED/3; 

    if (collect && position < 120) {

        if (120 - position < 20) {
            speed = 5;
        }

        m.spin(forward, speed, percent);
    }
    else if (release && position > 0) {

        if (position < 20) {
            speed = 5;
        }

        m.spin(reverse, speed, percent);
    }
    else {

        m.stop(hold);
    }
}

void Manual_Control() {


    motor2.stop(hold);
    motor5.stop(hold);

    motor2.resetPosition();
    motor5.resetPosition();

    while (true) {

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

        bool collect = clawInput < -20;
        bool release = clawInput > 20;


        controlClaw(motor2, collect, release);
        controlClaw(motor5, collect, release);

        wait(10, msec);
    }
}
// Matt -------------------------------------------------------------------------------------------
