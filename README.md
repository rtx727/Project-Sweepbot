# Project-Sweepbot
This is the Research and Development Sector for the Autonomous Robot Wiper's Firmware.
The minimum viable product should contain a functional wiper which:
- Perform the task unassisted on a standardized rectangular table with uniform surface colour.
- Navigates to travel in a pattern that wipes the surface entirely.
- Can be controlled wirelessly with a controller for someone to have fun with.
- Cleaning methods include at least a debris collector, and a form of chemical disinfection.
- Avoids obstacles on the table, the robot will stop and inform the user to remove the obstruction.
- Can be easily maintained after usage.

Project Management:
- Log all updates to the robot development through git, including CAD and programs (Take Photos of Prototypes if relevant)
- Some ideas for navigation, if auto detection fails, use button to determine the geometry of the applied table
- Formal presentation information curation process could begin simultaneously during development
- Videos of PID tuning

MVP:
- Automatically clean standard rectangle table without assistance
- Debris collector that collects debris on the table surface
- Chemical disinfection along its cleaning path
- Manual Control
  
Robot Building Task Assignments:
- Matt: Frame, wiper
- Riley:
- Richard: Frame, garbage collector
- Yunhao: 3D printing parts TBD

Program Task Assignments:
- Matt: Edge detection, v-shape collector program
- Riley: Trivial Functions, Tough LED handle inputs
- Richard: Manual Controller program, v-shape collector program
- Yunhao: ~~PID controller~~, ~~first edge detection algorithim~~, collision detection using accelerometer

Weekend 1 tasks:
-Solve lack of friction problem
-Begin collision detection code
-Get first trial in: ->Use find absolute heading and a loop to travel down the table to wipe all of it.
DO NOT DELAY THIS. ABSOLUTELY CRITICAL TO START FINDING ISSUES.
-Think of how to incorporate functions together into a program
-Clean up code on spare time


Collision logic:
-Find unexpected high jerk for multiple loops. This implies a collision. 
-A collision monitor object is created in int main which needs to be passed into all movement functions.
-In checking loops of the movement functions, the object's method of calculating jerk is called.
The function will be changed from void to bool such that it returns the boolean of whether a 
collision has been detected.


Edge detection logic:
-Get two sensors to fall of table at exact same time to get absolute heading
-perform this calibration multiple times throughout the cleaning process
