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

MVP:
- Automatically clean standard rectangle table without assistance
- Debris collector that collects debris on the table surface
- Chemical disinfection along its cleaning path
- Manual Control
  
Robot Building Task Assignments:
- Matt: Frame, wiper
- Riley: Spinning rotor
- Richard: Frame, garbage collector
- Yunhao: 3D printing parts TBD

Program Task Assignments:
- Matt: Edge detection, v-shape collector program
- Riley: Trivial Functions (Driveforward, Backward, turnleft, turnright, PID for movement program)
- Richard: Manual Controller program, v-shape collector program
- Yunhao: PID controller (rotation)
