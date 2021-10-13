# VexCode2021-2022

#### Important Files
```
RobotCode/src/Configuration.hpp  -> contains the motor and sensor port definitions and some other stuff
RobotCode/src/DriverControl.cpp  -> contains all the code for driving
RobotCode/src/Autons.hpp         -> code for writing the autons, should be easy enough to follow (don't forget to update the debug auton number in the implementation file
```

#### Upload instructions
```
open command prompt
use dir to list the files you can navigate to next
use cd to change to the next directory on the path of the project
(if you need help with these steps find a tutorial like this one https://www.youtube.com/watch?v=NDbNVxg7NOY)
when in the RobotCode directory you can use the build and upload commands

To build:
    prosv5 make                   -> use this one to build only the changed files (faster)
    prosv5 make all               -> use this one to rebuild everything (slower but can fix weird bugs sometimes)
Once project is built successfully, you can upload it to the robot
To upload (you can use multiple "--" flags)
    prosv5 upload                 -> this will upload the project to slot 1
    prosv5 upload --slot n        -> uploads to slot 'n' (1 <= n <= 8)
    prosv5 upload --name name     -> this sets the name of the program (maybe useful to you)
    prosv5 upload --help          -> will help you
To debug
    prosv5 terminal               -> open a terminal to see stdout to help with debugging (especially pid)
```

# Autonomous movement

### Setting pid parameters

Each function has its own set of pid parameters and each set of parameters can be dynamically updated at any time. The default values are located towards the top of ```RobotCode/src/objects/subsystems/chassis.cpp``` and after the ```generate_chassis_velocity_profile``` function. Setting these parameters are done with one of the following functions
```
chassis.set_pid_sdrive_gains({kP, kI, kD, integral_max, slew_rate)});
chassis.set_profiled_sdrive_gains({kP, kI, kD, integral_max, slew_rate)});
chassis.set_okapi_sdrive_gains({kP, kI, kD, integral_max, slew_rate)});
```

In addition, there are also pid parameters for the heading correction and for turning:
```
chassis.set_heading_gains({kP, kI, kD, integral_max, slew_rate)});

chassis.set_turn_gains({kP, kI, kD, integral_max, slew_rate)});
```

For each of the straight drive functions, heading correction can be disable by either setting the correct_heading parameter to false or by setting kP, kI, and kD of the heading parameters to 0

### moving straight

there are four functions for straight movement currently
* pid_straight_drive
* profiled_straight_drive
* okapi_pid_straight_drive
* drive_to_point (uses okapi_pid_straight_drive internally)

Each has parameters similar to the following, however most of them are optional and do not need to be changed
* encoder_ticks  ->  the number of ticks to move based on the free spinning wheels
* relative_heading  ->  for the love of all things good on earth do not set this to anything other than 0
* max_velocity  ->  the maximum velocity allowed (lower = more stable/consistent)
* timeout  ->  if the function takes longer than this it will return (good to ensure there is no just sitting there randomly because it can't reach the settled threshold)
* asynch  ->   if true this function returns right away so other functions can be called afterwards
* correct_heading  ->  set to true to correct heading (set to false if weird things are happening)
* slew  ->   the max change in voltage/velocity at each step (might be useful, to disable set to INT32_MAX)
* log_data  ->  log_data (might be useful while debugging)

check ```RobotCode/src/objects/subsystems/chassis.hpp``` for function definitions to know which parameters each function can take (most are optional but helpful, these are not well documented but should be relatively self explanatory and are explained above)

### asynchronous movement

most functions in the chassis subsystem allow you to return right after calling it and will return a uid to track when the function finishes. This is useful if you want to run another subsystem at the same time. This might end up looking like this (be sure to set the asynch flag to true if you want it to return immediately):
```
int uid = chassis.okapi_pid_straight_drive(1000, 6000, 3000, true, 2200);
while(!chassis.is_finished(uid)) {
    indexer.increment();  // this function needs to be called at each step so that it can decide when to stop the rollers when a ball is present
    pros::delay(5);
}
```

or

```
int uid = chassis.okapi_pid_straight_drive(1000, 6000, 3000, true, 2200);
indexer.start();
chassis.wait_until_finished(uid);
```


# Tuning PID Loops
This is a long and tedious process. Here is the method I use to tune them. First some terminology:
* setpoint - the target for the sensor
* error - the distance between the actual sensor value and the setpoint
* kP - the proportional constant
* kI - the integral constant
* kD - the derivative constant
* I_max - max value for the integral

Then read pages 12 and 13 of this document (the rest of it is also pretty interesting but not as important for you) http://smithcsrobot.weebly.com/uploads/6/0/9/5/60954939/pid_control_document.pdf

Part 1
1. for straight driving choose a setpoint to see a decent amount of movement (I usually go somewhere around 1000 to 2000) for turning start with a setpoint of 90 degrees going either left or right
2. set kP to 1, kI to 0, kD to 0, and I_max to INT32_MAX
3. increase kP until the robot reaches the setpoint with a reasonable speed and oscillates around the setpoint
4. increase kD to dampen the oscillations (usually ends up being much higher than kP). If kD is too high the robot will likely not reach the setpoint
5. If it works well here go to the next section. If it is not quite reaching the setpoint continue on
6. decrease kP by a little bit
7. start with a very small kI (~.0001 - .001) and slowly increase until the robot starts oscillating again
8. increase kD until you dampen the oscillations
9. increase kP, decrease kI, increase kD to maybe help fix it
10. use your intuition and the table at the end of the document listed previously to play around with the constants

Part 2
1. Choose some different setpoints -- with 10 you can be confident it probably is tuned well enough (higher, lower, and everything in between). These setopints should go forward, backwards, left, or right. Variety of tests will ensure you have a robustly tuned control loop
2. Skip this step (or don't). When it doesn't work as well for all of these setpoints re-evaluate your life and why you chose to be sitting in the programmers chair late at night slowly losing your sanity when you have an english paper to write ordering someone to bring the robot back to the table and plug it in while you make adjustments to the pid constants while they are dying of boredom and saying braindead, incoherent things the whole 6 hours you are trying to get this one stupid loop tuned
3. go back to part 1 and make small adjustments or do the janky trick of setting different constants for different setpoints in the autonomous
4. Call it good enough at some point and be sure to add timeouts in the code


# Adding more auton functions
1. go to ```RobotCode/src/Autons.hpp```
2. under each of the maps AUTONOMOUS_NAMES, AUTONOMOUS_DESCRIPTIONS, AUTONOMOUS_COLORS add an entry with an in order number and the data necessary (follow the syntax of the others ones)
3. make sure to not delete driver control and debug from the entries and to keep these at the first and last
4. add a function for the new auton the syntax should be ```void skills();``` where skills is the name of the auton function
5. go to ```RobotCode/src/Autons.cpp```
6. update the debug_auton_num variable in the Autons class constructor (should be the first function)
7. ensure driver control auton is in fact 1
8. implement the function added in the .hpp file this looks like ```void Autons::skills() {}``` (read [this](http://www.cppforschool.com/tutorial/separate-header-and-implementation-files.html))
9. add the code for the function between the brackets there are some default lines that you can find in the example autons (just copy them over)
10. find the function ```run_autonomous```
11. in the switch statement, add a case for the auton number you added in order (the case number is the number assigned to it in the .hpp file in one of the maps)
12. ensure clean code by having the swich cases in numerical order
13. note that debug does not need a case
14. note the switch statement should call the auton function you created and end with ```break;``` (this should be obvious from looking at what is already there)
