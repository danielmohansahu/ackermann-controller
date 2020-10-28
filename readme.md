# Ackermann Controller
[![Build Status](https://travis-ci.org/danielmohansahu/ackermann-controller.svg?branch=master)](https://travis-ci.org/danielmohansahu/ackermann-controller)
[![Coverage Status](https://coveralls.io/repos/github/danielmohansahu/ackermann-controller/badge.svg?branch=master)](https://coveralls.io/github/danielmohansahu/ackermann-controller?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---

## Overview

This is an implementation of an [Ackermann Steering Geometry](https://en.wikipedia.org/wiki/Ackermann_steering_geometry) controller. The user is able to input a desired heading and speed in the global reference frame which the controller will drive the robot model towards, based upon rover parameters and limitations.

To calculate the Ackermann steering angles for a four wheeled vehicles, a simplified model with a single front and rear wheel is assumed. The steering angle of this simulated single steering wheel is used to calculate the updated heading of the rover along with the vehicle wheel base. For an Ackermann steering system, the two actual steering wheels are perpindicular to the radius of the turning circle traced by each wheel; the difference in angles for each of these wheels can be calculated based on the track of the rover. The different turning circles and the vehicle speed are used to calculate individual wheel speeds.
  
## Dependencies

This project requires a C++11 enabled compiler and `cmake`. In addition, we use QT5 for visualization of our demo.

QT5 (and QtCharts) can be installed via the following command on Ubuntu 18.04:

```bash
sudo apt-get install qt5-default libqt5charts5-dev
```

## Usage Documentation
### Build Instructions

To build the package on Ubuntu 18.04, run the following from a terminal.

```bash
git clone https://github.com/danielmohansahu/ackermann-controller
cd ackermann-controller
mkdir build && cd build
cmake .. && make
```

### Demonstration Instructions

Assuming the build succeeded, you can then run the demo code.

```bash
# from your build directory (e.g. ackermann-controller/build/)
./app/demo
```

You should see something similar to the following, which allows you to evaluate the system and play with parameters against a mock Plant.

Empty | Running
 --- | ---
 !["Empty Demo"](docs/media/empty_demo.png) | !["Running Demo"](docs/media/running_demo.png)

### Testing Instructions

Unit and System tests are run during Continuous integration, but you can run them manually from the command line as well:

```bash
# from your build directory (e.g. ackermann-controller/build/)
./test/cpp-test
```

### Results and Performance Examples
**TODO: Add results and performance examples when visualization complete. Stub section included below.**
* Example of speed PID controller effectiveness (overshoot, oscillation, limits are enforced).
* Example of heading PID controller effectiveness (overshoot, oscillation, limits are enforced).
* Example of wheel speed outputs during straight driving and continuous speed turns in both directions (outer wheel speeds up, inner wheel slows down).

## Personnel
* Spencer Elyard, *TODO: Complete personnel blurb.*
* Santosh Kesani, *TODO: Complete personnel blurb.*
* Daniel Sahu, *TODO: Complete personnel blurb.*

## Licensing

This project uses the MIT License as described in [the license file](LICENSE).

## Engineering Processes and Documentation

Details on the status of our Agile Iterative Process (AIP) [can be found here](https://docs.google.com/spreadsheets/d/1nx85sowA3IRX-usU_M1hhwHplOLXMWdkvec2w3Roi5Q/edit?usp=sharing).

Sprint planning notes and reviews [can be found here](https://docs.google.com/document/d/1MEoRXtJXdUWnkTbJmcDfJYct3i6_LEJ-TULpP2h_qYA/edit?usp=sharing).

## Known Bugs and Issues
**TODO: Annote bugs and issues when uncovered.**

### Documentation Generation instructions
**TODO: Stub section - complete with instructions for Doxygen documentation generation.**

### Parameters and Limitations
**TODO: Add default values once established.**
#### Rover Model
The controller enforces the various limitations of the rover:

* Physical Parameters
  * Robot wheel base (length between front and rear wheels)
  * Robot wheel track (width between center of left and right wheels)
  * Robot maximum and minimum motor RPM
  * Maximum allowable steering angle (absolute value)
* Dynamic Limitations
  * Maximum (forward) and minimum (reverse) speed limitations
  * Maximum (forward) and minimum (reverse) acceleration limitations
  * Maximum (right) and minimum (left) angular velocity limitations
  * Maximum (right) and minimum (left) angular acecleration limitations
* Controller Parameters
  * Frequency
  * PID controller parameters for speed control
  * PID controller parameters for heading control
  
#### Controller Model
The controller will accept the following inputs:

* Desired heading (global coordinate frame) and speed
* Update to current speed and heading from rover model

The controller will provide the following outputs:

* Throttle and Steering Commands, limited as appropriate by the established limitations from the rover model
* Current calculated speed and heading
* Current desired heading (global coordinate frame) and speed
* Current drive wheel velocities

## Class Diagram

An overview of the classes used and their dependencies is shown in the following UML diagram:

![Class-Diagram](docs/uml/class_diagram.png)

## Activity Diagram

An example usage activity diagram is shown in the following UML diagram:

![Activity-Diagram](docs/uml/activity_diagram.png)
