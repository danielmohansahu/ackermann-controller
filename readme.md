# Ackermann Controller
[![Build Status](https://travis-ci.org/danielmohansahu/ackermann-controller.svg?branch=master)](https://travis-ci.org/danielmohansahu/ackermann-controller)
[![Coverage Status](https://coveralls.io/repos/github/danielmohansahu/ackermann-controller/badge.svg?branch=master)](https://coveralls.io/github/danielmohansahu/ackermann-controller?branch=master)
---

## Overview

A basic implementation of an [Ackermann Steering Geometry](https://en.wikipedia.org/wiki/Ackermann_steering_geometry) controller.

## Engineering Processes and Documentation

Details on the status of our Agile Iterative Process (AIP) [can be found here](https://docs.google.com/spreadsheets/d/1nx85sowA3IRX-usU_M1hhwHplOLXMWdkvec2w3Roi5Q/edit?usp=sharing).

## Standard install via command-line
```
git clone https://github.com/danielmohansahu/ackermann-controller
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```
