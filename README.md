# Occupancy Grid Mapping with Python and Arduino

This project implements an occupancy grid mapping algorithm using a robot with ultrasonic sensing and Arduino/Python control.

## Overview

The goal is to build a map of the environment by taking ultrasonic sensor measurements and estimating the occupancy probability of each cell in the grid.

The robot can be controlled manually via keyboard or autonomously navigate between two points.

### Hardware

The robot uses the following hardware:

- Arduino Uno
- Raspberry Pi 
- Ultrasonic distance sensor
- Chassis with two motors

### Software

The system uses Python on the Raspberry Pi to run the mapping algorithm. The Arduino runs a control loop to operate the motors and read the ultrasonic sensor.

Key Python modules used:

- `numpy` for numerical computations
- `matplotlib` for displaying maps
- `pyserial` for serial communication

The Arduino code handles motor control and sensor reading.

### Algorithms

- Bayes filter to update occupancy probabilities based on sensor measurements 
- Dijkstra's algorithm to find the shortest path for autonomous navigation
- My own implementation to follow the shortest path autonomously

## Usage

- Run the Arduino code on the Uno
- Run `python code.py` to start the Python program 
- Control the robot and take sensor measurements via keyboard or set start and end points for autonomous navigation 
- View the final occupancy grid map

The code is documented to explain the approach.
