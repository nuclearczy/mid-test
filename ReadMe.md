# Xingyun (星云) Perception Module
[![Build Status](https://travis-ci.com/jingCGM/xingyun.svg?branch=master)](https://travis-ci.com/jingCGM/xingyun)
[![Coverage Status](https://coveralls.io/repos/github/jingCGM/xingyun/badge.svg)](https://coveralls.io/github/jingCGM/xingyun)
![GitHub](https://img.shields.io/github/license/jingCGM/xingyun)
---

## Overview

Xingyun is a software module which uses data from a 2D LIDAR to detect humans near a robot and give their relative locations in the robot’s reference frame. The purpose is to allow a robot to evoke appropriate behaviour in the vicinity of humans. 

The 2D LIDAR used in the module has the following parameters:

Resolution: 0.01m

Range: [0.1m, 4m]

FOV: 240 degrees

Updating frequency: 40 fps

Gaussian Noise: 0.0 mean, 0.02 standard variance

The module will only operate on one frame of data. The input of the module is a point cloud in the form of a vector with 512 elements. Each element represents a receptor of the LIDAR and contains the detected distance of a point. The output of the module is a new vector with the detected human locations and orientations in Cartesian coordinates in the robot’s reference frame, along with a visualization that shows the robot, human centroids and safety boundaries.

The architecture for the Xingyun module can be broken down into five main stages.

1) Data Input and Point Cloud Transform
The first part of the module parses the dataset file (in CSV format) and extracts its data into a 1D vector. A new 2D vector is then generated which represents the points in polar coordinates, before converting them to Cartesian.

2) Obstacle Classification and Extraction
Clusters of points close to the robot are interpreted as detected obstacle. These clusters are identified and isolated from the main point cloud by comparing the Euclidean distance between local points to a threshold. 

3) Leg Recognition
The list of obstacles is filtered, leaving only obstacles recognized as legs. Recognition involves evaluating contraints that deal with both the curvature and diameter of the obstacle contours and comparing them to thresholds.

4) Human Recognition
The list of legs is organized into pairs by evaluating which legs are in close proximity to each other. Each pair is recognized as a human, and the centroid of each human is calculated by averaging the centroids of the legs detected. Legs that are not organized into a pair are recognized as humans standing sideways with respect to the LIDAR beam. The centroids for these humans are approximated by extending the distance of the leg centroid by a fixed amount and then calculating the Cartesian coordinates. For each human, a “safety boundary" in the shape of an ellipse is applied to indicate possible occupied areas. The major and minor axes are fixed parameters and the orientation is calculated from the centroids of the legs.

5) Visualization
The final part of the module is an optionally run function that uses the MatPlotLib library to plot the robot, relative location of human centroids and safety boundaries on a 2D map in the robot’s reference frame.


## Personnel
Zuyang Cao: Student at the University of Maryland, Masters in Robotics

Hao Da (Kevin) Dong: Student at the University of Maryland, Masters in Systems Engineering

Jing Liang: Student at the University of Maryland, Masters in Robotics


## License
This project is licensed under the BSD 3-Clause. Please see LICENSE for additional details and disclaimer.

## Agile Iterative Process (AIP) Logs and Notes
AIP backlogs and work log:
https://drive.google.com/open?id=18jxJrOyLGHMjCCLtB2YXyqMBegSXfmTHaoVn-cLslTg

AIP Sprint Notes and Reviews:
https://drive.google.com/open?id=1H1pcWtISbMI9v3IDPGyBU7tNAPbOYgCZcXf3tWq5JKY


## Install Dependencies
<To be completed after Phase 2>


## Build Instructions
<To be updated after Phase 2>
```
git clone https://github.com/jingCGM/xingyun.git
cd <path to repository>
mkdir build
cd build
cmake ..
make
```


## Run Unit Tests
<To be updated after Phase 2>
```
./test/cpp-test
```


## Run Demonstration Program
<To be updated after Phase 2>
```
./app/shell-app
```


## Known Issues
<To be completed after Phase 2>
