# Xingyun (星云) Perception Module
[![Build Status](https://travis-ci.com/jingCGM/xingyun.svg?branch=master)](https://travis-ci.com/jingCGM/xingyun)
[![Coverage Status](https://coveralls.io/repos/github/jingCGM/xingyun/badge.svg)](https://coveralls.io/github/jingCGM/xingyun)
![GitHub](https://img.shields.io/github/license/jingCGM/xingyun)
---

![Output Sample](https://github.com/nuclearczy/mid-test/blob/master/dataset/combined_case_1.png)

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

Sample visualization results of some of the default demonstration scenarios can be found in dataset/demos/sample_results/. Please refer to the Run Demonstration Program section on how to interpret the pictures.


## Personnel
Zuyang Cao: Student at the University of Maryland, Masters in Robotics

Hao Da (Kevin) Dong: Student at the University of Maryland, Masters in Systems Engineering

Jing Liang: Student at the University of Maryland, Masters in Robotics


## License
This project is licensed under the BSD 3-Clause. Please see LICENSE for additional details and disclaimer. 


## Agile Iterative Process (AIP) Logs and Notes
AIP backlogs and work log:
https://drive.google.com/open?id=18jxJrOyLGHMjCCLtB2YXyqMBegSXfmTHaoVn-cLslTg

AIP sprint notes and reviews:
https://drive.google.com/open?id=1H1pcWtISbMI9v3IDPGyBU7tNAPbOYgCZcXf3tWq5JKY


## Install Dependencies
The Matplotlib and Boost libraries are required for the software to run. Matplotlib provides the visualizer and Boost provides methods that facilitate the manipulation of large vectors. Install both libraries with the following commands:
```
sudo apt-get install python-matplotlib python-numpy python2.7-dev
sudo apt-get install libboost-all-dev
```


## Build Instructions
To build the Xingyun module, apply the following commands in whichever local directory is desired to host the repository:
```
git clone https://github.com/jingCGM/xingyun.git
cd xingyun
mkdir build
cd build
cmake ..
make
```
This will clone the repository, create a build directory, run CMake and build the source code.

## Run Doxygen File
To install Doxygen
```
sudo apt install doxygen
```
To generate the Doxygen documentation
```
doxygen ./doxygen
````

## Run Unit Tests
To run our unit tests (optional), execute the following command within the build directory:
```
./test/cpp-test
```


## Run Demonstration Program
To run the default demonstration program, execute the following command within the build directory:
```
./app/shell-app
```
The demonstration code is in app/main.cpp and uses CSV dataset files that are located in dataset/demos/. To change the demo scenario, the following line in app/main.cpp must be changed:
```
std::string fileName = "../dataset/demos/Case1.csv";
```
Select any of the 7 default demo files, save, and rebuild the code. Pictures that correspond to each of the scenarios are also included in dataset/demos/ that depict the ground truth. The visualization produced will show the robot (facing right) as a blue square at the center of the plot, along with the centroids and safety boundaries of the detected humans in red.


## Notes and Known Issues
1) If a human is standing slanted enough such that the gap between its legs are not visible to the robot, the LIDAR will only see one "leg" and thus will identify a sideways human. In cases like this the orientation of the detected human will be less accurate than usual.

2) It was discovered through testing that the curvature contraint in legRecognition() that utilized gradient differences is not very robust. Thus, the more non-human obstacles are in the environment, the more difficulty the algorithm will have in correctly identifying legs from obstacles. An algorithm redesign will come with the next version of Xingyun.

3) If it is desired to use Xingyun as a standalone module as part of a larger project without any visualization, Matplotlib is not required. Xingyun object can be instantiated as normal and only humanPerception() is required to generate a vector of Human objects with centroid and orientation information.

4) Please note that include/matplotlibcpp.h is not developed by the authors, and thus does not contain a header indicating authorship and license terms. No changes were made to the file and thus it also does not pass cppcheck and cpplint format tests.


## Explanation of Commit Message Format Inconsistencies
Due to inexperience, commit messages before October 14, 2019, do not follow any consistent format and lack the AIP task numbers that link them to the backlogs and work log. Since it is not possible to modify these messages, the authors have instead listed the task numbers as a comment to these commits. 

Commit messages after October 14, 2019 follow a standardized format: [Files Changed][Task Number] "Message". The detailed messages often exceed the header limit set by GitHub, and thus the commits need to be clicked open to view the full messages.
