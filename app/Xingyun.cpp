/**Copyright (c) 2019 Hao Da (Kevin) Dong, Zuyang Cao, Jing Liang
 * @file       Xingyun.cpp
 * @date       10/13/2019
 * @brief      The file Xingyun.cpp implements a human perception class.
 *             The class will be used for detecting human from 2D Lidar data-sets.
 * @license    This project is released under the BSD-3-Clause License.
 */
#include <matplotlibcpp.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <iterator>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>
#include <boost/range/combine.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/range/irange.hpp>

#include <Obstacle.hpp>
#include <Human.hpp>
#include <Xingyun.hpp>


namespace plt = matplotlibcpp;

#define LIDAR_RANGE 3.9
#define CLUSTER_THRESHOLD 0.3
#define LEG_DIAMETER 0.4
#define LEG_DISTANCE 0.3
#define MAJOR_AXIS 0.5
#define MINOR_AXIS 0.2
#define GRAD_DIFF_THRESHOLD 3

/** @brief Read data and classify the points into obstacles. */
void Xingyun::obstacleClassification() {
    double preDistance = rawLidarDistances[0];
    std::vector<double> xList, yList, distanceList;
    std::vector<std::vector<double>> objectBufferX, objectBufferY;
    std::vector<std::vector<std::vector<double>>> objects;

    // point cloud classification
    for (auto const& tupleValue : boost::combine(rawLidarDistances, pointCloudCartesian[0], pointCloudCartesian[1])) {
        double distance, x, y;
        boost::tie(distance, x, y) = tupleValue;

        if (fabs(distance - preDistance) >= CLUSTER_THRESHOLD) {
            if (distanceList.size() > 0) {
                if (distanceList[0] < LIDAR_RANGE) {
                    objectBufferX.push_back(xList);
                    objectBufferY.push_back(yList);
                }
                xList.clear();
                yList.clear();
                distanceList.clear();
            }
        }
        preDistance = distance;
        xList.push_back(x);
        yList.push_back(y);
        distanceList.push_back(distance);
    }
    if (distanceList.size() > 0) {
        if (distanceList[0] < LIDAR_RANGE) {
            objectBufferX.push_back(xList);
            objectBufferY.push_back(yList);
        }
    }

    // extract information needed for obstacle object
    for (auto const& tupleValue : boost::combine(objectBufferX, objectBufferY)) {
        std::vector<double> xTempList, yTempList, cartesianVector;
        Obstacle obstacleValue;

        boost::tie(xTempList, yTempList) = tupleValue;
        obstacleValue.rightMostPoint = { *(xTempList.end() - 1) , *(yTempList.end() - 1)};
        obstacleValue.leftMostPoint =  { *xTempList.begin() , *yTempList.begin() };
        obstacleValue.midPoint = {(*xTempList.begin() + *(xTempList.end() - 1))/2,
            (*(yTempList.end() - 1) + *yTempList.begin())/2 };

        std::vector<double> gradiences;
        auto const tupleCombination = boost::combine(xTempList, yTempList);
        for (auto beginIndex = tupleCombination.begin(); beginIndex < tupleCombination.end() - 1; beginIndex++) {
            double x, y, nextX, nextY;
            boost::tie(x, y) = *beginIndex;
            boost::tie(nextX, nextY) = *(beginIndex+1);

            double gradience;
            if ((nextX-x) == 0)
                gradience = 9999;
            else
                gradience = (nextY - y) / (nextX - x);
            if (gradience > 9999) gradience = 9999;
            gradiences.push_back(gradience);
        }

        obstacleValue.largestGrad = *std::max_element(gradiences.begin(), gradiences.end());
        obstacleValue.smallestGrad = *std::min_element(gradiences.begin(), gradiences.end());

        obstacleList.push_back(obstacleValue);
    }

    std::cout << "Number of obstacles: " << obstacleList.size() << std::endl;

    return;
}


/** @brief Recognize legs among obstacles. */
void Xingyun::legRecognition() {
    double obstacleLength = 0;   // Length of obstacle contour

    // Check each obstacle in obstacleList
    for (auto const obstacle : obstacleList) {
        // Check gradient constraint
        if (obstacle.largestGrad - obstacle.smallestGrad > GRAD_DIFF_THRESHOLD) {
            // Calculate length of obstacle contour
            obstacleLength = sqrt(pow(obstacle.rightMostPoint[0] - obstacle.leftMostPoint[0], 2)
                                    + pow(obstacle.rightMostPoint[1] - obstacle.leftMostPoint[1], 2));
            // Check diameter constraint. Add obstacle to legList if it passes both constraints.
            if (obstacleLength < LEG_DIAMETER)
                legList.push_back(obstacle);
        }
    }

    std::cout << "Number of legs: " << legList.size() << std::endl;

    return;
}


/** @brief Supporting function to humanRecognition() to process humans where both legs are visible
 *  @param queue 2-element inspection queue for leg pairs
*/
void Xingyun::processNormalHuman(std::vector<Obstacle> queue) {
    // Calculate human centroid from midpoints of legs
    std::vector<double> centroid;
    centroid.push_back((queue[0].midPoint[0] + queue[1].midPoint[0]) / 2);
    centroid.push_back((queue[0].midPoint[1] + queue[1].midPoint[1]) / 2);

    // Calculate orientation angle
    double orientationAngle = atan((queue[0].midPoint[1] - queue[1].midPoint[1]) / (queue[0].midPoint[0] - queue[1].midPoint[0]));

    // Create Human object and add it to humanList
    Human human;
    human.centroid = centroid;
    human.orientationAngle = orientationAngle + M_PI/2;
    humanList.push_back(human);
    return;
}


/** @brief Supporting function to humanRecognition() to process humans where only one leg is visible
 *  @param queue 2-element inspection queue for leg pairs
*/
void Xingyun::processSidewaysHuman(std::vector<Obstacle> queue) {
    // Calculate orientation angle directly from leg coordinates
    double orientation = atan(queue[0].midPoint[1] / queue[0].midPoint[0]);

    // Approximate human centroid from midpoints of legs
    std::vector<double> centroid;
    centroid.push_back(queue[0].midPoint[0] + (LEG_DISTANCE/2)*cos(orientation));
    centroid.push_back(queue[0].midPoint[1] + (LEG_DISTANCE/2)*sin(orientation));

    // Create Human object and add it to humanList
    Human human;
    human.centroid = centroid;
    human.orientationAngle = orientation + M_PI/2;
    humanList.push_back(human);
    return;
}


/** @brief Recognize humans from legs. */
void Xingyun::humanRecognition() {
    std::vector<Obstacle> queue, legListTemp;    // 2-element inspection queue for leg pairs
    legListTemp = legList;
    while (legListTemp.empty() == false) {
        // Pop first element of legList into the inspeaction queue
        queue.push_back(legListTemp.front());

        legListTemp.erase(legListTemp.begin());
        // Load up queue to two elements, unless legList is empty, in which case process the 1-element queue as a sideways human
        if (queue.size() < 2) {
            if (legListTemp.empty() == false) {
                continue;
            } else {
                processSidewaysHuman(queue);
                queue.pop_back();
                continue;
            }
        }

        // Calculate distance between legs in queue
        double distance = sqrt(pow(queue[0].midPoint[0] - queue[1].midPoint[0], 2)
                            + pow(queue[0].midPoint[1] - queue[1].midPoint[1], 2));

        // If the legs are close together, process the pair as a normal human then empty the queue
        if (distance < LEG_DISTANCE) {
            processNormalHuman(queue);
            queue.pop_back();
            queue.pop_back();
        } else {  // If legs are far apart, process the first leg as a sideways human and pop it out of the queue
            // Create temporary queue to hold the first leg
            std::vector<Obstacle> temp;
            temp.push_back(queue.front());
            queue.erase(queue.begin());
            processSidewaysHuman(temp);
        }
    }

    if (queue.empty() == false) {
        std::vector<Obstacle> temp;
        temp.push_back(queue.front());
        queue.erase(queue.begin());
        processSidewaysHuman(temp);
    }
}


/**
 * @brief Main function to detect human.
 * @param lidarDatasetFilename - Name of the data-set.
 * @return humanList - The detected human list.
 */
std::vector<Human> Xingyun::humanPerception(std::string lidarDatasetFilename) {
    // load lidar data from csv file to vector rawLidarDistances
    std::ifstream fileStream(lidarDatasetFilename);
    std::string item;
    while (std::getline(fileStream, item, ',')) {
        double value;
        std::stringstream readString;
        readString << item;
        readString >> value;
        rawLidarDistances.push_back(value);
    }
    if (rawLidarDistances.size() != 512) {
        std::cout << "didn't read file right: " << rawLidarDistances.size() << std::endl;
    }
    // get polar data from raw lidar data
    std::vector<double> angles;  // vector with 100 ints.
    for (int i : boost::irange(0, 512)) {angles.push_back((-120 + i * 240 / 512) * M_PI / 180);}

    pointCloudPolar.push_back(rawLidarDistances);
    pointCloudPolar.push_back(angles);

    // converte polar coordinates to cartesian coordinates
    std::vector<double> xValues, yValues;

    for (auto const& tupleValue : boost::combine(rawLidarDistances, angles)) {
        double distance, angle;
        boost::tie(distance, angle) = tupleValue;
        xValues.push_back(distance * cos(angle));
        yValues.push_back(distance * sin(angle));
    }
    pointCloudCartesian.push_back(xValues);
    pointCloudCartesian.push_back(yValues);

    obstacleClassification();
    legRecognition();
    humanRecognition();

    return humanList;
}


/** @brief Show the output map. */
void Xingyun::visualization() {
    plt::plot({ 0 }, { 0 }, "bs");  // Show robot as square at origin.
    for (auto human : humanList) {
           plt::plot({ human.centroid[0] }, { human.centroid[1] }, "ro");  // Plot human centroid in map.
           double orientation = human.orientationAngle;  // Human orientation radians, x-axis (pointing to the right) is 0 radians
           int n = 500;
           std::vector<double> x(n), y(n);
           for (int i = 0; i < n; ++i) {
               double t = 2 * M_PI * i / n;
               x.at(i) = MINOR_AXIS * cos(t) * cos(orientation)
                       - MAJOR_AXIS * sin(t) * sin(orientation) + human.centroid[0];
               y.at(i) = MINOR_AXIS * cos(t) * sin(orientation)
                       + MAJOR_AXIS * sin(t) * cos(orientation) + human.centroid[1];
           }
           plt::plot(x, y, "r-");  // Plot human as ellipse.
    }
    plt::xlim(-4, 4);
    plt::ylim(-4, 4);
    plt::show();
    return;
}


/** @brief Get rawLidarDistances.
 *  @return rawLidarDistances - Raw data.
 */
std::vector<double> Xingyun::getRawLidarDistances() {
    return rawLidarDistances;
}


/** @brief Get pointCloudPolar.
 *  @return pointCloudPolar - Converted data in Polar coordinate.
 */
std::vector<std::vector<double>> Xingyun::getPointCloudPolar() {
    return pointCloudPolar;
}


/** @brief Get pointCloudCartesian.
 *  @return pointCloudCartesian - Converted data in Cartesian coordinate.
 */
std::vector<std::vector<double>> Xingyun::getPointCloudCartesian() {
    return pointCloudCartesian;
}


/** @brief Get obstacleList.
 *  @return obstacleList - List of obstacles.
 */
std::vector<Obstacle> Xingyun::getObstacleList() {
    return obstacleList;
}


/** @brief Get legList.
 *  @return legList - List of legs.
 */
std::vector<Obstacle> Xingyun::getLegList() {
    return legList;
}


/** @brief Get humanList.
 *  @return humanList - List of humans.
 */
std::vector<Human> Xingyun::getHumanList() {
    return humanList;
}

