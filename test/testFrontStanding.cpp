/**Copyright (c) 2019 Hao Da (Kevin) Dong, Zuyang Cao, Jing Liang
 * @file       Xingyun.hpp
 * @date       10/13/2019
 * @brief      This file is to test functions of the project in scenario where
 *             person stand in front of the lidar. The file test if lidar can
 *             recognize people correct
 * @license    This project is released under the BSD-3-Clause License.
 */
#include <gtest/gtest.h>

#include <vector>
#include <fstream>
#include <sstream>
#include <iterator>
#include <string>

#include <Xingyun.hpp>
#include <Obstacle.hpp>
#include <Human.hpp>
#include <boost/range/combine.hpp>
#include <boost/tuple/tuple.hpp>

#define TOLERANCE 1

/**
 * @brief test if the 1D vector is within tolerance comparing with ground truth
 * @param testingData, which is vector used to be tested
 * @param groundTruth of the 1D vector
 * @param tolerance, which is tolerance for each value
 * @return bool value showing if the vector is near ground truth
 */
bool test1DVectors(std::vector<double> testingData, std::vector<double> groundTruth, double tolerance) {
    bool condition = true;
    for (auto const& tupleValue : boost::combine(testingData, groundTruth)) {
        double x, y;
        boost::tie(x, y) = tupleValue;
        if (fabs(x-y) > tolerance) condition = false;
    }
    return condition;
}

/**
 * @brief test if the 2D vector is within tolerance comparing with ground truth
 * @param testingData, which is vector used to be tested
 * @param groundTruth of the 2D vector
 * @param tolerance, which is tolerance for each value
 * @return bool value showing if the vector is near ground truth
 */
bool test2DVectors(std::vector<std::vector<double>> testingData, std::vector<std::vector<double>> groundTruth, double tolerance) {
    bool condition = true;
    condition &= test1DVectors(testingData[0], groundTruth[0], tolerance);
    condition &= test1DVectors(testingData[1], groundTruth[1], tolerance);
    return condition;
}

/**
 * @brief function to compare two Obstacle objects
 * @param Obstacle object of ground truth
 * @param Obstacle object of testing data
 * @return judgement if obstacle is same as groundtruth
 */
bool testObstacle(Obstacle groundTruth, Obstacle testingData) {
    bool returnValue = true;
    double tolerance = TOLERANCE;

    // test if left point is right
    if (!test1DVectors(testingData.leftMostPoint, groundTruth.leftMostPoint, tolerance)) {
        returnValue = false;
                std::cout << "left most point is not right" << std::endl;
    }

    // test if right point is right
    if (!test1DVectors(testingData.rightMostPoint, groundTruth.rightMostPoint, tolerance)) {
        returnValue = false;
        std::cout << "right most point is not right" << std::endl;
    }

    // test if mid point is right
    if (!test1DVectors(testingData.midPoint, groundTruth.midPoint, tolerance)) {
        returnValue = false;
        std::cout << "middle point is not right" << std::endl;
    }

    // test if max gradience is right
    if (fabs(testingData.largestGrad-groundTruth.largestGrad) >= tolerance) {
        returnValue = false;
        std::cout << "largestGrad is not right" << testingData.largestGrad << std::endl;
    }

    // test if min gradience is right
    if (fabs(testingData.smallestGrad-groundTruth.smallestGrad) >= tolerance) {
        returnValue = false;
        std::cout << "smallestGrad is not right" << testingData.smallestGrad << std::endl;
    }
    return returnValue;
}


/**
 * @brief test value in x axis of detected human position
 * @param testPointCloudPolar
 * @param shouldPass
 * @return computes error
 */
TEST(testFrontHumanPositionX, shouldPass) {
  Xingyun xingyun;
  std::string fileName = "../dataset/unit_tests/test_double.csv";
  std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
  std::vector<double> midpoint = humanInfo[0].centroid;
  std::cout << "Human numbers: " << humanInfo.size() << std::endl;
  EXPECT_NEAR(midpoint[0], 2, TOLERANCE);
}

/**
 * @brief test value in y axis of detected human position
 * @param testPointCloudPolar
 * @param shouldPass
 * @return computes error
 */
TEST(testFrontHumanPositionY, shouldPass) {
    Xingyun xingyun;
    std::string fileName = "../dataset/unit_tests/test_double.csv";
    std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
    std::vector<double> midpoint = humanInfo[0].centroid;
    std::cout << "Human numbers: " << humanInfo.size() << std::endl;
    EXPECT_NEAR(midpoint[1], 0, TOLERANCE);
}


/**
 * @brief test value in yaw axis of detected human
 * @param testPointCloudPolar
 * @param shouldPass
 * @return computes error
 */
TEST(testFrontHumanPoseYaw, shouldPass) {
    Xingyun xingyun;
    std::string fileName = "../dataset/unit_tests/test_double.csv";
    std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
    double orientationAngle = humanInfo[0].orientationAngle;
    std::cout << "Human numbers: " << humanInfo.size() << std::endl;
    std::cout << "orientation angle: " << orientationAngle << std::endl;
    EXPECT_NEAR(orientationAngle, 3.14, TOLERANCE);
}


/**
 * @brief test if tested obstacle is right
 * @param testobstacles
 * @param shouldPass
 * @return computes error
 */
TEST(testobstacles, shouldPass) {
    Xingyun xingyun;
    std::string fileName = "../dataset/unit_tests/test_double.csv";
    std::vector<Human> humanInfo = xingyun.humanPerception(fileName);

    Obstacle groundTruth;
    groundTruth.largestGrad = 3.46;
    groundTruth.smallestGrad = -1.76;
    groundTruth.leftMostPoint.push_back(1.90);
    groundTruth.leftMostPoint.push_back(-0.17);
    groundTruth.rightMostPoint.push_back(1.92);
    groundTruth.rightMostPoint.push_back(-0.06);
    groundTruth.midPoint.push_back(1.91);
    groundTruth.midPoint.push_back(-0.12);
    std::cout << "Human numbers: " << humanInfo.size() << std::endl;
    std::vector<Obstacle> testingData = xingyun.getObstacleList();
    bool testCondition = testObstacle(groundTruth, testingData[0]);
    EXPECT_EQ(testCondition, true);
}


/**
 * @brief test if tested leg is right
 * @param testlegs
 * @param shouldPass
 * @return computes error
 */
TEST(testlegs, shouldPass) {
    Xingyun xingyun;
    std::string fileName = "../dataset/unit_tests/test_double.csv";
    std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
    Obstacle groundTruth;
    groundTruth.largestGrad = 3.46;
    groundTruth.smallestGrad = -1.76;
    groundTruth.leftMostPoint.push_back(1.90);
    groundTruth.leftMostPoint.push_back(-0.17);
    groundTruth.rightMostPoint.push_back(1.92);
    groundTruth.rightMostPoint.push_back(-0.06);
    groundTruth.midPoint.push_back(1.91);
    groundTruth.midPoint.push_back(-0.12);
    std::cout << "Human numbers: " << humanInfo.size() << std::endl;
    std::vector<Obstacle> testingData = xingyun.getLegList();
    bool testCondition = testObstacle(groundTruth, testingData[0]);
    EXPECT_EQ(testCondition, true);
}


/**
 * @brief test if polar data is right
 * @param testPolar
 * @param shouldPass
 * @return computes error
 */
TEST(testPolar, shouldPass) {
    std::ifstream file("../dataset/unit_tests/polar_1.csv");
    if (!file) {
        std::cout << "Error, file couldn't be opened" << std::endl;
    }
    std::string readRow, item;
    std::vector<std::vector<double>> readPolar;

    while (std::getline(file, readRow)) {
        std::vector<double> readVector;
        std::stringstream ss(readRow);
        while (std::getline(ss, item, ',')) {
            double value;
            std::stringstream readString;
            readString << item;
            readString >> value;
            readVector.push_back(value);
        }
        readPolar.push_back(readVector);
    }

    Xingyun xingyun;
    std::string fileName = "../dataset/unit_tests/test_double.csv";
    std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
    std::cout << "Human numbers: " << humanInfo.size() << std::endl;
    std::vector<std::vector<double>> testingData = xingyun.getPointCloudPolar();
    bool testCondition = test2DVectors(testingData, readPolar, TOLERANCE);
    EXPECT_EQ(testCondition, true);
}


/**
 * @brief test if cartesian data is right
 * @param testCartesian
 * @param shouldPass
 * @return computes error
 */
TEST(testCartisian, shouldPass) {
    std::ifstream file("../dataset/unit_tests/cartesian_1.csv");
    if (!file) {
        std::cout << "Error, file couldn't be opened" << std::endl;
    }
    std::string readRow, item;
    std::vector<std::vector<double>> readCartesian;
    while (std::getline(file, readRow)) {
        std::vector<double> readVector;
        std::stringstream ss(readRow);
        while (std::getline(ss, item, ',')) {
            double value;
            std::stringstream readString;
            readString << item;
            readString >> value;
            readVector.push_back(value);
        }
        readCartesian.push_back(readVector);
    }

    Xingyun xingyun;
    std::string fileName = "../dataset/unit_tests/test_double.csv";
    std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
    std::cout << "Human numbers: " << humanInfo.size() << std::endl;
    std::vector<std::vector<double>> testingData = xingyun.getPointCloudCartesian();
    bool testCondition = test2DVectors(testingData, readCartesian, TOLERANCE);
    EXPECT_EQ(testCondition, true);
}
