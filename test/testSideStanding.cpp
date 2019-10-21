/**Copyright (c) 2019 Hao Da (Kevin) Dong, Zuyang Cao, Jing Liang
 * @file       Xingyun.hpp
 * @date       10/13/2019
 * @brief      This file is to test functions of the project in the scenario
 *             where person stand a side the lidar. The file test if lidar can
 *             recognize people correct when lidar can only detect one leg.
 * @license    This project is released under the BSD-3-Clause License.
 */
#include <gtest/gtest.h>
#include <Xingyun.hpp>
#include <Obstacle.hpp>
#include <Human.hpp>

#include <vector>
#include <fstream>
#include <sstream>
#include <iterator>
#include <string>
#include <boost/range/combine.hpp>
#include <boost/tuple/tuple.hpp>

/**
 * @brief test if the 1D vector is within tolerance comparing with ground truth
 * @param testingData, which is vector used to be tested
 * @param groundTruth of the 1D vector
 * @param tolerance, which is tolerance for each value
 * @return bool value showing if the vector is near ground truth
 */
bool testSide1DVectors(std::vector<double> testingData, std::vector<double> groundTruth,double tolerance) {
	bool condition = true;
	for(auto const& tupleValue: boost::combine(testingData,groundTruth)) {
		double x, y;
		boost::tie(x,y) = tupleValue;
		if (abs(x-y)>tolerance) condition = false;
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
bool testSide2DVectors(std::vector<std::vector<double>> testingData, std::vector<std::vector<double>> groundTruth,double tolerance) {
	bool condition = true;
	condition &= testSide1DVectors(testingData[0], groundTruth[0],tolerance);
	condition &= testSide1DVectors(testingData[1], groundTruth[1],tolerance);
	return condition;
}

/**
 * @brief function to compare two Obstacle objects
 * @param Obstacle object of ground truth
 * @param Obstacle object of testing data
 * @return judgement if obstacle is same as groundtruth
 */
bool testSideObstacle(Obstacle groundTruth, Obstacle testingData) {
	bool returnValue = true;
	double tolerance = 0.01;

	// test if left point is right
	if(!testSide1DVectors(testingData.leftMostPoint,groundTruth.leftMostPoint,tolerance)) {
		returnValue = false;
				std::cout<<"left most point is not right"<<std::endl;
	}

	// test if right point is right
	if(!testSide1DVectors(testingData.rightMostPoint,groundTruth.rightMostPoint,tolerance)) {
		returnValue = false;
		std::cout<<"right most point is not right"<<std::endl;
	}

	// test if mid point is right
	if(!testSide1DVectors(testingData.midPoint,groundTruth.midPoint,tolerance)) {
		returnValue = false;
		std::cout<<"middle point is not right"<<std::endl;
	}

	// test if max gradience is right
	if (abs(testingData.largestGrad-groundTruth.largestGrad)>=tolerance) {
		returnValue = false;
		std::cout<<"largestGrad is not right"<<std::endl;
	}

	// test if min gradience is right
	if (abs(testingData.smallestGrad-groundTruth.smallestGrad)>=tolerance) {
		returnValue = false;
		std::cout<<"smallestGrad is not right"<<std::endl;
	}
	return returnValue;
}


/**
 * @brief test value in x axis of detected human position
 * @param testPointCloudPolar
 * @param shouldPass
 * @return computes error
 */
TEST(testSideHumanPositionX, shouldPass) {
  Xingyun xingyun;
  std::string fileName = "../dataset/unit_tests/test_single.csv";
  std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
  std::vector<double> midpoint = humanInfo[0].centroid;

  EXPECT_NEAR(midpoint[0], 2,0.2);
}

/**
 * @brief test value in y axis of detected human position
 * @param testPointCloudPolar
 * @param shouldPass
 * @return computes error
 */
TEST(testSideHumanPositionY, shouldPass) {
  Xingyun xingyun;
  std::string fileName = "../dataset/unit_tests/test_single.csv";
  std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
  std::vector<double> midpoint = humanInfo[0].centroid;

  EXPECT_NEAR(midpoint[1], 0,0.2);
}


/**
 * @brief test value in yaw axis of detected human
 * @param testPointCloudPolar
 * @param shouldPass
 * @return computes error
 */
TEST(testSideHumanPoseYaw, shouldPass) {
  Xingyun xingyun;
  std::string fileName = "../dataset/unit_tests/test_single.csv";
  std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
  double orientationAngle = humanInfo[0].orientationAngle;
std::cout<<"orientation: "<<orientationAngle<<std::endl;
  EXPECT_NEAR(orientationAngle, 0,0.2);
}





/**
 * @brief test if tested obstacle is right
 * @param testSideObstacles
 * @param shouldPass
 * @return computes error
 */
TEST(testSideObstacles, shouldPass) {
	Xingyun xingyun;
	std::string fileName = "../dataset/unit_tests/test_single.csv";
	std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
	Obstacle groundTruth;
	groundTruth.largestGrad = 2.07;
	groundTruth.smallestGrad = -244.87;
	groundTruth.leftMostPoint.push_back(1.90);
	groundTruth.leftMostPoint.push_back(-0.17);
	groundTruth.rightMostPoint.push_back(1.92);
	groundTruth.rightMostPoint.push_back(-0.06);
	groundTruth.midPoint.push_back(1.91);
	groundTruth.midPoint.push_back(-0.12);
	std::vector<Obstacle> testingData = xingyun.getObstacleList();
	bool testCondition = testSideObstacle(groundTruth, testingData[0]);
	EXPECT_EQ(testCondition,true);
}

/**
 * @brief test if tested leg is right
 * @param testSideLegs
 * @param shouldPass
 * @return computes error
 */
TEST(testSideLegs, shouldPass) {
	Xingyun xingyun;
	std::string fileName = "../dataset/unit_tests/test_single.csv";
	std::vector<Human> humanInfo = xingyun.humanPerception(fileName);

	Obstacle groundTruth;
	groundTruth.largestGrad = 2.07;
	groundTruth.smallestGrad = -244.87;
	groundTruth.leftMostPoint.push_back(1.90);
	groundTruth.leftMostPoint.push_back(-0.17);
	groundTruth.rightMostPoint.push_back(1.92);
	groundTruth.rightMostPoint.push_back(-0.06);
	groundTruth.midPoint.push_back(1.91);
	groundTruth.midPoint.push_back(-0.12);

	std::vector<Obstacle> testingData = xingyun.getLegList();
	bool testCondition = testSideObstacle(groundTruth, testingData[0]);
	EXPECT_EQ(testCondition,true);
}

/**
 * @brief test if polar data is right
 * @param testSidePolar
 * @param shouldPass
 * @return computes error
 */
TEST(testSidePolar, shouldPass) {

	std::ifstream file("../dataset/unit_tests/polar_0.csv");
	if (! file) {
		std::cout << "Error, file couldn't be opened" << std::endl;
	}
	std::string readRow, item;
	std::vector<std::vector<double>> readPolar;

	while(std::getline(file, readRow)) {
		std::vector<double> readVector;
		std::stringstream ss( readRow );
		while(std::getline(ss, item,',')) {
			double value;
			std::stringstream readString;
			readString << item;
			readString >> value;
			readVector.push_back(value);
		}
		readPolar.push_back(readVector);
	}

	Xingyun xingyunSidePolar;
	std::string fileName = "../dataset/unit_tests/test_single.csv";
	std::vector<Human> humanInfo = xingyunSidePolar.humanPerception(fileName);

	std::vector<std::vector<double>> testingData = xingyunSidePolar.getPointCloudPolar();
	bool testCondition = testSide2DVectors(testingData,readPolar,0.01);

	EXPECT_EQ(testCondition,true);
}


/**
 * @brief test if cartesian data is right
 * @param testSideCartisian
 * @param shouldPass
 * @return computes error
 */
TEST(testSideCartisian, shouldPass) {

	std::ifstream file("../dataset/unit_tests/cartesian_0.csv");
	if (! file) {
		std::cout << "Error, file couldn't be opened" << std::endl;
	}
	std::string readRow, item;
	std::vector<std::vector<double>> readCartesian;

	while(std::getline(file, readRow)) {
		std::vector<double> readVector;
		std::stringstream ss( readRow );
		while(std::getline(ss, item,',')) {
			double value;
			std::stringstream readString;
			readString << item;
			readString >> value;
			readVector.push_back(value);
		}
		readCartesian.push_back(readVector);
	}

	Xingyun xingyunSideCartesian;
	std::string fileName = "../dataset/unit_tests/test_single.csv";
	std::vector<Human> humanInfo = xingyunSideCartesian.humanPerception(fileName);

	std::vector<std::vector<double>> testingData = xingyunSideCartesian.getPointCloudCartesian();

	bool testCondition = testSide2DVectors(testingData,readCartesian,0.01);
	EXPECT_EQ(testCondition,true);
}
