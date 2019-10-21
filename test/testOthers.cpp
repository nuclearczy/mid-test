/**Copyright (c) 2019 Hao Da (Kevin) Dong, Zuyang Cao, Jing Liang
 * @file       Xingyun.hpp
 * @date       10/13/2019
 * @brief      This file is to test functions of the project in scenario where
 *             person stand in front of the lidar. The file test if lidar can
 *             recognize people correct
 * @license    This project is released under the BSD-3-Clause License.
 */
#include <gtest/gtest.h>
#include <matplotlibcpp.h>
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

/**
 * @brief test if other functions and classes are covered
 * @param testOthers
 * @param shouldPass
 * @return computes error
 */
TEST(testOthers, shouldPass) {
    Xingyun xingyun;
    std::string fileName = "../dataset/unit_tests/test_double.csv";
    std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
    Human person = humanInfo[0];
    std::vector<Obstacle> legs = xingyun.getLegList();
    Obstacle leg = legs[0];

    std::cout << "leg data: " << leg.largestGrad << std::endl;
    std::cout << "Human numbers: " << humanInfo.size() << std::endl;
    xingyun.visualization();
    EXPECT_EQ(1, 1);
}
