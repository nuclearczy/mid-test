/**Copyright (c) 2019 Hao Da (Kevin) Dong, Zuyang Cao, Jing Liang
 * @file       Xingyun.cpp
 * @date       10/13/2019
 * @brief      The file Xingyun.cpp implements a human perception class.
 *             The class will be used for detecting human from 2D Lidar data-sets.
 * @license    This project is released under the BSD-3-Clause License.
 */
#include <gtest/gtest.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <iterator>
#include <string>
#include <boost/range/combine.hpp>
#include <boost/tuple/tuple.hpp>

#include <Xingyun.hpp>
#include <Obstacle.hpp>
#include <Human.hpp>


int main() {
    Xingyun xingyun;
    std::string fileName = "../dataset/demos/Case6.csv";
    std::vector<Human> humanInfo = xingyun.humanPerception(fileName);
    std::cout << "Number of humans: " << humanInfo.size() << std::endl;
    xingyun.visualization();
    return 0;
}
