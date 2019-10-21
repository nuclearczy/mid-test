/**Copyright (c) 2019 Hao Da (Kevin) Dong, Zuyang Cao, Jing Liang
 * @file       Human.hpp
 * @date       10/13/2019
 * @brief      The file Human.hpp contains the header declarations for a
 *             Human class. The class will be used in Xingyun class.
 * @license    This project is released under the BSD-3-Clause License.
 */
#ifndef INCLUDE_HUMAN_HPP_
#define INCLUDE_HUMAN_HPP_

#include<iostream>
#include<vector>

class Human{
 public:
  /** @brief The centroid of a possible human obstacle. */
  std::vector<double> centroid;

  /** @brief The orientation angle (in radians) of a possible human obstacle. */
  double orientationAngle;
};


#endif  // INCLUDE_HUMAN_HPP_
