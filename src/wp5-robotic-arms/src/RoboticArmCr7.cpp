/**
 * @file RoboticArmCr7.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "RoboticArmCr7.h"

#include <ros/ros.h>

#include <iostream>

using namespace std;

RoboticArmCr7::RoboticArmCr7() : IRoboticArmBase(string("xMateCR7")) {}
