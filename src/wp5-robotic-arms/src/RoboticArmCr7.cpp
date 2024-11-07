/**
 * @file RoboticArmCr7.cpp
 * @author [Louis Munier] - lmunier@protonmail.com
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */

#include "RoboticArmCr7.h"

#include <ros/ros.h>

#include <iostream>

#include "IRosInterfaceBase.h"
#include "yaml_tools.h"

using namespace std;

RoboticArmCr7::RoboticArmCr7(ROSVersion rosVersion, string configFilename) :
    IRoboticArmBase(string("xMateCR7"),
                    rosVersion,
                    YAML::LoadFile(YamlTools::getYamlPath(configFilename, string(WP5_ROBOTIC_ARMS_DIR)))) {}
