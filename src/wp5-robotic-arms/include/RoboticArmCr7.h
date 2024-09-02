/**
 * @file RoboticArmCr7.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include "IRoboticArmBase.h"

/**
 * @brief Child class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class RoboticArmCr7 : public IRoboticArmBase {
public:
  // TODO: implement all the public members, accessible from everyone owning a class object
  explicit RoboticArmCr7();

protected:
  // TODO: implement all the protected members, accessible from its own and herited classes

private:
  // TODO: implement all the private members, only accessible from its own class
};
