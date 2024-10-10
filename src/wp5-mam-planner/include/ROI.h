/**
 * @file ROI.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

class ROI {
public:
  ROI() = default;

  bool empty() const { return id_.empty(); }

  void clear();

  void print() const;

  std::vector<double> getPoseVector(const std::string& posType) const;

  // Getters and setters for the private members
  const std::string& getID() const { return id_; }
  void setID(const std::string& id) { id_ = id; }

  const Eigen::Vector3d& getPosStart() const { return posStart_; }
  void setPosStart(const Eigen::Vector3d& posStart) { posStart_ = posStart; }

  const Eigen::Vector3d& getPosEnd() const { return posEnd_; }
  void setPosEnd(const Eigen::Vector3d& posEnd) { posEnd_ = posEnd; }

  const Eigen::Quaterniond& getQuat() const { return quat_; }
  void setQuat(const Eigen::Quaterniond& quat) { quat_ = quat; }

private:
  //TODO(lmunier(2024-10-10)): Implement vector<Eigen::Vector3d> to have more waypoints regarding cleaning task
  std::string id_{};
  Eigen::Vector3d posStart_{};
  Eigen::Vector3d posEnd_{};
  Eigen::Quaterniond quat_{};
};