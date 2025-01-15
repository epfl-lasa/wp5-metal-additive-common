#include <array>
#include <iostream>
#include <vector>

#include "ik_geo.h"

using namespace ik_geo;

int main(int argc, char const *argv[]) {
  Robot robot = Robot::ur5();
  double rotation_matrix[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  double position_vector[3] = {0.5, 0.5, 0.0};

  std::vector<ik_geo::Solution> solutions;
  robot.ik(rotation_matrix, position_vector, solutions);

  for (auto &solution : solutions) {

    std::cout << "Solution: ";
    for (std::size_t i = 0; i < 6; i++) {
      std::cout << solution.q[i] << " ";
    }

    std::array<double, 9> rotation_matrix;
    std::array<double, 3> position_vector;
    robot.fk(solution.q, rotation_matrix, position_vector);
    std::cout << "Is LS: " << (solution.is_ls ? "True" : "False") << std::endl;
    std::cout << "Rotation Matrix: " << std::endl;
    for (std::size_t i = 0; i < 3; i++) {
      for (std::size_t j = 0; j < 3; j++) std::cout << rotation_matrix[i * 3 + j] << " ";
      std::cout << std::endl;
    }
    std::cout << "Position Vector: " << std::endl;
    for (std::size_t i = 0; i < 3; i++) {
      std::cout << position_vector[i] << " ";
    }
    std::cout << std::endl;
  }
  return 0;
}
