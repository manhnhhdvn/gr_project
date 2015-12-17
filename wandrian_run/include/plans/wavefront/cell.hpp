/*
 * cell.hpp
 *
 *  Created on: Dec 17, 2015
 *      Author: haininh
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_WAVEFRONT_CELL_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_WAVEFRONT_CELL_HPP_

#include <stdlib.h>
#include <cmath>
#include <limits>

namespace wandrian {
namespace plans {
namespace wavefront {

struct Cell {
  double x, y;

  Cell();
  Cell(double, double);
};

inline bool operator==(const Cell &p1, const Cell &p2) {
  return (p1.x == p2.x) && (p1.y == p2.y);
}

inline bool operator!=(const Cell &p1, const Cell &p2) {
  return !(p1 == p2);
}

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_WAVEFRONT_CELL_HPP_ */
