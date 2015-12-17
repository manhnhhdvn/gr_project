/*
 * wavefront.hpp
 *
 *  Created on: Dec 17, 2015
 *      Author: haininh
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_WAVEFRONT_WAVEFRONT_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_WAVEFRONT_WAVEFRONT_HPP_

#include <iostream>       // std::cout
#include <stack>           // std::list
#include <queue>          // std::queue
#include "../../common/environment.hpp"
#include "../base_plan.hpp"
#include "cell.hpp"

#define VISITED   -1
#define UNVISITED 0
#define OBSTACLE  1
#define X     1
#define GOAL    2

// edit this first
#define MAP_X 8
#define MAP_Y 8

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace wavefront {

class Wavefront: public BasePlan {

public:
  Wavefront();
  ~Wavefront();
  void initialize(CellPtr, CellPtr, double);
  void cover();

protected:
  bool go_to(PointPtr, bool);
  bool go_by_step(CellPtr, CellPtr);

private:
  std::vector<std::vector<int> > WORLD_MAP;
  std::queue<CellPtr> path;
  CellPtr start;
  CellPtr goal;
  double robot_size;

  void wave_fill();
  bool check_coordinate(CellPtr);
  int get_cell_value(CellPtr);

  void path_planning();
  CellPtr get_next_hop(CellPtr);
};

typedef boost::shared_ptr<Wavefront> WavefrontPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_WAVEFRONT_WAVEFRONT_HPP_ */
