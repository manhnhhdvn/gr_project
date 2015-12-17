/*
 * cell.cpp
 *
 *  Created on: Dec 17, 2015
 *      Author: haininh
 */

#include "../../../include/plans/wavefront/wavefront.hpp"

using namespace std;

namespace wandrian {
namespace plans {
namespace wavefront {

Wavefront::Wavefront() :
    robot_size(0) {
}

Wavefront::~Wavefront() {
}

// example worlds
// O -> (go right)
// |
// | (go down)
//World the First 16 * 8
//
//{ { 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, X, X, X, X },
//  { 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0 },
//  { 0, 0, 0, X, X, X, X, 0, 0, 0, 0, 0, X, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0 } }
//
//World the Second
//
//{ { 0, 0, 0, 0, 0, 0, 0, X, X, X, X, 0, 0, X, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0 },
//  { 0, 0, 0, X, X, X, X, 0, 0, X, 0, 0, 0, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0 } }

void Wavefront::initialize(CellPtr _start, CellPtr _goal, double robot_size) {
  this->robot_size = robot_size;
  int world[MAP_X][MAP_Y] = { { 0, X, X, 0, 0, 0, 0, 0 }, { 0, X, X, 0, 0, 0, 0,
      0 }, { 0, 0, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0,
      0, 0, 0, 0 }, { 0, 0, 0, 0, X, 0, 0, 0 }, { 0, 0, 0, X, X, X, 0, 0 }, { 0,
      0, 0, X, X, 0, 0, 0 } };

  for (int i = 0; i < MAP_X; i++) {
    std::vector<int> tmp;
    for (int j = 0; j < MAP_Y; j++) {
      tmp.push_back(world[i][j]);
    }
    WORLD_MAP.push_back(tmp);
  }

  start = _start;
  goal = _goal;
  WORLD_MAP[goal->get_center()->x][goal->get_center()->y] = GOAL;
}

void Wavefront::cover() {
  wave_fill();
  path_planning();
  cout << "1" << endl;

  CellPtr current = start;
  CellPtr next_cell;
  while (!path.empty()) {
    next_cell = path.front();
    path.pop();

    //    PointPtr new_position = PointPtr(
    //        new Point(next_cell->get_center()->x * robot_size, next_cell->get_center()->y * robot_size));
    //    go_to(new_position, STRICTLY);
    go_by_step(current, next_cell);
    current = next_cell;
  }
}

bool Wavefront::go_to(PointPtr position, bool flexibly) {
  std::cout << "    pos: " << position->x << "," << position->y << "\n";
  if (behavior_go_to != NULL)
    return behavior_go_to(position, flexibly);
  return true;
}

bool Wavefront::go_by_step(CellPtr current, CellPtr next) {
  if (current->get_center()->x == next->get_center()->x
      || current->get_center()->y == next->get_center()->y) { // Von Neuman neighbors
    PointPtr new_position = PointPtr(
        new Point(next->get_center()->x * robot_size,
            next->get_center()->y * robot_size));
    return go_to(new_position, STRICTLY);
  }

  CellPtr step;
  PointPtr new_position;
  step->get_center()->x = current->get_center()->x;
  step->get_center()->y = next->get_center()->y;
  new_position = PointPtr(
      new Point(step->get_center()->x * robot_size,
          step->get_center()->y * robot_size));
  go_to(new_position, STRICTLY);
  new_position = PointPtr(
      new Point(next->get_center()->x * robot_size,
          next->get_center()->y * robot_size));
  return go_to(new_position, STRICTLY);
}

/***** 1st Stage - Building world map *****/
void Wavefront::wave_fill() {
  queue<CellPtr> wave;
  int current_score;
  CellPtr next_cell, visiting_cell;

  next_cell = goal;
  cout << "4" << endl;
  wave.push(next_cell);
  while (!wave.empty()) {
    visiting_cell = wave.front();
    wave.pop();
    current_score = get_cell_value(visiting_cell->get_center()->x,
        visiting_cell->get_center()->y) + 1;

    // 8-directions version
    // 0 0 0
    // 0 1 0
    // 0 0 0

    // 1st row
    int x, y;

    x = visiting_cell->get_center()->x - 1;
    y = visiting_cell->get_center()->y - 1;
    next_cell->set_center(PointPtr(new Point(x, y)));
    if (check_coordinate(x, y) == true && get_cell_value(x, y) == UNVISITED) {
      WORLD_MAP[next_cell->get_center()->x][next_cell->get_center()->y] =
          current_score;
      wave.push(next_cell);
    }

    x = visiting_cell->get_center()->x;
    y = visiting_cell->get_center()->y - 1;
    next_cell->set_center(PointPtr(new Point(x, y)));
    if (check_coordinate(x, y) == true && get_cell_value(x, y) == UNVISITED) {
      WORLD_MAP[next_cell->get_center()->x][next_cell->get_center()->y] =
          current_score;
      wave.push(next_cell);
    }

    x = visiting_cell->get_center()->x + 1;
    y = visiting_cell->get_center()->y - 1;
    next_cell->set_center(PointPtr(new Point(x, y)));
    if (check_coordinate(x, y) == true && get_cell_value(x, y) == UNVISITED) {
      WORLD_MAP[next_cell->get_center()->x][next_cell->get_center()->y] =
          current_score;
      wave.push(next_cell);
    }

    // 2nd row
    x = visiting_cell->get_center()->x - 1;
    y = visiting_cell->get_center()->y;
    next_cell->set_center(PointPtr(new Point(x, y)));
    if (check_coordinate(x, y) == true && get_cell_value(x, y) == UNVISITED) {
      WORLD_MAP[next_cell->get_center()->x][next_cell->get_center()->y] =
          current_score;
      wave.push(next_cell);
    }

    x = visiting_cell->get_center()->x + 1;
    y = visiting_cell->get_center()->y;
    next_cell->set_center(PointPtr(new Point(x, y)));
    if (check_coordinate(x, y) == true && get_cell_value(x, y) == UNVISITED) {
      WORLD_MAP[next_cell->get_center()->x][next_cell->get_center()->y] =
          current_score;
      wave.push(next_cell);
    }

    // 3rd row
    x = visiting_cell->get_center()->x - 1;
    y = visiting_cell->get_center()->y + 1;
    next_cell->set_center(PointPtr(new Point(x, y)));
    if (check_coordinate(x, y) == true && get_cell_value(x, y) == UNVISITED) {
      WORLD_MAP[next_cell->get_center()->x][next_cell->get_center()->y] =
          current_score;
      wave.push(next_cell);
    }

    x = visiting_cell->get_center()->x;
    y = visiting_cell->get_center()->y + 1;
    next_cell->set_center(PointPtr(new Point(x, y)));
    if (check_coordinate(x, y) == true && get_cell_value(x, y) == UNVISITED) {
      WORLD_MAP[next_cell->get_center()->x][next_cell->get_center()->y] =
          current_score;
      wave.push(next_cell);
    }

    x = visiting_cell->get_center()->x + 1;
    y = visiting_cell->get_center()->y + 1;
    next_cell->set_center(PointPtr(new Point(x, y)));
    if (check_coordinate(x, y) == true && get_cell_value(x, y) == UNVISITED) {
      WORLD_MAP[next_cell->get_center()->x][next_cell->get_center()->y] =
          current_score;
      wave.push(next_cell);
    }
  }
  cout << "3" << endl;
}

bool Wavefront::check_coordinate(int x, int y) {
  return !((x < 0 || y < 0 || x > (MAP_X - 1) || y > (MAP_Y - 1)));
}

int Wavefront::get_cell_value(int x, int y) {
  return WORLD_MAP[x][y];
}

/***** 2nd Stage - Coverage path generate *****/
// tie-break rule: west first, clockwise: chon ben trai truoc, cung chieu kim dong ho
void Wavefront::path_planning() {
  stack<CellPtr> trajectory;
  CellPtr current = start;
  CellPtr next;
  do {
    next = get_next_hop(current);
    cout << next->get_center()->x << " " << next->get_center()->y << endl;
    if (next->get_center()->x == -1 && next->get_center()->y == -1) {
      // go back to previous
      next = trajectory.top();
      trajectory.pop();
    } else {
      trajectory.push(current);
    }
    cout << "6" << endl;

    path.push(next);
    WORLD_MAP[current->get_center()->x][current->get_center()->y] = VISITED;
    current = next;
  } while (current != goal);
}

CellPtr Wavefront::get_next_hop(CellPtr current) {
  cout << "5" << endl;
  CellPtr next, tmp;
  next = CellPtr(new Cell(PointPtr(new Point(-1, -1)), robot_size));
  int max_score = 1;

  int x, y;

  x = current->get_center()->x - 1;
  y = current->get_center()->y;
  if (check_coordinate(x, y) == true && get_cell_value(x, y) > max_score) {
    max_score = get_cell_value(x, y);
    next->set_center(PointPtr(new Point(x, y)));
  }

  x = current->get_center()->x - 1;
  y = current->get_center()->y - 1;
  if (check_coordinate(x, y) == true && get_cell_value(x, y) > max_score) {
    max_score = get_cell_value(x, y);
    next->set_center(PointPtr(new Point(x, y)));
  }

  x = current->get_center()->x;
  y = current->get_center()->y - 1;
  if (check_coordinate(x, y) == true && get_cell_value(x, y) > max_score) {
    max_score = get_cell_value(x, y);
    next->set_center(PointPtr(new Point(x, y)));
  }

  x = current->get_center()->x + 1;
  y = current->get_center()->y - 1;
  if (check_coordinate(x, y) == true && get_cell_value(x, y) > max_score) {
    max_score = get_cell_value(x, y);
    next->set_center(PointPtr(new Point(x, y)));
  }

  x = current->get_center()->x + 1;
  y = current->get_center()->y;
  if (check_coordinate(x, y) == true && get_cell_value(x, y) > max_score) {
    max_score = get_cell_value(x, y);
    next->set_center(PointPtr(new Point(x, y)));
  }

  x = current->get_center()->x + 1;
  y = current->get_center()->y + 1;
  if (check_coordinate(x, y) == true && get_cell_value(x, y) > max_score) {
    max_score = get_cell_value(x, y);
    next->set_center(PointPtr(new Point(x, y)));
  }

  x = current->get_center()->x;
  y = current->get_center()->y + 1;
  if (check_coordinate(x, y) == true && get_cell_value(x, y) > max_score) {
    max_score = get_cell_value(x, y);
    next->set_center(PointPtr(new Point(x, y)));
  }

  x = current->get_center()->x - 1;
  y = current->get_center()->y + 1;
  if (check_coordinate(x, y) == true && get_cell_value(x, y) > max_score) {
    max_score = get_cell_value(x, y);
    next->set_center(PointPtr(new Point(x, y)));
  }
  cout << "5.5" << endl;

  return next;
}

}
}
}
