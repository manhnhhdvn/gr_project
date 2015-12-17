/*
 * cell.cpp
 *
 *  Created on: Dec 17, 2015
 *      Author: haininh
 */

#include "../../../include/plans/wavefront/cell.hpp"

namespace wandrian {
namespace plans {
namespace wavefront {

Cell::Cell(PointPtr center, double size) :
    center(center), size(size), parent(CellPtr()) {
  points.insert(points.end(),
      PointPtr(new Point(center->x - size / 2, center->y + size / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + size / 2, center->y + size / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + size / 2, center->y - size / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x - size / 2, center->y - size / 2)));
  build();
}

PointPtr Cell::get_center() {
  return center;
}

double Cell::get_size() {
  return size;
}

CellPtr Cell::get_parent() {
  return parent;
}

void Cell::set_center(PointPtr center) {
  this->center = center;
}

void Cell::set_parent(CellPtr parent) {
  this->parent = parent;
}

}
}
}
