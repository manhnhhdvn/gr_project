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

Cell::Cell() :
    x(), y() {
}

Cell::Cell(double x, double y) :
    x(x), y(y) {
}

}
}
}
