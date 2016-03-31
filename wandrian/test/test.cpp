/*
 * test.cpp
 *
 *  Created on: Jul 31, 2015
 *      Author: sontd
 */

#include <GL/glut.h>
#include <GL/freeglut.h>
#include <limits>
#include <set>
#include <ctime>
#include <cstdlib>
#include <stdlib.h>
#include <boost/next_prior.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#include <fstream>
#include <stdlib.h>

#include "../include/environment/map.hpp"
#include "../include/plans/boustrophedon_online/boustrophedon_online.hpp"
#include "../include/plans/stc/full_scan_stc.hpp"

#define T_SIZE 0.5 // Tool size
#define B_SIZE 4.0 // Default space boundary size
#define WORLD_INSERT_OBSTACLE "<!-- INSERT: Boundary and Obstacles here -->" // Flag at original world file to insert boundary and obstacles into

using namespace wandrian::common;
using namespace wandrian::plans::stc;
using namespace wandrian::plans::boustrophedon_online;

double b_size = 0;
double t_size;

MapPtr map;
PointPtr starting_point;
std::list<PointPtr> path;

/**
 * Linked libraries to compile: -lglut -lGL (g++)
 */

void draw(std::list<PointPtr> points, int type) {
  glBegin(type);
  for (std::list<PointPtr>::iterator p = points.begin(); p != points.end();
      p++) {
    glVertex2d((*p)->x, (*p)->y);
  }
  glVertex2d((*points.begin())->x, (*points.begin())->y);
  glEnd();
}

void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-6, 6, -6, 6, -1, 1);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glScalef((b_size <= B_SIZE ? 5.0 : 10.0) / b_size,
      (b_size <= B_SIZE ? 5.0 : 10.0) / b_size, 0);

  // Center point
  glPointSize(4);
  glColor3ub(255, 255, 0);
  glBegin(GL_POINTS);
  glVertex2i(0, 0);
  glEnd();

  // Coordinate
  glPointSize(b_size <= 8 ? 2 : 1);
  glColor3ub(255, 255, 255);
  glBegin(GL_POINTS);
  RectanglePtr boundary = map->get_boundary();
  for (double i = boundary->get_center()->x - boundary->get_width() / 2.0;
      i <= boundary->get_center()->x + boundary->get_width() / 2.0;
      i += t_size * 2.0) {
    for (double j = boundary->get_center()->y - boundary->get_height() / 2.0;
        j <= boundary->get_center()->y + boundary->get_height() / 2.0;
        j += t_size * 2.0) {
      if (i != 0 || j != 0)
        glVertex2d(i, j);
    }
  }
  glEnd();

  // Space
  glColor3ub(255, 0, 0);
  draw(map->get_boundary()->get_boundary(), GL_LINE_STRIP);
  std::list<RectanglePtr> obstacles = map->get_obstacles();
  for (std::list<RectanglePtr>::iterator obstacle = obstacles.begin();
      obstacle != obstacles.end(); obstacle++) {
    draw((*obstacle)->get_boundary(), GL_POLYGON);
  }

  // Starting point
  glPointSize(b_size <= 8 ? 8 : 4);
  glColor3ub(0, 255, 0);
  glBegin(GL_POINTS);
  glVertex2d(starting_point->x, starting_point->y);
  glEnd();
  glRasterPos2i(0, -b_size / 2 - 1);
  std::stringstream ss;
  ss << starting_point->x << ", " << starting_point->y;
  for (int i = 0; i < ss.str().length(); i++)
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ss.str()[i]);

  // Covering path
  glColor3ub(0, 255, 0);
  draw(path, GL_LINE_STRIP);

  glutSwapBuffers();
}

void print_space() {
  RectanglePtr boundary = map->get_boundary();
  std::cout << boundary->get_center()->x << " " << boundary->get_center()->y
      << " " << boundary->get_width() << " " << boundary->get_height()
      << "\n\n";
  std::list<RectanglePtr> obstacles = map->get_obstacles();
  for (std::list<RectanglePtr>::iterator o = obstacles.begin();
      o != obstacles.end(); o++) {
    PointPtr c = (boost::static_pointer_cast<Cell>(*o))->get_center();
    std::cout << c->x << " " << c->y << "\n";
  }
  std::cout << "\n# " << starting_point->x << " " << starting_point->y << "\n";
}

int run(int argc, char **argv) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(600, 600);
  glutCreateWindow("Environment");
  glutDisplayFunc(display);
  glutCloseFunc(print_space);
  glutMainLoop();
  return 0;
}

bool test_go_to(PointPtr position, bool) {
  path.insert(path.end(), position);
  return true;
}

bool test_see_obstacle(VectorPtr direction, double distance) {
  // Simulator check obstacle
  PointPtr last_position = path.back();
  PointPtr new_position = last_position + direction * distance;
  if (map) {
    RectanglePtr boundary = map->get_boundary();
    if (new_position->x
        >= boundary->get_center()->x + boundary->get_width() / 2 - EPSILON
        || new_position->x
            <= boundary->get_center()->x - boundary->get_width() / 2 + EPSILON
        || new_position->y
            >= boundary->get_center()->y + boundary->get_height() / 2 - EPSILON
        || new_position->y
            <= boundary->get_center()->y - boundary->get_height() / 2
                + EPSILON) {
      return true;
    }
    std::list<RectanglePtr> obstacles = map->get_obstacles();
    for (std::list<RectanglePtr>::iterator o = obstacles.begin();
        o != obstacles.end(); o++) {
      CellPtr obstacle = boost::static_pointer_cast<Cell>(*o);
      if (new_position->x
          >= obstacle->get_center()->x - obstacle->get_size() / 2 - EPSILON
          && new_position->x
              <= obstacle->get_center()->x + obstacle->get_size() / 2 + EPSILON
          && new_position->y
              >= obstacle->get_center()->y - obstacle->get_size() / 2 - EPSILON
          && new_position->y
              <= obstacle->get_center()->y + obstacle->get_size() / 2
                  + EPSILON) {
        return true;
      }
    }
  }
  return false;
}

int main(int argc, char **argv) {
  // Boundary size and obstacle center
  RectanglePtr boundary;
  bool map_input = false;
  std::istringstream iss1(argv[1]);
  if (!(iss1 >> b_size)) { // Read from input map file
    map_input = true;
  } else {
    map_input = false;
    boundary = RectanglePtr(
        new Rectangle(PointPtr(new Point(0, 0)), b_size, b_size));
  }

  // Obstacle size
  double o_size;
  int number_of_obstacles;
  const double r = 0.2;
  bool obstacle_size_input = false;
  std::istringstream iss2(argv[2]);
  if (!(iss2 >> o_size)) {
    obstacle_size_input = true;
  } else {
    obstacle_size_input = false;
    double n = 0.75 * r * (b_size * b_size) / (o_size * o_size);
    number_of_obstacles = n
        + ((int) (0.25 * n) != 0 ? std::rand() % (int) (0.25 * n) : 0);
  }

  // Tool size
  std::istringstream iss3(argv[3]);
  if (!(iss3 >> t_size)) {
    t_size = T_SIZE;
  }

  // Read center point of obstacles from input file
  std::vector<PointPtr> obstacle_centers;
  std::vector<double> obstacle_sizes;
  bool global_obstacle_size = true;
  if (map_input) {
    std::ifstream map(("../" + std::string(argv[1])).c_str());
    std::string line;
    // Boundary size
    std::getline(map, line, '\n');
    int delim_pos1;
    int delim_pos2;
    int delim_pos3;
    delim_pos1 = line.find(" ");
    delim_pos2 = line.find(" ", delim_pos1 + 1);
    double center_x = boost::lexical_cast<double>(
        boost::lexical_cast<double>(line.substr(0, delim_pos1)));
    double center_y = boost::lexical_cast<double>(
        boost::lexical_cast<double>(
            line.substr(delim_pos1 + 1, delim_pos2 - delim_pos1 - 1)));
    delim_pos1 = delim_pos2;
    delim_pos2 = line.find(" ", delim_pos1 + 1);
    delim_pos3 = line.find(" ", delim_pos2 + 1);
    global_obstacle_size = !obstacle_size_input
        || (delim_pos3 != std::string::npos);
    double width = boost::lexical_cast<double>(
        boost::lexical_cast<double>(
            line.substr(delim_pos1 + 1, delim_pos2 - delim_pos1 - 1)));
    double height = boost::lexical_cast<double>(
        boost::lexical_cast<double>(
            line.substr(delim_pos2 + 1,
                (obstacle_size_input && global_obstacle_size ?
                    delim_pos3 : line.length()) - delim_pos2 - 1)));
    b_size = std::max(width, height);
    boundary = RectanglePtr(
        new Rectangle(PointPtr(new Point(center_x, center_y)), width, height));
    if (obstacle_size_input && global_obstacle_size)
      o_size = boost::lexical_cast<double>(
          boost::lexical_cast<double>(
              line.substr(delim_pos3 + 1, line.length() - delim_pos3 - 1)));
    // Center point of obstacles
    while (std::getline(map, line, '\n')) {
      if (line == "" || line.substr(0, 1) == "#")
        continue;
      double center_point_x;
      double center_point_y;
      int d_pos1 = line.find(" ");
      int d_pos2 = line.find(" ", d_pos1 + 1);
      center_point_x = boost::lexical_cast<double>(line.substr(0, d_pos1));
      center_point_y = boost::lexical_cast<double>(
          line.substr(d_pos1 + 1,
              (obstacle_size_input && !global_obstacle_size ?
                  d_pos2 : line.length()) - d_pos1 - 1));
      obstacle_centers.push_back(
          PointPtr(new Point(center_point_x, center_point_y)));
      if (obstacle_size_input && !global_obstacle_size) {
        obstacle_sizes.push_back(
            boost::lexical_cast<double>(
                line.substr(d_pos2 + 1, line.length() - d_pos2 - 1)));
      }
    }
  }

  // Generate obstacles
  std::list<RectanglePtr> obstacles;
  std::srand(std::time(0));
  for (int i = 0;
      i <= (map_input ? (int) obstacle_centers.size() - 1 : number_of_obstacles);
      i++) {
    PointPtr center =
        map_input ?
            obstacle_centers[i] :
            PointPtr(
                new Point(
                    ((std::rand()
                        % (int) (boundary->get_width() / t_size
                            / (o_size / t_size))
                        - (int) (boundary->get_width() / t_size
                            / (o_size / t_size) / 2.0)) + 0.5) * o_size,
                    ((std::rand()
                        % (int) (boundary->get_height() / t_size
                            / (o_size / t_size))
                        - (int) (boundary->get_height() / t_size
                            / (o_size / t_size) / 2.0)) + 0.5) * o_size));
    bool valid = true;
    for (std::list<RectanglePtr>::iterator o = obstacles.begin();
        o != obstacles.end(); o++)
      if ((boost::static_pointer_cast<Cell>(*o))->get_center() == center) {
        valid = false;
        break;
      };
    if (valid) {
      obstacles.insert(obstacles.end(),
          CellPtr(
              new Cell(center,
                  global_obstacle_size ? o_size : obstacle_sizes[i])));
    }
  }

  // Write output world file
  std::ifstream world_in("../../worlds/empty.world");
  std::ofstream world_out("../../worlds/tmp.world");
  std::string line;
  while (std::getline(world_in, line, '\n')) {
    world_out << line << '\n';
    if (line.find(WORLD_INSERT_OBSTACLE) != std::string::npos) {
      int n;
      n = 1;
      // Upper boundary
      for (double i = boundary->get_center()->x - boundary->get_width() / 2
          + t_size / 2;
          i
              <= boundary->get_center()->x + boundary->get_width() / 2
                  - t_size / 2 + EPSILON; i += t_size) {
        world_out << "    <model name='cinder_block_boundary_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << i << " "
            << (boundary->get_center()->y + boundary->get_height() / 2
                + t_size / 4) << " 0 0 0 0</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }
      // Right boundary
      for (double i = boundary->get_center()->y - boundary->get_height() / 2
          + t_size / 2;
          i
              <= boundary->get_center()->y + boundary->get_height() / 2
                  - t_size / 2 + EPSILON; i += t_size) {
        world_out << "    <model name='cinder_block_boundary_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>"
            << (boundary->get_center()->x + boundary->get_width() / 2
                + t_size / 4) << " " << i << " 0 0 0 " << M_PI_2 << "</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }
      // Lower boundary
      for (double i = boundary->get_center()->x - boundary->get_width() / 2
          + t_size / 2;
          i
              <= boundary->get_center()->x + boundary->get_width() / 2
                  - t_size / 2 + EPSILON; i += t_size) {
        world_out << "    <model name='cinder_block_boundary_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << i << " "
            << (boundary->get_center()->y - boundary->get_height() / 2
                - t_size / 4) << " 0 0 0 0</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }
      // Left boundary
      for (double i = boundary->get_center()->y - boundary->get_height() / 2
          + t_size / 2;
          i
              <= boundary->get_center()->y + boundary->get_height() / 2
                  - t_size / 2 + EPSILON; i += t_size) {
        world_out << "    <model name='cinder_block_boundary_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>"
            << (boundary->get_center()->x - boundary->get_width() / 2
                - t_size / 4) << " " << i << " 0 0 0 " << M_PI_2 << "</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }
      n = 1;
      // Obstacles
      for (std::list<RectanglePtr>::iterator o = obstacles.begin();
          o != obstacles.end(); o++) {
        CellPtr cell = boost::static_pointer_cast<Cell>(*o);
        PointPtr c = cell->get_center();
        double s = cell->get_size();
        double x = c->x - t_size * (s / t_size / 2.0 - 1.0 / 2.0);
        int j = 1;
        for (int i = 1; i <= (int) (s / t_size); i++) {
          for (double y = c->y - t_size * (s / t_size / 2.0 - 1.0 / 4.0);
              y <= c->y + t_size * (s / t_size / 2.0 - 1.0 / 4.0) + EPSILON;
              y += t_size / 2.0) {
            world_out << "    <model name='cinder_block_obstacle_" << n << "_"
                << j << "'>\n";
            world_out << "      <include>\n";
            world_out << "        <uri>model://cinder_block</uri>\n";
            world_out << "      </include>\n";
            world_out << "      <pose>" << x << " " << y << " 0 0 0 0</pose>\n";
            world_out << "      <static>1</static>\n";
            world_out << "    </model>\n";
            j++;
          }
          x += t_size;
        }
        n++;
      }
    }
  }
  world_in.close();
  world_out.close();

  // Starting point
  std::istringstream iss4(argv[4]);
  double starting_point_x;
  double starting_point_y;
  std::string plan_name;
  if (iss4 >> starting_point_x) {
    std::istringstream iss5(argv[5]);
    iss5 >> starting_point_y;
    starting_point = PointPtr(new Point(starting_point_x, starting_point_y));
    plan_name = std::string(argv[6]);
  } else {
    starting_point = PointPtr(
        new Point(
            (std::rand() % (int) (boundary->get_width() / t_size / 2.0)
                - (int) (boundary->get_width() / t_size / 4.0)) * t_size / 0.5
                + t_size + t_size / 2.0,
            (std::rand() % (int) (boundary->get_height() / t_size / 2.0)
                - (int) (boundary->get_height() / t_size / 4.0)) * t_size / 0.5
                + t_size - t_size / 2.0));
    plan_name = iss4.str();
  }

  map = MapPtr(new Map(boundary, obstacles));
  print_space();
  if (plan_name == "spiral_stc") {
    SpiralStcPtr plan_spiral_stc = SpiralStcPtr(new SpiralStc());
    plan_spiral_stc->initialize(starting_point, t_size);
    path.insert(path.end(), starting_point);
    plan_spiral_stc->set_behavior_go_to(boost::bind(&test_go_to, _1, _2));
    plan_spiral_stc->set_behavior_see_obstacle(
        boost::bind(&test_see_obstacle, _1, _2));
    plan_spiral_stc->cover();
  } else if (plan_name == "full_spiral_stc") {
    FullSpiralStcPtr plan_full_spiral_stc = FullSpiralStcPtr(
        new FullSpiralStc());
    plan_full_spiral_stc->initialize(starting_point, t_size);
    path.insert(path.end(), starting_point);
    plan_full_spiral_stc->set_behavior_go_to(boost::bind(&test_go_to, _1, _2));
    plan_full_spiral_stc->set_behavior_see_obstacle(
        boost::bind(&test_see_obstacle, _1, _2));
    plan_full_spiral_stc->cover();
  } else if (plan_name == "full_scan_stc") {
    FullScanStcPtr plan_full_scan_stc = FullScanStcPtr(new FullScanStc());
    plan_full_scan_stc->initialize(starting_point, t_size);
    path.insert(path.end(), starting_point);
    plan_full_scan_stc->set_behavior_go_to(boost::bind(&test_go_to, _1, _2));
    plan_full_scan_stc->set_behavior_see_obstacle(
        boost::bind(&test_see_obstacle, _1, _2));
    plan_full_scan_stc->cover();
  } else if (plan_name == "boustrophedon_online") {
    BoustrophedonOnlinePtr plan_boustrophedon_online = BoustrophedonOnlinePtr(
        new BoustrophedonOnline());
    plan_boustrophedon_online->initialize(starting_point, t_size);
    path.insert(path.end(), starting_point);
    plan_boustrophedon_online->set_behavior_go_to(
        boost::bind(&test_go_to, _1, _2));
    plan_boustrophedon_online->set_behavior_see_obstacle(
        boost::bind(&test_see_obstacle, _1, _2));
    plan_boustrophedon_online->cover();
  }

  run(argc, argv);
  return 0;
}
