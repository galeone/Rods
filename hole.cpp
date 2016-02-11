#include "hole.h"

Hole::Hole(vector<Point>& contour) {
  _contour = contour;
}

Point Hole::getCenter() {
  return Point(0, 0);
}

float Hole::getDiameter() {
  return 0;
}
