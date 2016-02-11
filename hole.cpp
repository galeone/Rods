#include "hole.h"

Hole::Hole(vector<Point>& contour) {
  _contour = contour;
  Moments mu = moments(_contour, false);
  _center = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
  Point2i integerCenter((int)_center.x, (int)_center.y);
  _diameter = 0;
  for (Point p : contour) {
    _diameter += norm(p - integerCenter);
  }
  _diameter /= _contour.size();
  _diameter *= 2;
}

Point2f Hole::getCenter() {
  return _center;
}

double Hole::getDiameter() {
  return _diameter;
}
