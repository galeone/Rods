#include "hole.h"

Hole::Hole(vector<Point>& contour) {
  _contour = contour;
  Moments mu = moments(_contour, false);
  _center = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
  Point2i integerCenter(static_cast<int>(_center.x),
                        static_cast<int>(_center.y));
  double radius = 0;
  for (Point p : contour) {
    radius += norm(p - integerCenter);
  }
  radius /= _contour.size();
  _diameter = radius * 2;
}

Point2f Hole::getCenter() {
  return _center;
}

double Hole::getDiameter() {
  return _diameter;
}
