#include "hole.h"

Hole::Hole(vector<Point>& contour) {
  _contour = contour;

  // Find the center as a ratio of moments
  Moments mu = moments(_contour, false);
  _center = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
  Point2i integerCenter(static_cast<int>(_center.x),
                        static_cast<int>(_center.y));

  // Calculate the radius as the average distance from every contour point and
  // the center
  double radius = 0;
  for (Point p : contour) {
    radius += norm(p - integerCenter);
  }
  radius /= _contour.size();

  // Store the diameter
  _diameter = radius * 2;
}

Point2f Hole::getCenter() {
  return _center;
}

double Hole::getDiameter() {
  return _diameter;
}
