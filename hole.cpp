#include "hole.h"

Hole::Hole(vector<Point>& contour) {
  _contour = contour;

  // Find the center as a ratio of moments
  Moments mu = moments(_contour, false);

  _center = Point2i(static_cast<int>(mu.m10 / mu.m00),
                    static_cast<int>(mu.m01 / mu.m00));
  // Calculate the radius as the average distance from
  // every contour point and the center
  double radius = 0;
  for (Point p : contour) {
    radius += norm(p - _center);
  }
  radius /= _contour.size();

  // Store the diameter
  _diameter = radius * 2;
}

Point2i Hole::getCenter() {
  return _center;
}

double Hole::getDiameter() {
  return _diameter;
}

vector<Point> Hole::getContour() {
  return _contour;
}
