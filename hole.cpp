/*
Rods: visual inspection of motorcycle connecting rods improving the
result of the watershed algorithm exploiting the domain knowledge
Copyright (C) 2016 Paolo Galeone <nessuno@nerdz.eu>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
Exhibit B is not attached; this software is compatible with the
licenses expressed under Section 1.12 of the MPL v2.
*/

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
