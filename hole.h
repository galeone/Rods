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

#ifndef HOLE_H
#define HOLE_H

#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

class Hole {
 public:
  Hole(vector<Point>& contour);
  Point2i getCenter();
  double getDiameter();
  vector<Point> getContour();

 private:
  vector<Point> _contour;
  Point2i _center;
  double _diameter;
};

#endif  // HOLE_H
