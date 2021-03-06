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

#include <iostream>
#include <vector>
#include <cstdlib>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "rod.h"

using namespace std;
using namespace cv;

Mat1b read(const char* path) {
  const String name(path);
  return imread(name, IMREAD_GRAYSCALE);
}

void show(const Mat& image, string winName) {
  namedWindow(winName);
  imshow(winName, image);
}

void show(const Mat& image) {
  static int counter = 1;
  string winName = string("window " + to_string(counter));
  show(image, winName);
  ++counter;
}

double binarize(const Mat1b& input, Mat1b& output) {
  return threshold(input, output, 0, 255, THRESH_OTSU | THRESH_BINARY_INV);
}

void filter(const Mat1b& input, Mat1b& output) {
  medianBlur(input, output, 3);
  medianBlur(output, output, 3);
  medianBlur(output, output, 3);
}

// Experimental area thresholds
#define ROD_AREA_THRESHOLD_LOW 1500
#define ROD_AREA_THRESHOLD_UP 6000

// builTree returns a tree structure of rods, analyzing the bw image
// It draws detected rods on rois.
// If withInvalid is passed and setted to true, it returns even rods invalid
// (that are connected) and considered as a whole ros
vector<Rod> buildTree(const Mat1b& bw, Mat1b& rois, bool withInvalid = false) {
  vector<Rod> ret;
  ret.reserve(4);
  // Find all the contours in the binary image
  vector<Vec4i> hierarchy;
  vector<vector<Point>> contours;

  Mat1b bw_work;
  bw.copyTo(bw_work);

  findContours(bw_work, contours, hierarchy, CV_RETR_TREE,
               CV_CHAIN_APPROX_NONE);

  rois = Mat1b::zeros(bw.size());

  // define a tree to store the association
  // < root countour index, vector of child (contour index of the holes) of
  // the rod>
  map<int, vector<int>> tree;

  for (size_t i = 0; i < contours.size(); ++i) {
    // Calculate the area of each contour
    double area = contourArea(contours[i]);
    int fatherPosition = hierarchy[i][3];
    // does not have a father and its area passes the thresold(s)
    bool isRoot = false;
    bool isLeaf = false;
    if (fatherPosition < 0) {
      if (withInvalid) {
        isRoot = area > ROD_AREA_THRESHOLD_LOW;
      } else {
        isRoot = area > ROD_AREA_THRESHOLD_LOW && area < ROD_AREA_THRESHOLD_UP;
      }
    } else {
      // has a father and its father is a rod (passes the threshold(s))
      auto fatherArea = contourArea(contours[fatherPosition]);
      if (withInvalid) {
        isLeaf = fatherArea > ROD_AREA_THRESHOLD_LOW;
      } else {
        isLeaf = fatherArea > ROD_AREA_THRESHOLD_LOW &&
                 fatherArea < ROD_AREA_THRESHOLD_UP;
      }
    }

    if (isRoot || isLeaf) {
      if (isRoot) {
        tree[i] = vector<int>();
      } else {
        tree[fatherPosition].push_back(i);
      }
      // Draw each contour only for visualisation purposes
      drawContours(rois, contours, static_cast<int>(i), Scalar(255), 1, 8);
    }
  }

  for (const auto& rawRod : tree) {
    Rod rod(contours[rawRod.first], rois.size());
    for (const auto childIdx : rawRod.second) {
      Hole hole(contours[childIdx]);
      rod.holes.push_back(hole);
    }
    ret.push_back(rod);
  }
  return ret;
}

vector<Rod> buildTree(const Mat1b& bw, bool withInvalid = false) {
  Mat1b _;
  return buildTree(bw, _, withInvalid);
}

void drawAxis(Mat& img,
              Point p,
              Point q,
              Scalar colour,
              const float scale = 0.2) {
  double angle = atan2(static_cast<double>(p.y - q.y),
                       static_cast<double>(p.x - q.x));  // angle in radians

  double hypotenuse = sqrt(static_cast<double>((p.y - q.y) * (p.y - q.y) +
                                               (p.x - q.x) * (p.x - q.x)));
  // Here we lengthen the arrow by a factor of scale
  q.x = (int)(p.x - scale * hypotenuse * cos(angle));
  q.y = (int)(p.y - scale * hypotenuse * sin(angle));
  line(img, p, q, colour, 1, CV_AA);
  // create the arrow hooks
  p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
  p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
  line(img, p, q, colour, 1, CV_AA);
  p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
  p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
  line(img, p, q, colour, 1, CV_AA);
}

int main() {
  const char images[15][18] = {
      "./rods/TESI00.BMP", "./rods/TESI01.BMP", "./rods/TESI12.BMP",
      "./rods/TESI21.BMP", "./rods/TESI31.BMP", "./rods/Tesi33.bmp",
      "./rods/TESI44.BMP", "./rods/TESI47.BMP", "./rods/TESI48.BMP",
      "./rods/TESI49.BMP", "./rods/TESI50.BMP", "./rods/TESI51.BMP",
      "./rods/TESI90.BMP", "./rods/TESI92.BMP", "./rods/TESI98.BMP"};

  for (auto image : images) {
    if (strlen(image) == 0) {
      continue;
    }
    cout << "Image: " << image << "\n";

    // Read image in grayscale
    Mat1b gray = read(image);

    // Binaryze image
    Mat1b binary;
    binarize(gray, binary);

    // Remove noise
    Mat1b bw;
    filter(binary, bw);

    // Find connected components of not touching rods
    Mat1b rois;
    vector<Rod> tree = buildTree(bw, rois);

    // Remove every detected rods from the work image
    // thus we can focus on the research of connected rods
    // without affecting the founded ones
    Mat1b bw_work;
    bw.copyTo(bw_work);
    for (Rod& rod : tree) {
      vector<vector<Point>> fakeHierarchy;
      fakeHierarchy.push_back(rod.getContour());
      drawContours(bw_work, fakeHierarchy, 0, Scalar(0), CV_FILLED);
    }

    // It there are other objects, extract touching rods
    if (countNonZero(bw_work) > 0) {
      // Separate the rods in the image, using the watershed algorithm

      // First erode the binarized image the increase the degree of separation
      // between objects
      // Separe touching objects
      Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
      Mat1b bw_erosed;
      morphologyEx(bw_work, bw_erosed, MORPH_ERODE, element, Point(-1, -1), 1);

      // Apply the distance transform to the erosed image to get an image
      // with a distribution of pixel in function their distance
      Mat dist;
      distanceTransform(bw_erosed, dist, CV_DIST_L2, 3);

      // Normalize the distance image between [0,1] and threshold it with a
      // threshold of 0.5. Than dilate to make the rods bigger
      normalize(dist, dist, 0, 1, NORM_MINMAX);
      threshold(dist, dist, 0.5, 1, CV_THRESH_BINARY);

      // Convert the thresholded distance image back to a uchar matrix
      // and normalize its values between 0 and 255
      Mat dist_8u;
      dist.convertTo(dist_8u, CV_8U);
      normalize(dist_8u, dist_8u, 0, 255, NORM_MINMAX);

      // Extract contours of the "objects" (the result of the distance
      // transform) to get the seeds for the whatershed algoritm
      vector<vector<Point>> dt_contours;
      Mat1b dist_8u_work;
      dist_8u.copyTo(dist_8u_work);
      findContours(dist_8u_work, dt_contours, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_NONE);
      /*
       * WARNING: HIGH LEVEL OF MAGIC BELOW
       * Lets improve the whatershed segmentation performance exploting the
       * elongation of the rods.
       * In fact, a rod has an elongated body (where's the center of mass) and
       * ends with 1 or 2 holes that have no mass.
       * Therfore, to increase the chache of classify the hole perimeter as part
       * of the right rod, we draw a line that starts from the center of mass
       * (where the distance transform has its max value) and ends on the
       * opposite side of the detected hole along the direction of the rod
       */
      vector<Rod> blobs = buildTree(bw_work, true);
      // Extract every hole of evry blogs (we have no idea how many invalid rods
      // are present in the image)
      vector<Hole> holes;
      for (Rod rod : blobs) {
        holes.insert(holes.end(), rod.holes.begin(), rod.holes.end());
      }
      for (size_t i = 0; i < dt_contours.size(); i++) {
        // fake rod (to compute PCA)
        Rod rod(dt_contours[i], gray.size());
        vector<Point2d> eigen_vecs = rod.getEigenVectors();
        vector<double> eigen_val = rod.getEigenValues();

        // p1 shows the direction along the mayor axis.
        Point center = rod.getPosition();
        Point2i eigenPoint =
            Point2i(static_cast<int>(eigen_vecs[0].x * eigen_val[0]),
                    static_cast<int>(eigen_vecs[0].y * eigen_val[0]));

        // remove the actual contour from the distance transform image
        drawContours(dist_8u, dt_contours, i, Scalar(0), CV_FILLED);

        // Search on the posive and negative side of major axis

        // it will be replaced with a strong representation.
        // If at the end of the following loop, dist_8u is empty, no rods
        // are present in the image.
        // The following loop search for an intersection along the mayor axis of
        // the "rod" and the holes present in the "rod".
        // If an intersection is found, the "rod" is a rod.
        for (int j = 0, sign = -1; j < 2; ++j, sign += 2) {
          Point p1 = center + sign * eigenPoint;

          // If there's a hole along the direction of the greatest variance
          // that hole belogs to the current rod, thus must be connected
          Mat1b mainDirection = Mat1b::zeros(bw_work.size());
          // draw the lines (majox axis)
          line(mainDirection, center, p1, Scalar(255), 1);
          // draw the current contour
          drawContours(mainDirection, dt_contours, i, Scalar(255));

          // vector of candidates hole along the main direction (sign oriented)
          vector<pair<Hole, double>> candidates;
          candidates.reserve(holes.size());
          for (Hole& hole : holes) {
            // look for an intersection of the line with the current hole
            Mat1b holeImg = Mat1b::zeros(bw_work.size());
            circle(holeImg, hole.getCenter(), hole.getDiameter() / 2,
                   Scalar(255), CV_FILLED);
            Mat1b intersection;
            bitwise_and(mainDirection, holeImg, intersection);
            if (countNonZero(intersection) > 0) {  // if an intersection exists
              // add it to the candidate list
              candidates.push_back(
                  make_pair(hole, norm(center - hole.getCenter())));
            }
          }
          auto nearestIT =
              min_element(candidates.begin(), candidates.end(),
                          [](pair<Hole, double>& a, pair<Hole, double>& b) {
                            return a.second < b.second;
                          });

          if (nearestIT != candidates.end()) {
            Hole hole = (*nearestIT).first;
            // update dist_8u, connecting this circle with the current body rod

            // draw a line that connects the body with the holes
            // the line is the new body rod
            line(dist_8u, center, hole.getCenter(), Scalar(255),
                 static_cast<int>(floor(hole.getDiameter() / 2)));
            // In order to improve the rod detection, cirles must be the same
            // size of the OUTER circle
            // the hole is the inner circle. Thus we have to estimate the real
            // diameter
            // Since we have the center of the circle, we can compute the
            // distance from every point of the border of the rod from the
            // circle.
            // The minimum distance is the outer diameter

            // first we have to extract from the blob vector the contours of
            // every blob in the image
            vector<vector<Point>> blobContours;
            blobContours.reserve(blobs.size());
            for (Rod blob : blobs) {
              blobContours.push_back(blob.getContour());
            }

            // now we can merge the points in a vector of points
            vector<Point> fullContour;
            for (vector<Point> pts : blobContours) {
              fullContour.insert(fullContour.end(), pts.begin(), pts.end());
            }

            // now we can search for the point in the vector with a minimum
            // distance from the center of the hole
            int minDistance = INT_MAX;
            auto holeCenter = hole.getCenter();
            for (const auto& p : fullContour) {
              int dist = static_cast<int>(norm(p - holeCenter));
              if (dist < minDistance) {
                minDistance = dist;
              }
            }

            circle(dist_8u, hole.getCenter(), minDistance, Scalar(255),
                   CV_FILLED);
          }
        }
      }

      if (countNonZero(dist_8u) > 0) {
        // find contours again, to merge touching regions into one big region
        findContours(dist_8u, dt_contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_NONE);

        // Create the marker image for the watershed algorithm
        Mat markers = Mat::zeros(dist.size(), CV_32SC1);
        // Draw the foreground markers
        for (size_t i = 0; i < dt_contours.size(); i++) {
          Scalar color = Scalar::all(static_cast<int>(i) + 1);
          drawContours(markers, dt_contours, static_cast<int>(i), color,
                       CV_FILLED);
        }

        // Draw the background marker
        circle(markers, Point(5, 5), 3, CV_RGB(255, 255, 255), CV_FILLED);

        // Perform the watershed algorithm
        // convert bw_work in a color image (required by the algorithm)
        Mat color;

        cvtColor(gray, color, CV_GRAY2BGR);
        watershed(color, markers);

        Mat mark = Mat::zeros(markers.size(), CV_8UC1);
        markers.convertTo(mark, CV_8UC1);
        bitwise_not(mark, mark);

        // we don't need a classical usage of the watershed transform
        // (that's
        // colored)
        // we use watershed transfrom to segmentate objects. Thus, we set
        // foreground objects in white and background in black

        // Create the result image
        Mat1b dst_bw = Mat1b::zeros(markers.size());

        // Fill labeled objects
        // identify every rod with a color = 255 - index
        set<uchar> labels;
        for (int i = 0; i < markers.rows; i++) {
          for (int j = 0; j < markers.cols; j++) {
            int index = markers.at<int>(i, j);
            if (index > 0 && index <= static_cast<int>(dt_contours.size())) {
              dst_bw.at<uchar>(i, j) = 255 - index;
              labels.insert(static_cast<uchar>(index));
            } else {
              dst_bw.at<uchar>(i, j) = 0;
            }
          }
        }

        // treat every label separately (thus we can use findContours in
        // buildTree without any problem)
        for (uchar label : labels) {
          Mat1b onlyOne;
          dst_bw.copyTo(onlyOne);

          for (int i = 0; i < onlyOne.rows; i++) {
            for (int j = 0; j < onlyOne.cols; j++) {
              if (onlyOne.at<uchar>(i, j) != 255 - label) {
                onlyOne.at<uchar>(i, j) = 0;
              }
            }
          }

          // draw the holes into the image (holes are background: black color)
          for (Hole hole : holes) {
            fillConvexPoly(onlyOne, hole.getContour(), Scalar(0));
          }

          // Finally
          Mat1b nestedRoi;
          vector<Rod> touching = buildTree(onlyOne, nestedRoi);
          tree.insert(tree.end(), touching.begin(), touching.end());
          rois += nestedRoi;
        }
      }
    }

    for (Rod& rod : tree) {
      cout << rod << endl;
      // Draw MER
      Point2f rect_points[4];
      rod.getMER().points(rect_points);
      for (size_t i = 0; i < 4; ++i) {
        line(rois, rect_points[i], rect_points[(i + 1) % 4], Scalar(255));
      }

      // Draw a straight line perpendicular to the major axis and passing
      // throught the barycenter
      vector<Point2d> eigen_vecs = rod.getEigenVectors();
      vector<double> eigen_val = rod.getEigenValues();

      // p1 shows the direction along the mayor axis. Is the distance scaled
      // down by a factor of 0.02
      Point center = rod.getPosition();
      Point p1 = center +
                 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]),
                              static_cast<int>(eigen_vecs[0].y * eigen_val[0]));

      auto extremes = rod.getExtremesAtTheBarycenter();
      line(rois, extremes[0], extremes[1], Scalar(255));

      // Draw the center of the holes
      for (Hole& hole : rod.holes) {
        circle(rois, hole.getCenter(), 1, Scalar(255), 2);
      }

      // Draw barycenter
      circle(rois, center, 1, Scalar(255), 2);

      // Draw major axis
      drawAxis(rois, center, p1, Scalar(255), 1);
    }

    show(gray, "gray");
    show(rois, "analysis");
    waitKey(0);
  }

  waitKey(0);
  return EXIT_SUCCESS;
}
