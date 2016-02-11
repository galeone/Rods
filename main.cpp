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

void edge(const Mat1b& input, Mat1b& output, double otsuThreshold) {
  Canny(input, output, 0.5 * otsuThreshold, otsuThreshold);
}

#define ROD_AREA_THRESHOLD_LOW 1500
#define ROD_AREA_THRESHOLD_UP 6000

vector<Rod> buildTree(const Mat1b& bw, Mat1b& rois) {
  vector<Rod> ret;
  ret.reserve(4);
  // Find all the contours in the thresholded image
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
    // does not have a father and its area passes the thresold
    bool isRoot = false;
    bool isLeaf = false;
    if (fatherPosition < 0) {
      isRoot = area > ROD_AREA_THRESHOLD_LOW && area < ROD_AREA_THRESHOLD_UP;
    } else {
      // has a father and its father is a rod (passes the threshold)
      auto fatherArea = contourArea(contours[fatherPosition]);
      isLeaf = fatherArea > ROD_AREA_THRESHOLD_LOW &&
               fatherArea < ROD_AREA_THRESHOLD_UP;
    }

    if (isRoot || isLeaf) {
      cout << "[!] Shape area: " << area << "\n";
      if (isRoot) {
        tree[i] = vector<int>();
      } else {
        tree[fatherPosition].push_back(i);
      }
      // Draw each contour only for visualisation purposes
      // Find the orientation of each shape
      // getOrientation(contours[i], color);
      drawContours(rois, contours, static_cast<int>(i), Scalar(255), 1, 8);
    }
  }

  for (const auto& rawRod : tree) {
    Rod rod(contours[rawRod.first]);
    for (const auto childIdx : rawRod.second) {
      Hole hole(contours[childIdx]);
      rod.holes.push_back(hole);
      // cout << hole.getCenter() << endl;
    }
    ret.push_back(rod);
  }
  return ret;
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
    cout << "Processing " << image << endl;
    Mat1b gray = read(image);
    Mat1b binary;
    binarize(gray, binary);
    Mat1b bw;
    filter(binary, bw);

    Mat1b rois;
    vector<Rod> tree = buildTree(bw, rois);
    size_t treeSize = tree.size();

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

    // show(bw_work);

    Mat1b bw_erosed;
    Mat dist;
    bw_work.copyTo(bw_erosed);

    Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
    morphologyEx(bw_work, bw_erosed, MORPH_ERODE, element, Point(-1, -1), 1);
    // show(bw_erosed);

    distanceTransform(bw_erosed, dist, CV_DIST_L2, 3);

    normalize(dist, dist, 0, 1, NORM_MINMAX);
    threshold(dist, dist, 0.5, 1, CV_THRESH_BINARY);
    // morphologyEx(dist, dist, MORPH_DILATE, element, Point(-1, -1), 1);

    // show(dist);

    Mat1b dist_8u;
    dist.convertTo(dist_8u, CV_8UC1);
    bitwise_not(dist_8u, dist_8u);
    binarize(dist_8u, dist_8u);
    // show(dist_8u);

    vector<vector<Point>> contours;
    findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Create the marker image for the watershed algorithm
    Mat markers = Mat::zeros(dist.size(), CV_32SC1);

    // Draw the foreground markers
    for (size_t i = 0; i < contours.size(); i++) {
      drawContours(markers, contours, static_cast<int>(i),
                   Scalar::all(static_cast<int>(i) + 1), CV_FILLED);
    }

    // Draw the background marker
    circle(markers, Point(5, 5), 3, CV_RGB(255, 255, 255), CV_FILLED);

    // Perform the watershed algorithm
    Mat color;
    cvtColor(bw_work, color, CV_GRAY2BGR);
    watershed(color, markers);

    Mat mark = Mat::zeros(markers.size(), CV_8UC1);
    markers.convertTo(mark, CV_8UC1);
    bitwise_not(mark, mark);

    // Generate random colors
    vector<Vec3b> colors;
    for (size_t i = 0; i < contours.size(); i++) {
      int b = theRNG().uniform(0, 255);
      int g = theRNG().uniform(0, 255);
      int r = theRNG().uniform(0, 255);

      colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
    }

    // Create the result image
    Mat dst = Mat::zeros(markers.size(), CV_8UC3);

    // Fill labeled objects with random colors
    for (int i = 0; i < markers.rows; i++) {
      for (int j = 0; j < markers.cols; j++) {
        int index = markers.at<int>(i, j);
        if (index > 0 && index <= static_cast<int>(contours.size()))
          dst.at<Vec3b>(i, j) = colors[index - 1];
        else
          dst.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
      }
    }

    // Visualize the final image
    // show(dst);

    for (Rod& rod : tree) {
      cout << rod << endl;
      Point2f rect_points[4];
      rod.getMER().points(rect_points);
      for (size_t i = 0; i < 4; ++i) {
        line(rois, rect_points[i], rect_points[(i + 1) % 4], Scalar(255));
      }
      circle(rois, rod.getPosition(), 1, Scalar(255), 2);
      auto eigenVect = rod.getEigenVectors();
      auto eigenVal = rod.getEigenValues();
      cout << eigenVect[0] << " " << eigenVect[1] << "V: " << eigenVal[0] << " "
           << eigenVal[1] << "\n";
      for (Hole& hole : rod.holes) {
        circle(rois, hole.getCenter(), 1, Scalar(255), 2);
      }

      Point2d q = rod.getPosition(), p = Point2f(eigenVect[0].x * eigenVal[0],
                                                 eigenVect[0].y * eigenVal[0]);
      double angle;
      double hypotenuse;
      angle = atan2((double)p.y - q.y, (double)p.x - q.x);  // angle in radians
      hypotenuse =
          sqrt((double)(p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
      float scale = 1;
      q.x = (int)(p.x - scale * hypotenuse * cos(angle));
      q.y = (int)(p.y - scale * hypotenuse * sin(angle));
      line(rois, p, q, Scalar(255), 1, CV_AA);
    }

    // show(bw_work, string(image + string("a")));
    show(rois, string(image));
  }

  waitKey(0);
  return EXIT_SUCCESS;
}
