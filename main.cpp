#include <iostream>
#include <vector>
#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

Mat1b read(const char* path) {
  const String name(path);
  return imread(name, IMREAD_GRAYSCALE);
}

void show(const Mat1b& image) {
  static int counter = 1;
  string winName = string("window " + to_string(counter));
  namedWindow(winName);
  imshow(winName, image);
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

int main() {
  const char images[15][18] = {
      "./rods/TESI00.BMP", "./rods/TESI01.BMP", "./rods/TESI12.BMP",
      "./rods/TESI21.BMP", "./rods/TESI31.BMP", "./rods/Tesi33.bmp",
      "./rods/TESI44.BMP", "./rods/TESI47.BMP", "./rods/TESI48.BMP",
      "./rods/TESI49.BMP", "./rods/TESI50.BMP", "./rods/TESI51.BMP",
      "./rods/TESI90.BMP", "./rods/TESI92.BMP", "./rods/TESI98.BMP"};

  for (auto image : images) {
    cout << "Processing " << image << endl;
    Mat1b gray = read(image);
    Mat1b binary;
    double OtsuThreshold = binarize(gray, binary);
    Mat1b filtered;
    filter(binary, filtered);
    show(filtered);
    // Mat1b edgeImage;
    // edge(filtered, edgeImage, OtsuThreshold);
    // show(edgeImage);
  }

  waitKey(0);
  return EXIT_SUCCESS;
}
