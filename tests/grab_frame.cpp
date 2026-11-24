#include <chrono>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"

int main(int argc, char ** argv)
{
  io::Camera cam(argv[1]);
  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  for (int i = 0; i < 10; ++i) cam.read(img, t);
  if (img.empty()) {
    printf("empty frame\n");
    return 1;
  }
  cv::imwrite("/tmp/frame.jpg", img);
  cv::Scalar m = cv::mean(img);
  printf("size %dx%d mean B=%.1f G=%.1f R=%.1f\n", img.cols, img.rows, m[0], m[1], m[2]);
  return 0;
}
