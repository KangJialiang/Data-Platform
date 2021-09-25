#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>

class Camera {
 public:
  typedef struct {
    float fx;
    float fy;
    float cx;
    float cy;
    float coeffs[5];  // Order for Brown-Conrady: [k1, k2, p1, p2, k3].
  } intrinsicT;

 public:
  Camera() = default;
  ~Camera() = default;

  virtual cv::Mat getFrame() = 0;
  virtual cv::Mat getFrame(int exposureTime){};
  virtual cv::Mat getFrame(int exposureTime, long long& timeOfArrival){};
  virtual void getResolution(int& width, int& height) = 0;
  virtual intrinsicT getIntrinsic() = 0;
};

#endif