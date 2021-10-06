#ifndef USBCAMERA_H
#define USBCAMERA_H

#include <functional>
#include <mutex>
#include <thread>

#include "Camera.h"

class USBCamera : public Camera {
 public:
  USBCamera();
  ~USBCamera() = default;
  cv::Mat getFrame() override;
  void getResolution(int& width, int& height) override;
  intrinsicT getIntrinsic() override;
  void setIntrinsic(float fx, float fy, float cx, float cy, float k1, float k2,
                    float p1, float p2, float k3);

 private:
  cv::VideoCapture cap;
  int widthOfFrame;
  int heightOfFrame;
  intrinsicT intrinsic;

  cv::Mat currentFrame;
  std::mutex m;
  std::thread readThread;
  void readFrame();
};
#endif