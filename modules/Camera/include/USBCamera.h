#ifndef USBCAMERA_H
#define USBCAMERA_H

#include "Camera.h"

class USBCamera : public Camera {
 public:
  USBCamera();
  ~USBCamera() = default;
  cv::Mat getFrame() override;
  void getResolution(int& width, int& height) override;
  intrinsicT getIntrinsic() override;

 private:
  cv::VideoCapture cap;
  int widthOfFrame;
  int heightOfFrame;
}
#endif