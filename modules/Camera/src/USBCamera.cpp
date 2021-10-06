#include "USBCamera.h"

USBCamera::USBCamera() {
  cv::VideoCapture cap(0);
  widthOfFrame = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  heightOfFrame = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
}
cv::Mat USBCamera::getFrame() {
  cv::Mat frame;
  cap >> frame;
  return frame.clone();
}
void USBCamera::getResolution(int& width, int& height) {
  width = widthOfFrame;
  height = heightOfFrame;
}

Camera::intrinsicT USBCamera::getIntrinsic() { return intrinsic; }
void USBCamera::setIntrinsic(float fx, float fy, float cx, float cy, float k1,
                             float k2, float p1, float p2, float k3) {
  intrinsic.fx = fx;
  intrinsic.fy = fy;
  intrinsic.cx = cx;
  intrinsic.cy = cy;
  intrinsic.coeffs[0] = k1;
  intrinsic.coeffs[1] = k2;
  intrinsic.coeffs[2] = p1;
  intrinsic.coeffs[3] = p2;
  intrinsic.coeffs[4] = k3;
}