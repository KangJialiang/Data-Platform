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

Camera::intrinsicT USBCamera::getIntrinsic() {}