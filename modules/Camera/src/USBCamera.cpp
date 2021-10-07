#include "USBCamera.h"

USBCamera::USBCamera() : cap(0), startFlag(true) {
  // cv::VideoCapture cap(0);
  widthOfFrame = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  heightOfFrame = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  pthread_spin_init(&frameLock, PTHREAD_PROCESS_PRIVATE);
  pthread_create(
      &readThread, 0,
      [](void* arg) {
        USBCamera* a = (USBCamera*)arg;
        a->readFrame();
        return (void*)0;
      },
      this);
}
USBCamera::~USBCamera() {
  startFlag = false;
  // readThread.join();
  pthread_join(readThread, NULL);
  pthread_spin_destroy(&frameLock);
}
cv::Mat USBCamera::getFrame() {
  // m.lock();
  pthread_spin_lock(&frameLock);
  cv::Mat tempFrame = currentFrame;
  // m.unlock();
  pthread_spin_unlock(&frameLock);
  return tempFrame;
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

void USBCamera::readFrame() {
  while (startFlag) {
    // m.lock();
    pthread_spin_lock(&frameLock);
    cap >> currentFrame;
    // m.unlock();
    pthread_spin_lock(&frameLock);
  }
}