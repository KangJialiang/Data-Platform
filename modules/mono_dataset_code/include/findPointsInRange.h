#ifndef FINDPOINTSINRANGE_H
#define FINDPOINTSINRANGE_H
#include <opencv2/opencv.hpp>
#include <string>
void findPointsInRange(cv::InputArray img, cv::Mat &result,
                       std::string cameraTxtPath);
#endif