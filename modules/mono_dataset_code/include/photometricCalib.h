#ifndef PHOTOMETRICCALIB_H
#define PHOTOMETRICCALIB_H
#include <QtWidgets/QTextBrowser>
#include <opencv2/opencv.hpp>
#include <string>
int responseCalib(std::string folder, QTextBrowser *textBrowser);
int vignetteCalib(std::string folder, QTextBrowser *textBrowser);
void findPointsInRange(cv::InputArray img, cv::Mat &result,
                       std::string cameraTxtPath);
#endif
