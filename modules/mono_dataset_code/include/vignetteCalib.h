#ifndef VIGNETTECALIB_H
#define VIGNETTECALIB_H
#include <QLabel>
#include <QtWidgets/QTextBrowser>
#include <string>
void vignetteCalib(std::string folder, QTextBrowser* textBrowser,
                   std::string folderforResponseCalib, QLabel* label);
#endif
