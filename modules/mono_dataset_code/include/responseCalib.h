#ifndef RESPONSECALIB_H
#define RESPONSECALIB_H
#include <QLabel>
#include <QtWidgets/QTextBrowser>
#include <string>
void responseCalib(std::string folder, QTextBrowser *textBrowser, QLabel *label,
                   const bool &stopResponseCalib);
#endif
