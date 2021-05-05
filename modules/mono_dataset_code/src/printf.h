#ifndef PRINTF_H
#define PRINTF_H
#include <QtWidgets/QTextBrowser>

extern QTextBrowser *setTextTest;

#define printf(...)                        \
  do {                                     \
    char tempBuff[1000];                   \
    snprintf(tempBuff, 1000, __VA_ARGS__); \
  } while (0)

#endif
