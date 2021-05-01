#ifndef PRINTF_H
#define PRINTF_H
#include <QtWidgets/QTextBrowser>

extern QTextBrowser *setTextTest;

#define printf(...)                             \
  do {                                          \
    char haskdfhaksjfh[1000];                   \
    snprintf(haskdfhaksjfh, 1000, __VA_ARGS__); \
  } while (0)

#endif
