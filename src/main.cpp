#include <ros/ros.h>

#include <QApplication>

#include "mainwindow.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  ros::init(argc, argv, "node");
  MainWindow w;
  w.show();
  return a.exec();
}
