#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <string>
#include <QGraphicsScene>
#include <QPainter>
#include <QPainterPath>
#include <QList>
#include <QRectF>

#include <QThread>
#include <include/camera.h>
//#include "visual_traj.h"

namespace Ui {

class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
signals:
    void cam_start();
    void cam_stop();
    void cam_look();
public slots:
    void visual_image(QImage img);
    void camera_error(char *dirname);
    void visual_imu(float*);

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::MainWindow *ui;
    bool conti = true;
    QGraphicsScene *scene;
    QPainterPath traj;
    QList<double> xList;
    QList<double> yList;
    const QRectF boundingRect=QRectF(30.5,120.4,1.5,1.5);
    //RenderArea *originalRenderArea;
    QThread cam_th;
    Camera camera;
    char dir_name[100];
    //visual_traj my_visual;
};

#endif // MAINWINDOW_H
