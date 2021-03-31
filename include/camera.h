#ifndef CAMERA_H
#define CAMERA_H

#include <QObject>
#include <QImage>
#include <opencv2/opencv.hpp>

class Camera : public QObject
{
    Q_OBJECT
public:
    explicit Camera(QObject *parent = nullptr);
    int exposure=0;
    char dir_name[100];
signals:
    void send_img(QImage img);
    void file_error(char *dirname);
    void imu_data(float*);

public slots:
    void exit_collect();
    void get_img();
    void look_img();

private:
    cv::Mat image;
    bool quit=false;
};

#endif // CAMERA_H
