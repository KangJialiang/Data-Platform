#include "include/camera.h"
#include <QDebug>
#include <QThread>
#include<QFile>
#include<QFileDialog>

#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

Camera::Camera(QObject *parent) : QObject(parent)
{

}
int64_t cam_CurrentTimeMillis()
{
    int64_t timems = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return timems;
}
bool make_dir(char *dirname)
{
    if (access(dirname, 0)==-1) {
        return (mkdir(dirname,0777)==0);
      }
    else {
        return true;
    }
}

void Camera::get_img(){
    //create a file to log camera timestamp
    char cam_time_dir[100];
    sprintf(cam_time_dir, "%s/cam_info.txt", dir_name);
    float imu[6];
    QFile ftxt1(cam_time_dir);
    if(!ftxt1.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug()<<"ftxt open failed!";
        emit file_error(cam_time_dir);
    }
    QTextStream txtOutput(&ftxt1);
    char dirname[100];
    sprintf(dirname,"%s/images",dir_name);
    if(make_dir(dirname)==false)
        emit file_error(dirname);
    sprintf(dirname, "%s/%s/image_0", dir_name, "images");
    if(make_dir(dirname)==false)
        emit file_error(dirname);
    sprintf(dirname, "%s/%s/image_1", dir_name, "images");
    if(make_dir(dirname)==false)
        emit file_error(dirname);
    sprintf(dirname, "%s/%s/image_2", dir_name, "images");
    if(make_dir(dirname)==false)
        emit file_error(dirname);

    qDebug()<<"camera thread started at:"<<QThread::currentThreadId();
    //some parameters
    int imgWidth  = 1280;
    int imgHeight = 720;
    int cropU = 0;
    int cropV = 0;
    int cropWidth  = 1280;
    int cropHeight = 720;
    double visual_ratio = 1;
    // initialize realsense camera
    // contruct a pipeline which abstracts the device
    rs2::pipeline pipe;
    // create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_GYRO,RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_ACCEL,RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_COLOR, imgWidth, imgHeight, RS2_FORMAT_BGR8, 15);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, imgWidth, imgHeight, RS2_FORMAT_Y8, 15);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, imgWidth, imgHeight, RS2_FORMAT_Y8, 15);
    rs2::pipeline_profile selection = pipe.start(cfg);
    rs2::device selected_device = selection.get_device();

    rs400::advanced_mode advnc_mode(selected_device);
    STAEControl ae_ctrl;
    ae_ctrl.meanIntensitySetPoint = 1000;
    advnc_mode.set_ae_control(ae_ctrl);

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    rs2::sensor color_sensor = sensors[1];
    if(exposure!=0)
    {
        color_sensor.set_option(RS2_OPTION_EXPOSURE, exposure);
    }
//    disable emitter
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
    //Camera warmup - dropping several first frames to let auto-exposure stabilize
    //Add desired streams to configuration

   rs2::frameset frames;
   for (int i = 0; i < 30; i++)
   {
       // Wait for all configured streams to produce a frame
       frames = pipe.wait_for_frames();
   }



   for (int i = 0; quit == false; i++)
   {
       frames = pipe.wait_for_frames();       
       //Get each frame
       rs2::frame frame = frames.get_color_frame();
       rs2::frame frameLeft  = frames.get_infrared_frame(1);
       rs2::frame frameRight = frames.get_infrared_frame(2);
       qDebug()<<"got each frame";
       //Get current timestamp
       int64_t timestamp = cam_CurrentTimeMillis();
       qint64 tt(timestamp);
       qDebug()<<"got time";
       txtOutput << tt << ' ';
       txtOutput<<frameLeft.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)*0.001 << " "<< frameLeft.get_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL)<<' '<<\
               frameRight.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)*0.001 << " " <<frameRight.get_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL)<<' ';

       // Creating OpenCV Matrix from a color image
       cv::Mat color(cv::Size(imgWidth, imgHeight), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
       cv::Mat matLeft (cv::Size(imgWidth, imgHeight), CV_8U, (void*)frameLeft.get_data(),  cv::Mat::AUTO_STEP);
       cv::Mat matRight(cv::Size(imgWidth, imgHeight), CV_8U, (void*)frameRight.get_data(), cv::Mat::AUTO_STEP);
       qDebug()<<"got Mat";
       //imu data collection
       if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
           {
               rs2_vector accel_sample = accel_frame.get_motion_data();
               imu[0]=accel_sample.x;
               imu[1]=accel_sample.y;
               imu[2]=accel_sample.z;
               txtOutput<<accel_sample.x<<' '<<accel_sample.y<<' '<<accel_sample.z<<' ';
           }

           if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
           {
               rs2_vector gyro_sample = gyro_frame.get_motion_data();
               imu[3]=gyro_sample.x;
               imu[4]=gyro_sample.y;
               imu[5]=gyro_sample.z;
               txtOutput<<gyro_sample.x<<' '<<gyro_sample.y<<' '<<gyro_sample.z<<endl;
           }
       emit imu_data(imu);
       //Crop images
       cv::Rect rect(cropU, cropV, cropWidth, cropHeight);
       cv::Mat matCroppedColor = color(rect);
       cv::Mat matCroppedLeft  = matLeft(rect);
       cv::Mat matCroppedRight = matRight(rect);
       qDebug()<<"got cropped cvMat";
       //Saving images
       char nameImg[20], nameColor[100], nameLeft[100], nameRight[100];
       sprintf(nameImg, "%06d%s", i, ".png");
       sprintf(nameColor, "%s/%s/image_0/%s", dir_name, "images", nameImg);
       sprintf(nameLeft,  "%s/%s/image_1/%s", dir_name, "images", nameImg);
       sprintf(nameRight, "%s/%s/image_2/%s", dir_name, "images", nameImg);

       cv::imwrite(nameColor, matCroppedColor);
       cv::imwrite(nameLeft,  matCroppedLeft);
       cv::imwrite(nameRight, matCroppedRight);

       cv::cvtColor(matCroppedColor,image,cv::COLOR_BGR2RGB);
       //缩放
       cv::Size dsize = cv::Size(image.cols*visual_ratio, image.rows*visual_ratio);
       cv::Mat image2 = cv::Mat(dsize, CV_32S);
       cv::resize(image, image2, dsize);
       QImage img = QImage((const unsigned char*)(image2.data),image2.cols,image2.rows,QImage::Format_RGB888);
       qDebug()<<"got QImage ready for visualization";
       emit send_img(img);
       qDebug()<<"camera calling mainwindow to show image";
       //usleep(200*1000);
   }
   ftxt1.close();
}

void Camera::look_img(){
    qDebug()<<"camera thread started at:"<<QThread::currentThreadId();
    float imu[6];
    //some parameters
    int imgWidth  = 640;
    int imgHeight = 480;
    int cropU = 0;
    int cropV = 0;
    int cropWidth  = 640;
    int cropHeight = 480;
    double visual_ratio = 1;
    // initialize realsense camera
    // contruct a pipeline which abstracts the device
    rs2::pipeline pipe;
    // create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_GYRO,RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_ACCEL,RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_COLOR, imgWidth, imgHeight, RS2_FORMAT_BGR8, 15);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, imgWidth, imgHeight, RS2_FORMAT_Y8, 15);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, imgWidth, imgHeight, RS2_FORMAT_Y8, 15);

    //Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile selection = pipe.start(cfg);
    qDebug()<<exposure;
    rs2::device selected_device = selection.get_device();
    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    rs2::sensor color_sensor = sensors[1];
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    if(exposure!=0)
    {
        qDebug()<<"setting exposure";
        color_sensor.set_option(RS2_OPTION_EXPOSURE, exposure);
    }


    //Camera warmup - dropping several first frames to let auto-exposure stabilize
   rs2::frameset frames;
   for (int i = 0; i < 30; i++)
   {
       // Wait for all configured streams to produce a frame

       frames = pipe.wait_for_frames();
       //qDebug()<<"warming up";
   }

   for (int i = 0; quit == false; i++)
   {
       frames = pipe.wait_for_frames();

       //Get each frame
       rs2::frame frame = frames.get_color_frame();
       qDebug()<<"got each frame";
       // Creating OpenCV Matrix from a color image
       cv::Mat color(cv::Size(imgWidth, imgHeight), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
       //Crop images
       cv::Rect rect(cropU, cropV, cropWidth, cropHeight);
       cv::Mat matCroppedColor = color(rect);
       qDebug()<<"got cropped cvMat";
       cv::cvtColor(matCroppedColor,image,cv::COLOR_BGR2RGB);
       //缩放
       cv::Size dsize = cv::Size(image.cols*visual_ratio, image.rows*visual_ratio);
       cv::Mat image2 = cv::Mat(dsize, CV_32S);
       cv::resize(image, image2, dsize);
       QImage img = QImage((const unsigned char*)(image2.data),image2.cols,image2.rows,QImage::Format_RGB888);
       qDebug()<<"got QImage ready for visualization";
       emit send_img(img);
       qDebug()<<"camera calling mainwindow to show image";
       if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
           {
               rs2_vector accel_sample = accel_frame.get_motion_data();
               imu[0]=accel_sample.x;
               imu[1]=accel_sample.y;
               imu[2]=accel_sample.z;
               qDebug()<<"Accel:"<<accel_sample.x<<' '<<accel_sample.y<<' '<<accel_sample.z<<endl;
           }

           if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
           {
               rs2_vector gyro_sample = gyro_frame.get_motion_data();
               imu[3]=gyro_sample.x;
               imu[4]=gyro_sample.y;
               imu[5]=gyro_sample.z;
               qDebug()<<"Gyro:"<<gyro_sample.x<<' '<<gyro_sample.y<<' '<<gyro_sample.z<<endl;
           }
       emit imu_data(imu);
       //usleep(200*1000);
   }

}

void Camera::exit_collect(){

    quit=true;
    return;
}
