#-------------------------------------------------
#
# Project created by QtCreator 2020-11-02T21:25:57
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = signals_and_slots
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

###################################对opencv的支持
INCLUDEPATH += /usr/local/include/opencv4 \
               /home/ubuntu/opencv/build/

LIBS += -L /usr/local/lib \
        #/home/ubuntu/opencv/build/lib/libopencv_calib3d.so \
        #/usr/local/lib/libopencv_core.so.4.1.2 \
        #/usr/local/lib/libopencv_core.so.4.1 \
        /usr/local/lib/libopencv_core.so \
        #/home/ubuntu/opencv/build/lib/libopencv_features2d.so\
        #/home/ubuntu/opencv/build/lib/libopencv_flann.so\
        #/home/ubuntu/opencv/build/lib/libopencv_highgui.so\
        /home/ubuntu/opencv/build/lib/libopencv_imgcodecs.so\
        /home/ubuntu/opencv/build/lib/libopencv_imgproc.so\
        #/home/ubuntu/opencv/build/lib/libopencv_ml.so\
        #/home/ubuntu/opencv/build/lib/libopencv_objdetect.so\
        #/home/ubuntu/opencv/build/lib/libopencv_photo.so\
        #/home/ubuntu/opencv/build/lib/libopencv_superres.so\
        #/home/ubuntu/opencv/build/lib/libopencv_shape.so\
        #/home/ubuntu/opencv/build/lib/libopencv_stitching.so\
        #/home/ubuntu/opencv/build/lib/libopencv_videoio.so\
        #/home/ubuntu/opencv/build/lib/libopencv_video.so\
        #/home/ubuntu/opencv/build/lib/libopencv_videostab.so\
        /usr/local/lib/librealsense2.so \
        /usr/local/lib/librealsense2.so.2.36 \
        /usr/local/lib/librealsense2.so.2.36.0

###################################

CONFIG += c++11

SOURCES += \
    src/camera.cpp \
    src/main.cpp \
    src/mainwindow.cpp
    #mygraphicsview.cpp

HEADERS += \
    include/camera.h \
    include/mainwindow.h
    #mygraphicsview.h

FORMS += \
        mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
