QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

#DEFINES += ROS_OPENCV
DEFINES += NOROS
#DEFINES += TX2BUILD
DEFINES += DESKTOP_BUILD
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
INCLUDEPATH += /opt/Vimba_6_0
INCLUDEPATH += /usr/include/opencv4/

#INCLUDEPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev/
LIBS += -L/opt/ros/kinetic/lib/x86_64-linux-gnu/
LIBS += -L/usr/local/lib
LIBS += -L../../dsvimlib/lib
LIBS += -ldsvimlib
LIBS += -llcm
#LIBS += -lmesobot-lcmtypes


contains(DEFINES,TX2BUILD){
LIBS += -L/opt/Vimba_2_1/VimbaCPP/DynamicLib/arm_64bit -lVimbaCPP -lVimbaC -llcm
LIBS += -L/opt/Vimba_2_1/VimbaImageTransform/DynamicLib/arm_64bit -lVimbaImageTransform
LIBS += -lopencv_core
LIBS += -lopencv_highgui
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_imgproc
LIBS += -lopencv_videoio
}

contains(DEFINES,DESKTOP_BUILD){
LIBS += -L/opt/Vimba_6_0/VimbaCPP/DynamicLib/x86_64bit -lVimbaCPP -lVimbaC -llcm
LIBS += -L/opt/Vimba_6_0/VimbaImageTransform/DynamicLib/x86_64bit -lVimbaImageTransform
}

contains(DEFINES,ROS_OPENCV){
LIBS += -lopencv_core3
LIBS += -lopencv_highgui3
LIBS += -lopencv_imgcodecs3
LIBS += -lopencv_imgproc3
LIBS += -lopencv_videoio3

}
contains(DEFINES,NOROS){

LIBS += -lopencv_core
LIBS += -lopencv_highgui
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_imgproc
LIBS += -lopencv_videoio
LIBS += -lexiv2
}

SOURCES += \
    AVTAttribute.cpp \
    CameraObserver.cpp \
    FrameObserver.cpp \
    lcmHandleThread.cpp \
    microstrain.cpp \
    msNetThread.cpp \
    nmea.cpp \
    sensorThread.cpp \
    simulationThread.cpp \
    vimc.cpp \
    vimcBusThread.cpp \
    vimcThread.cpp \
    vimcLoggingThread.cpp \
    stereoLoggingThread.cpp
    AVTAttribute.cpp

HEADERS += \
    CameraObserver.h \
    FrameObserver.h \
    lcmHandleThread.h \
    lcmRecieveThread.h \
    microstrain.h \
    msNetThread.h \
    nmea.h \
    sensorThread.h \
    simulationThread.h \
    vimc.h \
    vimcBusThread.h \
    vimcThread.h \
    vimcLoggingThread.h \
    AVTAttribute.h \
    stereoLoggingThread.h

