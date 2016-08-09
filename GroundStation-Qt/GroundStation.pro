#-------------------------------------------------
#
# Project created by QtCreator 2015-12-24T14:48:17
#
#-------------------------------------------------

QT       += core gui serialport printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = groundstation
TEMPLATE = app
RC_FILE = groundstation.rc


SOURCES += main.cpp\
        mainwindow.cpp \
    connection.cpp \
    packet.cpp \
    prefs.cpp \
    widgets/altimeter_widget.cpp \
    widgets/angle_widget.cpp \
    widgets/gauge_widget.cpp \
    widgets/heading_widget.cpp \
    widgets/horizon_widget.cpp \
    widgets/linefit_widget.cpp \
    widgets/movingaverage.cpp \
    widgets/orientation_widget.cpp \
    widgets/radiostick_widget.cpp \
    widgets/valuebar_widget.cpp \
    aboutbox.cpp \
    quatutil.cpp \
    qcustomplot.cpp \
    ahrs.cpp

HEADERS  += mainwindow.h \
    connection.h \
    packet.h \
    elev8data.h \
    prefs.h \
    widgets/altimeter_widget.h \
    widgets/angle_widget.h \
    widgets/gauge_widget.h \
    widgets/heading_widget.h \
    widgets/horizon_widget.h \
    widgets/linefit_widget.h \
    widgets/movingaverage.h \
    widgets/orientation_widget.h \
    widgets/radiostick_widget.h \
    widgets/valuebar_widget.h \
    aboutbox.h \
    quatutil.h \
    qcustomplot.h \
    ahrs.h

FORMS    += mainwindow.ui \
    aboutbox.ui

DISTFILES +=

RESOURCES += \
    resources.qrc
