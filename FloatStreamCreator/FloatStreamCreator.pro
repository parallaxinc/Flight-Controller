#-------------------------------------------------
#
# Project created by QtCreator 2016-09-18T12:40:18
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = FloatStreamCreator
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
    floatfunctions.cpp \
    inputsandoutputs.cpp \
    functionstream.cpp \
    functioncompiler.cpp \
    expressionparser.cpp \
    expressiontokenizer.cpp \
    expression.cpp \
    connection.cpp \
    packet.cpp \
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
    quatutil.cpp

HEADERS  += mainwindow.h \
    floatfunctions.h \
    inputsandoutputs.h \
    inputoutputlist.h \
    functioncompiler.h \
    expression.h \
    expressionparser.h \
    expressiontokenizer.h \
    connection.h \
    elev8data.h \
    packet.h \
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
    quatutil.h \
    functionstream.h

FORMS    += mainwindow.ui

RESOURCES += \
    resources.qrc
