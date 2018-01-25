#-------------------------------------------------
#
# Project created by QtCreator 2015-10-22T17:28:39
#
#-------------------------------------------------

QT       += core gui

CONFIG += c++11

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Kinematic_Arc_Solver
TEMPLATE = app

DEPLOYMENT.display_name = Kinematic Arc Solver

win32: RC_ICONS = logo.ico

SOURCES += main.cpp \
    mainwindow.cpp

HEADERS  += \
    mainwindow.h \
    rangeddata.h \
    kinematics_thread.h

FORMS    += \
    mainwindow.ui

RESOURCES += \
    resources.qrc

DISTFILES += \
    android/AndroidManifest.xml \
    android/res/values/libs.xml \
    android/build.gradle

ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android
