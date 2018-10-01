#
# Qt5 project CAR Radio by Hans-Juergen Arlt
#                          hj.arlt@online.de
#
QT += quick qml network widgets multimedia \
      multimediawidgets serialport location bluetooth

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# Avoid auto screen rotation
DEFINES += ORIENTATIONLOCK

CONFIG += c++11 declarative_debug qml_debug

PHONON_INSTALL_QT_EXTENSIONS_INTO_SYSTEM_QT

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
INCLUDEPATH += $$PWD/system $$PWD/src
DEPENDPATH += $$PWD/system

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH = $$PWD/qml

# Additional import path used to resolve QML modules just for Qt Quick Designer
QML_DESIGNER_IMPORT_PATH =

#PRE_TARGETDEPS += $$PWD/system/libid3v2.a
LIBS += -L$$PWD/system/ -lasound

if(linux-g++) {
    message("We are on Linux Host mode.")
    LIBS += -lid3v2_x86
    DEFINES += HOST_MODE
} else {
    message("We are on Linux ARM Target mode.")
    LIBS += -lid3v2_arm
    DEFINES += TARGET_MODE
}

SOURCES += \
        main.cpp \
    src/AudioControl.cpp \
    src/SystemCommand.cpp \
    src/FileUtil.cpp \
    src/TunerControl.cpp \
    src/RdsList.c \
    src/GnssPosition.cpp \
    src/TcpClient.cpp \
    src/Bluetooth.cpp

HEADERS += \
    src/SystemCommand.h \
    src/AudioControl.h \
    src/FileUtil.h \
    src/TunerControl.h \
    src/TunerDefines.h \
    src/RdsList.h \
    src/GnssPosition.h \
    src/TcpClient.h \
    src/Bluetooth.h

RESOURCES += qml.qrc

ICON =

# Default rules for deployment.
# target.path = /opt/$${TARGET}/bin

target.files    = car-radio
target.path     = /home/pi/car-radio
qmlAppfw.files  = qml/AppFW/*.qml
qmlAppfw.path   = /home/pi/car-radio/qml/AppFW
qmlMusic.files  = qml/Music/*.qml
qmlMusic.path   = /home/pi/car-radio/qml/Music
qmlVideo.files  = qml/Video/*.qml
qmlVideo.path   = /home/pi/car-radio/qml/Video
qmlRadio.files  = qml/Radio/*.qml
qmlRadio.path   = /home/pi/car-radio/qml/Radio
qmlNavi.files   = qml/Navigation/*.qml
qmlNavi.path    = /home/pi/car-radio/qml/Navigation

INSTALLS       += target qmlMusic qmlVideo qmlRadio qmlNavi qmlAppfw
