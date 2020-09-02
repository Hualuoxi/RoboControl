#DEFINES += QT_DEPRECATED_WARNINGS
#CONFIG += serialport
QT -= gui
QT += core
target.path = /root/robotapp
INSTALLS += target
CONFIG += c++11

HEADERS += \
    Utilities.h \
    can.h \
    driver.h \
    gyro.h \
    main.h \
    motor.h \
    robot.h \
    usart.h

SOURCES += \
    Utilities.cpp \
    can.cpp \
    driver.cpp \
    gyro.cpp \
    main.cpp \
    motor.cpp \
    robot.cpp \
    usart.cpp
