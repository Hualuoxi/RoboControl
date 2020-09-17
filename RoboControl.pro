#DEFINES += QT_DEPRECATED_WARNINGS
QT += serialport
QT -= gui
QT += core
QT += network
target.path = /root/robotapp
INSTALLS += target
CONFIG += c++11

HEADERS += \
    Utilities.h \
    can.h \
    clinet.h \
    driver.h \
    gyro.h \
    main.h \
    motor.h \
    robodriver.h \
    robot.h \
    usart.h

SOURCES += \
    Utilities.cpp \
    can.cpp \
    clinet.cpp \
    driver.cpp \
    gyro.cpp \
    main.cpp \
    motor.cpp \
    robodriver.cpp \
    robot.cpp \
    usart.cpp
