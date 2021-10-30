#-------------------------------------------------
#
# Project created by QtCreator 2021-07-22T10:51:28
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Apollo_control_algorithm
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

CONFIG += C++11
INCLUDEPATH += /usr/local/include

LIBS += -L/usr/local/lib -lglog
LIBS += -L/usr/local/lib -lqpOASES
SOURCES += \
        control/control.cpp \
        control/controller_factory.cpp \
        control/lat_controller.cpp \
        control/linear_quadratic_regulator.cc \
        control/lon_controller.cpp \
        control/mpc_controller.cpp \
        control/mpc_solver.cc \
        control/pid_controller.cc \
        control/pure_pursuit_controller.cpp \
        filters/digital_filter.cc \
        filters/digital_filter_coefficients.cc \
        filters/kalman_filter.cc \
        filters/mean_filter.cc \
        interpolation_1d/interpolatin_1d.cc \
        location/coorconv.cc \
        main.cpp \
        src/active_set_qp_solver.cc \
        src/qp_solver.cc \
        trajectory/trajectory.cpp \
        vehicle_info/vehicle_params.cpp \
        vehicle_info/vehicle_state.cc

HEADERS += \
        base_types.h \
        common/base_types.h \
        common/control_conf.h \
        common/debug.h \
        common/log.h \
        control/control.h \
        control/controller.h \
        control/controller_factory.h \
        control/lat_controller.h \
        control/linear_quadratic_regulator.h \
        control/lon_controller.h \
        control/mpc_controller.h \
        control/mpc_solver.h \
        control/pid_controller.h \
        control/pure_pursuit_controller.h \
        control_conf.h \
        filters/digital_filter.h \
        filters/digital_filter_coefficients.h \
        filters/kalman_filter.h \
        filters/mean_filter.h \
        include/active_set_qp_solver.h \
        include/qp_solver.h \
        interpolation_1d/interpolation_1d.h \
        location/coorconv.h \
        trajectory/trajectory.h \
        vehicle_info/vehicle_params.h \
        vehicle_info/vehicle_state.h



# Default rules for deployment.
#qnx: target.path = /tmp/$${TARGET}/bin
#else: unix:!android: target.path = /opt/$${TARGET}/bin
#!isEmpty(target.path): INSTALLS += target
