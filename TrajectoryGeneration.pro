QT += charts

HEADERS += \
    TrajectoryView.h \
    callout.h \
    mainwindow.h \
    third_party/json/json.hpp \
    third_party/ruckig/block.hpp \
    third_party/ruckig/brake.hpp \
    third_party/ruckig/calculator.hpp \
    third_party/ruckig/calculator_cloud.hpp \
    third_party/ruckig/calculator_target.hpp \
    third_party/ruckig/error.hpp \
    third_party/ruckig/input_parameter.hpp \
    third_party/ruckig/output_parameter.hpp \
    third_party/ruckig/position.hpp \
    third_party/ruckig/profile.hpp \
    third_party/ruckig/result.hpp \
    third_party/ruckig/roots.hpp \
    third_party/ruckig/ruckig.hpp \
    third_party/ruckig/trajectory.hpp \
    third_party/ruckig/utils.hpp \
    third_party/ruckig/velocity.hpp

SOURCES += \
    TrajectoryView.cpp \
    callout.cpp \
    main.cpp\
    mainwindow.cpp \
    third_party/ruckig/brake.cpp \
    third_party/ruckig/cloud_client.cpp \
    third_party/ruckig/position_first_step1.cpp \
    third_party/ruckig/position_first_step2.cpp \
    third_party/ruckig/position_second_step1.cpp \
    third_party/ruckig/position_second_step2.cpp \
    third_party/ruckig/position_third_step1.cpp \
    third_party/ruckig/position_third_step2.cpp \
    third_party/ruckig/velocity_second_step1.cpp \
    third_party/ruckig/velocity_second_step2.cpp \
    third_party/ruckig/velocity_third_step1.cpp \
    third_party/ruckig/velocity_third_step2.cpp

INCLUDEPATH += third_party third_party\ruckig\

target.path = $$[QT_INSTALL_EXAMPLES]/charts/callout
INSTALLS += target

FORMS += \
    mainwindow.ui

DISTFILES += \
    third_party/json/LICENSE

RESOURCES += \
    resource/resource.qrc

RC_FILE = resource/logo.rc
