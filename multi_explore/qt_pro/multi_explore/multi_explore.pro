TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    ../../src/costmap_client.cpp \
    ../../src/explore.cpp \
    ../../src/frontier_search.cpp

INCLUDEPATH += \
				/opt/ros/kinetic/include\
				/opt/ros/kinetic/include/eigen3\
				../mavros_ws/devel/.private/mav_msgs/include\
				/usr/include/eigen3\
				/home/jbs/mavros_ws/devel/include\

HEADERS += \
    ../../include/multi_explore/costmap_client.h \
    ../../include/multi_explore/costmap_tools.h \
    ../../include/multi_explore/explore.h \
    ../../include/multi_explore/frontier_search.h
