cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl-exam-project)

#SET(CMAKE_CXX_COMPILER g++)
#FIND_PACKAGE(OpenCV REQUIRED)
find_package(PCL 1.4 REQUIRED)
find_package(Qt4 4.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${QT_INCLUDE_DIRS})
include(${QT_USE_FILE})
link_directories(${PCL_LIBRARY_DIRS} ${QT_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${QT_DEFINITIONS})

SET(pcl-exam-project_SOURCES main.cpp mainwindow.cpp registration.cpp)
SET(pcl-exam-project_HEADERS mainwindow.h registration.h ui_mainwindow.h)

QT4_WRAP_CPP(pcl-exam-project_HEADERS_MOC ${qtpcl_HEADERS})

add_executable (pcl-exam-project ${qtpcl_SOURCES} ${qtpcl_HEADERS_MOC})
target_link_libraries (pcl-exam-project ${PCL_LIBRARIES} ${QT_LIBRARIES})
