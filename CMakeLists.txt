cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(qtpcl)

#SET(CMAKE_CXX_COMPILER g++)
#FIND_PACKAGE(OpenCV REQUIRED)
find_package(PCL 1.4 REQUIRED)
find_package(Qt4 4.7 REQUIRED)
find_package(VTK)
include(${VTK_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS} ${QT_INCLUDE_DIRS})
include(${QT_USE_FILE})
link_directories(${PCL_LIBRARY_DIRS} ${QT_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${QT_DEFINITIONS})

SET(qtpcl_SOURCES main.cpp ui.cpp pcqc.cpp registration.cpp componentMatch.cpp componentSelection.cpp component.cpp segmentDifferences.cpp componentcheck.cpp)
SET(qtpcl_HEADERS ui.h pcqc.h registration.h componentMatch.h componentSelection.h component.h segmentDifferences.h componentcheck.h)

QT4_WRAP_CPP(qtpcl_HEADERS_MOC ${qtpcl_HEADERS})

add_executable (qtpcl ${qtpcl_SOURCES} ${qtpcl_HEADERS_MOC})
target_link_libraries (qtpcl ${PCL_LIBRARIES} ${QT_LIBRARIES} QVTK vtkHybrid)
#it was target_link_libraries (qtpcl ${PCL_LIBRARIES} ${QT_LIBRARIES}), works now
