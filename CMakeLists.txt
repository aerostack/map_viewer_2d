##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME map_viewer_2d)
project(${PROJECT_NAME} C CXX)

add_definitions(-std=c++11)

##############################################################################
# Catkin
##############################################################################

set(MAPVIEWER2D_CONTROLLER_SOURCE_DIR
        src/controller/map_viewer_2d/source)

set(MAPVIEWER2D_CONTROLLER_INCLUDE_DIR
        src/controller/map_viewer_2d/include)

set(MAPVIEWER2D_CONTROLLER_TEST_DIR
        src/controller/map_viewer_2d/test)

set(MAPVIEWER2D_VIEW_INCLUDE_DIR
        src/view/*/)

set(MAPVIEWER2D_HEADER_FILES
        ${MAPVIEWER2D_CONTROLLER_INCLUDE_DIR}/*.h)

set(MAPVIEWER2D_SOURCE_FILES
        ${MAPVIEWER2D_CONTROLLER_SOURCE_DIR}/*.cpp)

set(MAPVIEWER2D_HEADER_FILES
        ${MAPVIEWER2D_VIEW_INCLUDE_DIR}/*.h)

set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs aerostack_msgs geometry_msgs nav_msgs angles pugixml)
catkin_package(INCLUDE_DIRS ${MAPVIEWER2D_INCLUDE_DIR} CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs DEPENDS aerostack_msgs angles pugixml)

include_directories(${MAPVIEWER2D_INCLUDE_DIR})
include_directories(${MAPVIEWER2D_CONTROLLER_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# check required dependencies
find_package(Boost REQUIRED)
#find_package(ZLIB REQUIRED)
#find_package(Threads REQUIRED)
#find_package(OpenGL REQUIRED)

##############################################################################
# Qwt library
##############################################################################

FIND_PATH(QWT_INCLUDE_DIR NAMES qwt.h PATHS
  /usr/include
  PATH_SUFFIXES qwt
)

MESSAGE(STATUS "qwt header: ${QWT_INCLUDE_DIR}")

find_library(QWT_LIB NAMES qwt-qt5)

MESSAGE(STATUS "Found Qwt: ${QWT_LIB}")

#set(QWT_LIBRARY "/usr/lib/libqwt-qt5.so.6.1.2")

include_directories(${QWT_INCLUDE_DIR})
link_directories(${QWT_LIBRARY})

##############################################################################
# Qt Environment
##############################################################################

# included via the dependency call in package.xml
find_package(Qt5 COMPONENTS Core Gui Svg OpenGL Widgets PrintSupport REQUIRED)
#include(${QT_USE_FILE})

## to avoid conflict with boost signals it is needed to define QT_NO_KEYWORDS.
add_definitions(-DQT_NO_KEYWORDS)
ADD_DEFINITIONS(${QT_DEFINITIONS})

##############################################################################
# Sections
##############################################################################

file(GLOB_RECURSE QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/view/ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/view/resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS  src/controller/*.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/controller/map_viewer_2d/source/*.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fPIC")

##############################################################################
# Binaries
##############################################################################

#MESSAGE(STATUS "Found Qt:  ${QT_LIBRARIES} ")
#add_executable(map_viewer_2d ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_MOC_HPP} ${QT_FORMS_HPP})
#add_dependencies(map_viewer_2d ${catkin_EXPORTED_TARGETS})
#target_link_libraries(map_viewer_2d ${QWT_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES} )
#install(TARGETS map_viewer_2d RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(map_viewer_2d_lib ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_MOC_HPP} ${QT_FORMS_HPP})
add_dependencies(map_viewer_2d_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_viewer_2d_lib ${QWT_LIB} ${catkin_LIBRARIES})
target_link_libraries(map_viewer_2d_lib yaml-cpp)

qt5_use_modules(map_viewer_2d_lib Core Gui Svg OpenGL Widgets PrintSupport)

add_executable(map_viewer_2d ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_MOC_HPP} ${QT_FORMS_HPP})
add_dependencies(map_viewer_2d ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_viewer_2d map_viewer_2d_lib)
target_link_libraries(map_viewer_2d ${QWT_LIB} ${catkin_LIBRARIES})

