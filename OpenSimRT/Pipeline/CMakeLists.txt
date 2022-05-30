set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES} RealTime Common)

# global files
file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
# file(GLOB applications applications/*.cpp)
file(
  GLOB applications
  applications/AccelerationBasedPhaseDetector.cpp
  applications/ContactForceBasedPhaseDetector.cpp
)
# file(GLOB tests tests/*.cpp)

# dependencies
include_directories(include/)
include_directories(../RealTime/include/)
include_directories(../Common/include/)

# dynamic library
#set(target Pipeline)


#addlibrary(
#  KIT ${target}
#  LINKLIBS ${DEPENDENCY_LIBRARIES}
#  INCLUDES ${includes}
#  SOURCES ${sources})

# The genmsg package gives us the code generator macros
#find_package(roscpp REQUIRED)
#https://cmake.org/cmake/help/v3.0/command/include.html
#include(/opt/ros/noetic/share/std_msgs/cmake/std_msgsConfig.cmake)
#https://stackoverflow.com/questions/20746936/what-use-is-find-package-when-you-need-to-specify-cmake-module-path
#catkin_package()
include_directories( ${catkin_INCLUDE_DIRS})

# applications
addapplications(
  SOURCES ${applications}
  LINKLIBS ${target} ${DEPENDENCY_LIBRARIES} ${catkin_LIBRARIES}
)


