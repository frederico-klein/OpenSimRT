set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES} RealTime Common)

# global files
file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
# dependencies
include_directories(include/)
include_directories(../RealTime/include/)
include_directories(../Common/include/)

# dynamic library
set(target Ros)


addlibrary(
  KIT ${target}
  LINKLIBS ${DEPENDENCY_LIBRARIES}
  INCLUDES ${includes}
  SOURCES ${sources})

include_directories( ${catkin_INCLUDE_DIRS})


