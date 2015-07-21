cmake_minimum_required(VERSION 2.8)

# add prefix to every string in a variable
FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

project(New_Map_Test)

find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(UNIX)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set include and lib folders
if(UNIX)
include_directories("/usr/lapack/include")
include_directories("/usr/Aris/include")
include_directories("/usr/xenomai/include")
include_directories("/opt/etherlab/include")
include_directories("/usr/Robots/include")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(/usr/lapack/lib)
link_directories(/usr/Aris/lib)
link_directories(/usr/xenomai/lib)
link_directories(/opt/etherlab/lib)
link_directories(/usr/Robots/lib)
link_directories(${PCL_LIBRARY_DIRS})
endif(UNIX)
if(WIN32)
include_directories("C:/include/Lapack")
include_directories("C:/include/Aris")
endif(WIN32)

# set link files
set(SYSTEM_LINK_LIBS pthread gfortran)
set(XENOMAI_LINK_LIBS native rtdm xenomai)
set(ETHERLAB_LINK_LIBS ethercat rt)
set(LAPACK_LINK_LIBS lapacke lapack cblas refblas)
set(ARIS_LINK_LIBS Aris_Plan Aris_DynKer Aris_ExpCal Aris_Control Aris_Core)

set(ROBOTS_LINK_LIBS  HexapodIII)

# build programs Vision Clent
set(SOURCE_FILES Kinect_main.cpp Kinect.cpp)
add_executable(Vision_Client ${SOURCE_FILES})
target_link_libraries(Vision_Client ${ARIS_LINK_LIBS}  ${SYSTEM_LINK_LIBS} ${PCL_LIBRARIES})


