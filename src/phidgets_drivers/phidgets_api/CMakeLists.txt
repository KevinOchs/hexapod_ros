cmake_minimum_required(VERSION 2.8.3)

## From http://www.kitware.com/blog/home/post/63
if(NOT DEFINED PROCESSOR_COUNT)
  # Unknown:
  set(PROCESSOR_COUNT 0)
 
  # Linux:
  set(cpuinfo_file "/proc/cpuinfo")
  if(EXISTS "${cpuinfo_file}")
    file(STRINGS "${cpuinfo_file}" procs REGEX "^processor.: [0-9]+$")
    list(LENGTH procs PROCESSOR_COUNT)
  endif()
 
  # Mac:
  if(APPLE)
    find_program(cmd_sys_pro "system_profiler")
    if(cmd_sys_pro)
      execute_process(COMMAND ${cmd_sys_pro} OUTPUT_VARIABLE info)
      string(REGEX REPLACE "^.*Total Number Of Cores: ([0-9]+).*$" "\\1"
        PROCESSOR_COUNT "${info}")
    endif()
  endif()
endif()

include(ExternalProject)
externalproject_add(libphidget21
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}
  DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
  URL http://www.phidgets.com/downloads/libraries/libphidget_2.1.8.20130723.tar.gz
  URL_MD5 04b4c3a58cf69ecd09431b01471bcd1d
  CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/src/libphidget21/configure CFLAGS=-O3 --disable-jni
--disable-ldconfig 
#--prefix=${CATKIN_DEVEL_PREFIX} --exec-prefix=${CATKIN_DEVEL_PREFIX}
--prefix=${CMAKE_CURRENT_BINARY_DIR} --exec-prefix=${CMAKE_CURRENT_BINARY_DIR}
--libdir=${CATKIN_DEVEL_PREFIX}/lib
#--includedir=${CMAKE_CURRENT_SOURCE_DIR}/include/phidgets_api
--includedir=${CATKIN_DEVEL_PREFIX}/include/libphidget21/
  BUILD_COMMAND make -j${PROCESSOR_COUNT}
  INSTALL_COMMAND make install
)

externalproject_get_property(libphidget21 install_dir)

project(phidgets_api)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidgets_api
)

add_library(phidget21 SHARED IMPORTED)
set_target_properties(phidget21 PROPERTIES IMPORTED_LOCATION "${CATKIN_DEVEL_PREFIX}/lib/libphidget21.so")

# the line below ensures the external project will be built before 
# phidgets_api and any other dependency down the tree
add_dependencies(phidget21 libphidget21)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_api src/phidget.cpp
                         src/imu.cpp
                         src/ir.cpp)

# the line below is not necessary, since the external project has already been built
#add_dependencies(phidgets_api phidget21)
target_link_libraries(phidgets_api ${catkin_LIBRARIES} phidget21)

install(TARGETS phidgets_api
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(FILES
   ${CATKIN_DEVEL_PREFIX}/lib/libphidget21.a
   ${CATKIN_DEVEL_PREFIX}/lib/libphidget21.so
   ${CATKIN_DEVEL_PREFIX}/lib/libphidget21.so.0
   ${CATKIN_DEVEL_PREFIX}/lib/libphidget21.so.0.0.0
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
   ${CMAKE_CURRENT_SOURCE_DIR}/share/setup-udev.sh
   ${install_dir}/src/libphidget21/udev/99-phidgets.rules
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

