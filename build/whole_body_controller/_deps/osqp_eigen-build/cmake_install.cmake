# Install script for directory: /gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/gardo_ws/install/whole_body_controller")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "shlib" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOsqpEigen.so.0.11.0" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOsqpEigen.so.0.11.0")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOsqpEigen.so.0.11.0"
         RPATH "\$ORIGIN/:\$ORIGIN/../lib:/opt/ros/jazzy/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/gardo_ws/build/whole_body_controller/libOsqpEigen.so.0.11.0")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOsqpEigen.so.0.11.0" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOsqpEigen.so.0.11.0")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOsqpEigen.so.0.11.0"
         OLD_RPATH "/opt/ros/jazzy/lib::::::::::::::::::::::::"
         NEW_RPATH "\$ORIGIN/:\$ORIGIN/../lib:/opt/ros/jazzy/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOsqpEigen.so.0.11.0")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "shlib" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/gardo_ws/build/whole_body_controller/libOsqpEigen.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "runtime" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OsqpEigen" TYPE FILE FILES
    "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src/include/OsqpEigen/OsqpEigen.h"
    "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src/include/OsqpEigen/Constants.hpp"
    "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src/include/OsqpEigen/SparseMatrixHelper.hpp"
    "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src/include/OsqpEigen/SparseMatrixHelper.tpp"
    "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src/include/OsqpEigen/Data.hpp"
    "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src/include/OsqpEigen/Data.tpp"
    "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src/include/OsqpEigen/Settings.hpp"
    "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src/include/OsqpEigen/Solver.hpp"
    "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src/include/OsqpEigen/Solver.tpp"
    "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src/include/OsqpEigen/Compat.hpp"
    "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src/include/OsqpEigen/Debug.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "OsqpEigen" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/OsqpEigen" TYPE FILE FILES "/gardo_ws/build/whole_body_controller/OsqpEigenConfigVersion.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "OsqpEigen" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/OsqpEigen" TYPE FILE RENAME "OsqpEigenConfig.cmake" FILES "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-build/CMakeFiles/OsqpEigenConfig.cmake.install")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "OsqpEigen" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/OsqpEigen/OsqpEigenTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/OsqpEigen/OsqpEigenTargets.cmake"
         "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-build/CMakeFiles/Export/4c47d8470c246e918f9ede030c4733c0/OsqpEigenTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/OsqpEigen/OsqpEigenTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/OsqpEigen/OsqpEigenTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/OsqpEigen" TYPE FILE FILES "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-build/CMakeFiles/Export/4c47d8470c246e918f9ede030c4733c0/OsqpEigenTargets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/OsqpEigen" TYPE FILE FILES "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-build/CMakeFiles/Export/4c47d8470c246e918f9ede030c4733c0/OsqpEigenTargets-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cmake/osqp-eigen" TYPE FILE FILES "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-build/tests/cmake_install.cmake")
endif()

