# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/gardo_ws/build/quadruped_estimators/_deps/inekf-src"
  "/gardo_ws/build/quadruped_estimators/_deps/inekf-build"
  "/gardo_ws/build/quadruped_estimators/_deps/inekf-subbuild/inekf-populate-prefix"
  "/gardo_ws/build/quadruped_estimators/_deps/inekf-subbuild/inekf-populate-prefix/tmp"
  "/gardo_ws/build/quadruped_estimators/_deps/inekf-subbuild/inekf-populate-prefix/src/inekf-populate-stamp"
  "/gardo_ws/build/quadruped_estimators/_deps/inekf-subbuild/inekf-populate-prefix/src"
  "/gardo_ws/build/quadruped_estimators/_deps/inekf-subbuild/inekf-populate-prefix/src/inekf-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/gardo_ws/build/quadruped_estimators/_deps/inekf-subbuild/inekf-populate-prefix/src/inekf-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/gardo_ws/build/quadruped_estimators/_deps/inekf-subbuild/inekf-populate-prefix/src/inekf-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
