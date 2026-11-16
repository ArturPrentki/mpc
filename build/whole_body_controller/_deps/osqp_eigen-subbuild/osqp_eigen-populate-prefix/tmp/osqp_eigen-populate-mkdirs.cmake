# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-src"
  "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-build"
  "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-subbuild/osqp_eigen-populate-prefix"
  "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-subbuild/osqp_eigen-populate-prefix/tmp"
  "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-subbuild/osqp_eigen-populate-prefix/src/osqp_eigen-populate-stamp"
  "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-subbuild/osqp_eigen-populate-prefix/src"
  "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-subbuild/osqp_eigen-populate-prefix/src/osqp_eigen-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-subbuild/osqp_eigen-populate-prefix/src/osqp_eigen-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/gardo_ws/build/whole_body_controller/_deps/osqp_eigen-subbuild/osqp_eigen-populate-prefix/src/osqp_eigen-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
