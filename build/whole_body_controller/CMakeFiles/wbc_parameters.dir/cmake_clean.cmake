file(REMOVE_RECURSE
  "include/wbc_parameters.hpp"
  "include/whole_body_controller/wbc_parameters.hpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/wbc_parameters.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
