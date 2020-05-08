# required for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++14)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Warnings
add_definitions(-Wall -Wextra -Wpedantic -Wredundant-decls -Wshadow -Wcast-qual -Wfloat-equal)

# unsafe-math-optimizations should normally not be enabled by optimization options except by ffast-math
# still putting this here to make sure
add_definitions(-fno-unsafe-math-optimizations)
