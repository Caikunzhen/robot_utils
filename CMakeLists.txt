cmake_minimum_required(VERSION 3.5.0)
project(robot_utils)

add_compile_options(-std=c++2a)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  if (NOT DEFINED USE_RU_ASSERT)
    set(USE_RU_ASSERT 1)
  endif()
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Release")
  if (NOT DEFINED USE_RU_ASSERT)
    set(USE_RU_ASSERT 0)
  endif()
endif()
add_definitions(-DUSE_RU_ASSERT=${USE_RU_ASSERT})


find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(qpOASES QUIET)
find_package(NLopt QUIET)

file(GLOB_RECURSE ROBOT_UTILS_INCLUDE
  include/*.hpp
)
file(GLOB_RECURSE ROBOT_UTILS_SRC
  src/*.cpp
)

add_library(${PROJECT_NAME}
  ${ROBOT_UTILS_INCLUDE}
  ${ROBOT_UTILS_SRC}
)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    yaml-cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

target_compile_definitions(
  ${PROJECT_NAME}
  PUBLIC
    USE_RU_ASSERT=${USE_RU_ASSERT}
)

target_compile_options(
  ${PROJECT_NAME}
  PUBLIC
    -Wno-sign-compare
    -Wno-reorder
)

if (qpOASES_FOUND)
  target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
      qpOASES
  )
  target_compile_definitions(
    ${PROJECT_NAME}
    PUBLIC
      HAS_QPOASES
  )
endif()

if (NLopt_FOUND)
  target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
      NLopt::nlopt
  )
  target_compile_definitions(
    ${PROJECT_NAME}
    PUBLIC
      HAS_NLOPT
  )
endif()