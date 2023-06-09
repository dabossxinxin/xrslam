cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-library VERSION 0.4.0 LANGUAGES CXX)

configure_file(cmake/version.h.in include/xrslam/version.h)

set(XRSLAM_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(XRSLAM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(XRSLAM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(XRSLAM_BINARY_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
set(XRSLAM_TEST_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)

set(COMMON_HEADERS
  ${XRSLAM_SOURCE_DIR}/xrslam/common.h
  ${XRSLAM_INCLUDE_DIR}/xrslam/inspection.h
  ${XRSLAM_INCLUDE_DIR}/xrslam/xrslam.h
  ${XRSLAM_BINARY_INCLUDE_DIR}/xrslam/version.h
)
set(COMMON_SOURCES
  ${XRSLAM_SOURCE_DIR}/xrslam/inspection.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/xrslam.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/config.cpp
)
source_group("common" FILES ${COMMON_HEADERS} ${COMMON_SOURCES})

file(GLOB CORE_HEADERS "${XRSLAM_SOURCE_DIR}/xrslam/core/*.h")
file(GLOB CORE_SOURCES "${XRSLAM_SOURCE_DIR}/xrslam/core/*.cpp")
source_group("core" FILES ${CORE_HEADERS} ${CORE_SOURCES})

file(GLOB ESTIMATION_HEADERS "${XRSLAM_SOURCE_DIR}/xrslam/estimation/*.h")
file(GLOB ESTIMATION_SOURCES "${XRSLAM_SOURCE_DIR}/xrslam/estimation/*.cpp")
source_group("estimation" FILES ${ESTIMATION_HEADERS} ${ESTIMATION_SOURCES})

file(GLOB GEOMETRY_HEADERS "${XRSLAM_SOURCE_DIR}/xrslam/geometry/*.h")
file(GLOB GEOMETRY_SOURCES "${XRSLAM_SOURCE_DIR}/xrslam/geometry/*.cpp")
source_group("geometry" FILES ${GEOMETRY_HEADERS} ${GEOMETRY_SOURCES})

file(GLOB LOCALIZER_HEADERS "${XRSLAM_SOURCE_DIR}/xrslam/localizer/*.h")
file(GLOB LOCALIZER_SOURCES "${XRSLAM_SOURCE_DIR}/xrslam/localizer/*.cpp")
source_group("localizer" FILES ${LOCALIZER_HEADERS} ${LOCALIZER_SOURCES})

file(GLOB MAP_HEADERS "${XRSLAM_SOURCE_DIR}/xrslam/map/*.h")
file(GLOB MAP_SOURCES "${XRSLAM_SOURCE_DIR}/xrslam/map/*.cpp")
source_group("map" FILES ${MAP_HEADERS} ${MAP_SOURCES})

file(GLOB UTILITY_HEADERS "${XRSLAM_SOURCE_DIR}/xrslam/utility/*.h")
file(GLOB UTILITY_SOURCES "${XRSLAM_SOURCE_DIR}/xrslam/utility/*.cpp")
source_group("utility" FILES ${UTILITY_HEADERS} ${UTILITY_SOURCES})

file(GLOB AR_HEADERS "${XRSLAM_SOURCE_DIR}/xrslam/ar/*.h")
file(GLOB AR_SOURCES "${XRSLAM_SOURCE_DIR}/xrslam/ar/*.cpp")
source_group("ar" FILES ${AR_HEADERS} ${AR_SOURCES})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /bigobj")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")

add_library(xrslam
  ${COMMON_HEADERS}
  ${COMMON_SOURCES}
  ${CORE_HEADERS}
  ${CORE_SOURCES}
  ${ESTIMATION_HEADERS}
  ${ESTIMATION_SOURCES}
  ${GEOMETRY_HEADERS}
  ${GEOMETRY_SOURCES}
  ${LOCALIZER_HEADERS}
  ${LOCALIZER_SOURCES}
  ${MAP_HEADERS} 
  ${MAP_SOURCES}
  ${UTILITY_HEADERS} 
  ${UTILITY_SOURCES}
  ${AR_HEADERS} 
  ${AR_SOURCES}
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(xrslam
    PRIVATE
      -Qunused-arguments
      -mllvm
      -inline-threshold=5000
  )
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  target_compile_definitions(xrslam
    PUBLIC
      _ENABLE_EXTENDED_ALIGNED_STORAGE
    PRIVATE
      _USE_MATH_DEFINES
      _CRT_SECURE_NO_WARNINGS
  )
endif()

target_include_directories(xrslam
  PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<BUILD_INTERFACE:${XRSLAM_INCLUDE_DIR}>
    ${XRSLAM_BINARY_INCLUDE_DIR}
  PRIVATE
    ${XRSLAM_SOURCE_DIR}
)

target_link_libraries(xrslam
PUBLIC
  options::modern-cpp
  XRPrimer::xrprimer
  ${OpenCV_LIBS}
)

set(XRSLAM_LIBRARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
