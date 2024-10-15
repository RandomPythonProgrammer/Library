cmake_minimum_required(VERSION 3.18)

project(RoboOp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

set(CXX_STANDARD 20)
set(CXX_STANDARD_REQUIRED TRUE)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_library(RoboOp SHARED ${SOURCES})
target_include_directories(RoboOp PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include> PRIVATE src)

include(GNUInstallDirs)

install(TARGETS RoboOp
    EXPORT RoboOpConfig
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/RoboOp)
install(EXPORT RoboOpConfig DESTINATION share/RoboOp/cmake)
export(TARGETS RoboOp FILE RoboOpConfig.cmake)

file(GLOB_RECURSE TEST_SOURCES "test_src/*.cpp" "src/*.cpp")
add_executable(RoboOpTest ${TEST_SOURCES})
target_include_directories(RoboOpTest PRIVATE include)
target_include_directories(RoboOpTest PRIVATE test_include)