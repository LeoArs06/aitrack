cmake_minimum_required(VERSION 3.16)
project(AITrackHybrid LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(HYBRID_DIR "${CMAKE_CURRENT_LIST_DIR}/../../Client/src/hybrid")
add_library(hybrid STATIC
    ${HYBRID_DIR}/ImuProtocol.cpp
    ${HYBRID_DIR}/ClockSync.cpp
    ${HYBRID_DIR}/PoseConvention.cpp
    ${HYBRID_DIR}/Calibration.cpp
    ${HYBRID_DIR}/HybridFusion.cpp
    ${HYBRID_DIR}/Simulator.cpp)
target_include_directories(hybrid PUBLIC ${HYBRID_DIR})
add_executable(hybrid_tests hybrid_tests.cpp)
target_link_libraries(hybrid_tests PRIVATE hybrid)
if(MSVC)
    target_compile_options(hybrid PRIVATE /W4 /permissive- /EHsc)
    target_compile_options(hybrid_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(hybrid PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(hybrid_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()
enable_testing()
add_test(NAME hybrid_tests COMMAND hybrid_tests)
add_executable(hybrid_replay replay.cpp)
target_link_libraries(hybrid_replay PRIVATE hybrid)
if(MSVC)
    target_compile_options(hybrid_replay PRIVATE /EHsc)
endif()
