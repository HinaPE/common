cmake_minimum_required(VERSION 3.16)
project(HinaPE_Common VERSION 1.1.0 DESCRIPTION "HinaPE is not a Physics Engine" LANGUAGES CXX)

file(GLOB
        HINAPE_COMMON_SRC
        "*.cpp" "*.h" "*.inl.h"
        "animation/*.cpp" "animation/*.h" "animation/*.inl.h"
        "array/*.cpp" "array/*.h" "array/*.inl.h"
        "field/*.cpp" "field/*.h" "field/*.inl.h"
        "flatbuffers/*.cpp" "flatbuffers/*.h" "flatbuffers/*.inl.h"
        "generated/*.cpp" "generated/*.h" "generated/*.inl.h"
        "geometry/*.cpp" "geometry/*.h" "geometry/*.inl.h"
        "grid/*.cpp" "grid/*.h" "grid/*.inl.h"
        "math/*.cpp" "math/*.h" "math/*.inl.h"
        "query_engine/*.cpp" "query_engine/*.h" "query_engine/*.inl.h"
        )

add_library(HinaPE_Common STATIC ${HINAPE_COMMON_SRC} field/custom_vector_field3.h field/custom_vector_field3.cpp array/cubic_semi_lagrangian3.h array/semi_lagrangian3.h array/semi_lagrangian3.cpp array/cubic_semi_lagrangian3.cpp geometry/cylinder3.h geometry/cylinder3.cpp)
set_target_properties(HinaPE_Common PROPERTIES CXX_STANDARD 20 CXX_EXTENSIONS OFF)
target_include_directories(HinaPE_Common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# add dependencies
add_subdirectory(deps/tinyobjloader)
target_link_libraries(HinaPE_Common PUBLIC tinyobjloader)
add_subdirectory(deps/fpm)
target_link_libraries(HinaPE_Common PUBLIC fpm)

set(TBB_DIR /opt/homebrew/Cellar/tbb@2020/2020_U3/lib/cmake/TBB)
find_package(TBB CONFIG)
if (TBB_FOUND)
    message(STATUS ">> TBB found")
    set(HINAPE_TBB TRUE CACHED BOOL "TBB is found")
    target_compile_definitions(HinaPE_Common PUBLIC JET_TASKING_TBB)
    target_link_libraries(HinaPE_Common PUBLIC TBB::tbb)
endif ()
