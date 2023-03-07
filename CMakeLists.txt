cmake_minimum_required(VERSION 3.18)
project(HinaPE_Common VERSION 2.0.0 DESCRIPTION "HinaPE is not a Physics Engine" LANGUAGES CXX)

option(HINAPE_DOUBLE "use double float precision" ON)
option(HINAPE_UNIT_TEST "build unit test" ON)

if (MSVC)
    add_compile_options(/wd4819 /wd4244)
endif ()

file(GLOB_RECURSE
        src
        "common.h"
        "animation/*.cpp" "animation/*.h"
        "geom/*.cpp" "geom/*.h"
        "math/*.cpp" "math/*.h"
        "util/*.cpp" "util/*.h"
        "vendor/igl/*.cpp" "vendor/igl/*.h" # builtin dependency: igl
        "vendor/json/*.cpp" "vendor/json/*.h" "vendor/json/*.hpp" # builtin dependency: json
        "vendor/nsearch/*.cpp" "vendor/nsearch/*.h" # builtin dependency: nsearch
        )

add_library(HinaPE_Common STATIC ${src})
set_target_properties(HinaPE_Common PROPERTIES CXX_STANDARD 20 CXX_EXTENSIONS ON)
target_include_directories(HinaPE_Common PUBLIC . vendor vendor/eigen3)
target_compile_definitions(HinaPE_Common PUBLIC HINAPE_DEBUG HINA_EIGEN)
if (HINAPE_DOUBLE)
    target_compile_definitions(HinaPE_Common PUBLIC HINAPE_DOUBLE USE_DOUBLE)
endif ()

# builtin dependency: google test
if (HINAPE_UNIT_TEST)
    add_subdirectory(vendor/googletest)
    add_subdirectory(_unit_test)
endif ()

# optional dependency: TBB
if (APPLE)
    set(TBB_DIR /opt/homebrew/Cellar/tbb@2020/2020_U3/lib/cmake/TBB)
endif ()
find_package(TBB CONFIG)
if (TBB_FOUND)
    message(STATUS ">> TBB found")
    target_compile_definitions(HinaPE_Common PUBLIC HINAPE_TBB)
    target_link_libraries(HinaPE_Common PUBLIC TBB::tbb)
endif ()

# optional dependency:
find_package(OpenVDB CONFIG)
if (OpenVDB_FOUND)
    message(STATUS ">> OpenVDB found")
    target_compile_definitions(HinaPE_Common PUBLIC HINAPE_OPENVDB)
    target_link_libraries(HinaPE_Common PUBLIC OpenVDB::openvdb)
endif ()

if (UNIX)
    find_package(OpenMP)
    if (OPENMP_FOUND)
        target_link_libraries(HinaPE_Common PUBLIC OpenMP::OpenMP_CXX)
    endif(OPENMP_FOUND)
endif (UNIX)
if (APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    find_package(oneDPL REQUIRED)
    target_link_libraries(HinaPE_Common PUBLIC oneDPL)
endif()