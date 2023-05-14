cmake_minimum_required(VERSION 3.24)
project(HinaPE_Common VERSION 2.0.0 DESCRIPTION "HinaPE is not a Physics Engine" LANGUAGES CXX)

option(HINAPE_DOUBLE "use double float precision" ON)
option(HINAPE_UNIT_TEST "build unit test" OFF)

add_library(HinaPE_Common STATIC)
file(GLOB_RECURSE headers common.h animation/*.h geom/*.h math/*.h util/*.h vendor/experimental/*.h vendor/json/*.h vendor/nsearch/*.h vendor/cppoptlib/*.h)
file(GLOB_RECURSE sources animation/*.cpp geom/*.cpp math/*.cpp util/*.cpp vendor/experimental/*.cpp vendor/json/*.cpp vendor/nsearch/*.cpp vendor/cppoptlib/*.cpp)
target_sources(HinaPE_Common
        PRIVATE
        ${sources}

        PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../
        FILES
        ${headers}
        )
set_target_properties(HinaPE_Common PROPERTIES CXX_STANDARD 20 CXX_EXTENSIONS ON)
target_include_directories(HinaPE_Common PUBLIC . vendor vendor/eigen3)
target_compile_definitions(HinaPE_Common PUBLIC HINAPE_DEBUG HINA_EIGEN)
if (HINAPE_DOUBLE)
    target_compile_definitions(HinaPE_Common PUBLIC HINAPE_DOUBLE USE_DOUBLE)
endif ()
if (MSVC)
    target_compile_options(HinaPE_Common PUBLIC /wd4819 /wd4244)
endif ()

# builtin dependency: libigl
add_subdirectory(vendor/igl)
target_link_libraries(HinaPE_Common PUBLIC igl_core)

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

# optional dependency: VDB
#add_subdirectory(vendor/nanovdb)
#target_link_libraries(HinaPE_Common PUBLIC nanovdb)

if (APPLE)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake/OpenVDB)
endif ()
find_package(OpenVDB)
if (OpenVDB_FOUND)
    message(STATUS ">> OpenVDB found")
    target_compile_definitions(HinaPE_Common PUBLIC HINAPE_OPENVDB)
    target_link_libraries(HinaPE_Common PUBLIC OpenVDB::openvdb)
endif ()

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE CACHE BOOL "Disable CGAL build type warning")
find_package(CGAL)
if (CGAL_FOUND)
    message(STATUS ">> CGAL found")
    target_compile_definitions(HinaPE_Common PUBLIC HINAPE_CGAL)
    target_link_libraries(HinaPE_Common PUBLIC CGAL::CGAL)
endif ()

if (UNIX)
    find_package(OpenMP)
    if (OPENMP_FOUND)
        target_link_libraries(HinaPE_Common PUBLIC OpenMP::OpenMP_CXX)
    endif(OPENMP_FOUND)
endif (UNIX)
if (APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    find_package(oneDPL)
    if (oneDPL_FOUND)
        target_link_libraries(HinaPE_Common PUBLIC oneDPL)
    endif ()
endif()

# INSTALL TARGETS
INSTALL(
        TARGETS HinaPE_Common
        CONFIGURATIONS Release
        FILE_SET HEADERS
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
)