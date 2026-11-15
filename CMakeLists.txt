cmake_minimum_required(VERSION 3.24)
project(sldprt_glb VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(SWGLB_WITH_OCCT "Regenerate stale display meshes from B-rep with OpenCASCADE" ON)
option(SWGLB_BUILD_TESTS "Build unit tests" ON)

find_package(ZLIB REQUIRED)

include(FetchContent)
# openswx (MIT): only libopenparasolid is used, for Parasolid B-rep decoding.
set(SWGLB_OPENSWX_REV 72291dc962bbf267fa40cd57813b273108ebac53 CACHE STRING "openswx revision")
FetchContent_Declare(openswx
  GIT_REPOSITORY https://github.com/schwitters/openswx.git
  GIT_TAG ${SWGLB_OPENSWX_REV}
  SOURCE_SUBDIR libopenparasolid)
set(OPENPARASOLID_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(openswx)

add_library(swglb STATIC
  src/bytes.cc
  src/container.cc
  src/engine.cc
  src/model_json.cc
  src/display_lists.cc
  src/appearance.cc
  src/brep.cc
  src/brep_config.cc
  src/brep_fit.cc
  src/brep_blend.cc
  src/brep_site.cc
  src/brep_streams.cc
  src/brep_visual.cc
  src/brep_xt.cc
  src/ownership.cc
  src/ownership_geometry.cc
  src/presentation.cc
  src/assembly.cc
  src/glb.cc
  src/json_value.cc
  src/json_parse.cc
  src/cadir_load.cc
  src/regenerate.cc
)
target_include_directories(swglb PUBLIC src)
target_link_libraries(swglb PUBLIC ZLIB::ZLIB openparasolid)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(swglb PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-maybe-uninitialized)
endif()

if(SWGLB_WITH_OCCT)
  find_package(OpenCASCADE REQUIRED)
  target_compile_definitions(swglb PRIVATE SWGLB_HAVE_OCCT)
  target_include_directories(swglb SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
  target_link_libraries(swglb PRIVATE
    TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep TKGeomAlgo TKTopAlgo
    TKShHealing TKMesh TKLCAF TKCAF TKXCAF TKRWMesh)
  # OCCT 7.8 moved RWGltf_CafWriter into TKDEGLTF; 7.6/7.7 keep it in TKRWMesh.
  if(TARGET TKDEGLTF)
    target_link_libraries(swglb PRIVATE TKDEGLTF)
  endif()
endif()

add_executable(sldprt-glb src/main.cc)
target_link_libraries(sldprt-glb PRIVATE swglb)

if(SWGLB_BUILD_TESTS)
  enable_testing()
  file(GLOB SWGLB_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*_test.cc)
  foreach(test_source IN LISTS SWGLB_TEST_SOURCES)
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(${test_name} ${test_source})
    target_link_libraries(${test_name} PRIVATE swglb)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()

install(TARGETS sldprt-glb RUNTIME DESTINATION bin)
