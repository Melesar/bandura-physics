cmake_minimum_required(VERSION 3.25)

project(bandura VERSION 0.1.0 LANGUAGES C)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(GNUInstallDirs)

option(BANDURA_BUILD_SHARED "Build Bandura as a shared library" OFF)
option(BANDURA_BUILD_DEMOS "Build Bandura demos" OFF)

set(BANDURA_LIBRARY_TYPE STATIC)
if(BANDURA_BUILD_SHARED)
  set(BANDURA_LIBRARY_TYPE SHARED)
endif()

file(GLOB BANDURA_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

add_library(bandura ${BANDURA_LIBRARY_TYPE} ${BANDURA_SOURCES})
add_library(Bandura::bandura ALIAS bandura)

target_compile_features(bandura PUBLIC c_std_99)
target_include_directories(bandura
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_options(bandura PRIVATE
  -Wall
  -Wextra
  -Werror=format
  -Werror=shadow
  -Werror=incompatible-pointer-types
  -Werror=pointer-type-mismatch
  -Werror=return-type
  -Wno-unused-parameter
)
target_compile_definitions(bandura PRIVATE
  $<$<CONFIG:Debug>:BND_DEBUG>
  $<$<BOOL:${BANDURA_BUILD_SHARED}>:BND_BUILD_DLL>
)
target_compile_definitions(bandura PUBLIC
  $<$<NOT:$<BOOL:${BANDURA_BUILD_SHARED}>>:BND_STATIC>
)

install(TARGETS bandura
  EXPORT BanduraTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/bandura.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/bnd-math.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(EXPORT BanduraTargets
  NAMESPACE Bandura::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Bandura
)

if(BANDURA_BUILD_DEMOS)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/raylib/CMakeLists.txt")
    message(FATAL_ERROR "Demo build requires the raylib submodule. Run: git submodule update --init")
  endif()

  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/clay/clay.h")
    message(FATAL_ERROR "Demo build requires the clay submodule. Run: git submodule update --init")
  endif()

  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/raygui/src/raygui.h")
    message(FATAL_ERROR "Demo build requires the raygui submodule. Run: git submodule update --init")
  endif()

  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

  add_subdirectory(vendor/raylib)

  file(GLOB BANDURA_DEMO_COMMON_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/demos/*.c"
  )
  file(GLOB BANDURA_DEMO_SCENARIOS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/demos/scenarios/*.c"
  )

  foreach(scenario_source IN LISTS BANDURA_DEMO_SCENARIOS)
    get_filename_component(scenario_name "${scenario_source}" NAME_WE)

    add_executable(${scenario_name}
      ${BANDURA_DEMO_COMMON_SOURCES}
      ${scenario_source}
    )

    target_compile_features(${scenario_name} PRIVATE c_std_99)
    target_include_directories(${scenario_name} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/demos/include
      ${CMAKE_CURRENT_SOURCE_DIR}/vendor/clay
      ${CMAKE_CURRENT_SOURCE_DIR}/vendor/raygui/src
      ${CMAKE_CURRENT_SOURCE_DIR}/vendor/raygui/styles
      ${CMAKE_CURRENT_SOURCE_DIR}/vendor/raylib/src
      ${CMAKE_CURRENT_SOURCE_DIR}/vendor/raylib/examples
    )
    target_compile_options(${scenario_name} PRIVATE
      -Wall
      -Wextra
      -Werror=format
      -Werror=shadow
      -Werror=incompatible-pointer-types
      -Werror=pointer-type-mismatch
      -Werror=return-type
      -Wno-unused-parameter
    )
    target_link_libraries(${scenario_name} PRIVATE bandura raylib)

    install(TARGETS ${scenario_name}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
  endforeach()
endif()
