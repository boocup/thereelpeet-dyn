cmake_minimum_required(VERSION 3.19)


if(NOT "${METAMODULE_SDK_DIR}" STREQUAL "")
    message("METAMODULE_SDK_DIR set by CMake variable ${METAMODULE_SDK_DIR}")
elseif(DEFINED ENV{METAMODULE_SDK_DIR})
    set(METAMODULE_SDK_DIR "$ENV{METAMODULE_SDK_DIR}")
    message("METAMODULE_SDK_DIR set by environment variable ${METAMODULE_SDK_DIR}")
else()
    set(METAMODULE_PLUGIN_STATIC_LIBC ON CACHE BOOL
        "Use static/prebuilt libc for MetaModule plugins" FORCE)
    set(METAMODULE_SDK_BUILD_PLUGIN_LIBC OFF CACHE BOOL
        "Build plugin-libc from source" FORCE)
    set(METAMODULE_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../MyModule-metamodule/sdk")
    message("METAMODULE_SDK_DIR set to default: ${METAMODULE_SDK_DIR}")
endif()

include(${METAMODULE_SDK_DIR}/plugin.cmake)

project(TheReelPeetSeqMM LANGUAGES C CXX ASM)

add_library(thereelpeetseqmm STATIC
    ${CMAKE_CURRENT_LIST_DIR}/src/plugin.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/TheReelPeet.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/TheReelPeetEvo.cpp
)

target_include_directories(thereelpeetseqmm PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}
)

# MetaModule renders PNG only, never SVG, at runtime. Per the plugin SDK's
# own docs (docs/graphics.md): the MetaModule Rack adaptor rewrites any
# "res/X.svg" load call to "X.png" and looks for it in the plugin's bundled
# assets — so this build's SOURCE_ASSETS points at assets/ (PNGs, generated
# via scripts/SvgToPng.py from res/), not res/ itself. The VCV build
# (Makefile) still uses res/ directly, since desktop Rack does render SVG.
create_plugin(
    SOURCE_LIB      thereelpeetseqmm
    SOURCE_ASSETS   ${CMAKE_CURRENT_LIST_DIR}/assets
    DESTINATION     ${CMAKE_CURRENT_BINARY_DIR}/out
    PLUGIN_NAME     thereelpeet-seq
    PLUGIN_JSON     ${CMAKE_CURRENT_LIST_DIR}/plugin.json
)
