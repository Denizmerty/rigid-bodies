# The packages the application layer needs, built from the copies in vendor/ that the native Visual
# Studio solution also uses. Both builds therefore link the same SDL3 and compile the same FreeType
# and RmlUi sources with the same options. Only the application modules use anything here: the
# simulation and mathematics modules configure and build with this file never included.

include(FetchContent)

set(RIGIDBODIES_SDL_VERSION "3.4.16" CACHE STRING "SDL3 release used when the package is fetched")
option(RIGIDBODIES_FETCH_DEPENDENCIES "Fetch dependencies vendor/ does not supply for this platform" ON)

set(RIGIDBODIES_VENDOR_DIR "${PROJECT_SOURCE_DIR}/vendor")

# SDL3 supplies the window, the input devices, the timer, and the platform abstraction, and its
# renderer backs the two-dimensional device implementation. vendor/SDL3 holds the Windows x64
# runtime and import library; other platforms find or fetch the same release.
set(RIGIDBODIES_VENDOR_SDL "${RIGIDBODIES_VENDOR_DIR}/SDL3")
if(WIN32 AND CMAKE_CXX_COMPILER_ARCHITECTURE_ID STREQUAL "x64" AND EXISTS "${RIGIDBODIES_VENDOR_SDL}/lib/x64/SDL3.lib")
    add_library(SDL3::SDL3 SHARED IMPORTED GLOBAL)
    set_target_properties(
        SDL3::SDL3
        PROPERTIES
        IMPORTED_IMPLIB "${RIGIDBODIES_VENDOR_SDL}/lib/x64/SDL3.lib"
        IMPORTED_LOCATION "${RIGIDBODIES_VENDOR_SDL}/bin/x64/SDL3.dll"
        INTERFACE_INCLUDE_DIRECTORIES "${RIGIDBODIES_VENDOR_SDL}/include"
    )
    set(RIGIDBODIES_SDL_RUNTIME "${RIGIDBODIES_VENDOR_SDL}/bin/x64/SDL3.dll")
elseif(RIGIDBODIES_FETCH_DEPENDENCIES)
    set(SDL_SHARED ON CACHE BOOL "" FORCE)
    set(SDL_STATIC OFF CACHE BOOL "" FORCE)
    set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
    set(SDL_TESTS OFF CACHE BOOL "" FORCE)
    set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(SDL_INSTALL OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(
        SDL3
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG release-${RIGIDBODIES_SDL_VERSION}
        GIT_SHALLOW TRUE
        FIND_PACKAGE_ARGS 3.4 CONFIG NAMES SDL3
    )
    FetchContent_MakeAvailable(SDL3)
else()
    find_package(SDL3 3.4 CONFIG REQUIRED)
endif()

# FreeType 2.13.3 supplies the shared scene/fallback font atlas as well as RmlUi's font engine. Its
# stock configuration headers apply: no system zlib, bzip2, PNG, HarfBuzz or Brotli.
enable_language(C)
set(RIGIDBODIES_VENDOR_FREETYPE "${RIGIDBODIES_VENDOR_DIR}/FreeType")
rigidbodies_project_files(rigidbodies_freetype_sources thirdparty/FreeType.vcxproj ClCompile)
if(NOT WIN32)
    # The Windows system and debug modules are the only platform-specific FreeType sources.
    list(TRANSFORM rigidbodies_freetype_sources REPLACE "/builds/windows/(ftsystem|ftdebug)\\.c$" "/src/base/\\1.c")
endif()
add_library(freetype STATIC ${rigidbodies_freetype_sources})
target_include_directories(
    freetype
    PUBLIC ${RIGIDBODIES_VENDOR_FREETYPE}/include
    PRIVATE ${RIGIDBODIES_VENDOR_FREETYPE}/include/freetype/config
)
target_compile_definitions(freetype PRIVATE FT2_BUILD_LIBRARY _CRT_NONSTDC_NO_WARNINGS _CRT_SECURE_NO_WARNINGS)
# Third-party code: its warnings are not ours to fix, and they do not change the generated code.
target_compile_options(freetype PRIVATE $<$<C_COMPILER_ID:MSVC>:/W0> $<$<NOT:$<C_COMPILER_ID:MSVC>>:-w>)
add_library(Freetype::Freetype ALIAS freetype)

# RmlUi 6.1 Core is the default document interface; the proportional overlay remains available
# without it. Every source includes its precompiled header first, as RmlUi's own build arranges.
if(RIGIDBODIES_ENABLE_RMLUI)
    set(RIGIDBODIES_VENDOR_RMLUI "${RIGIDBODIES_VENDOR_DIR}/RmlUi")
    rigidbodies_project_files(rigidbodies_rmlui_sources thirdparty/RmlUi.vcxproj ClCompile)
    # The Visual Studio project creates the precompiled header from its own one-line source.
    list(FILTER rigidbodies_rmlui_sources EXCLUDE REGEX "/rmlui_precompiled\\.cpp$")
    add_library(rmlui STATIC ${rigidbodies_rmlui_sources})
    target_compile_features(rmlui PUBLIC cxx_std_17)
    set_target_properties(rmlui PROPERTIES CXX_EXTENSIONS OFF)
    target_include_directories(rmlui PUBLIC ${RIGIDBODIES_VENDOR_RMLUI}/Include)
    target_compile_definitions(
        rmlui
        PUBLIC RMLUI_STATIC_LIB
        PRIVATE RMLUI_FONT_ENGINE_FREETYPE RMLUI_VERSION="6.1" _CRT_SECURE_NO_WARNINGS
    )
    target_compile_options(rmlui PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/W0;/permissive-> $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-w>)
    target_precompile_headers(rmlui PRIVATE ${RIGIDBODIES_VENDOR_RMLUI}/Source/Core/precompiled.h)
    target_link_libraries(rmlui PRIVATE freetype)
    add_library(RmlUi::Core ALIAS rmlui)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/developer_dependencies.cmake OPTIONAL)
