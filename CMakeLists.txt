cmake_minimum_required(VERSION 3.24)

# The version is written once, in the project identity header that the code, the Windows resource
# script and the Visual Studio solution read.
file(
    STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/src/RigidBodies.Core/include/rigidbodies/project_identity.hpp"
    rigidbodies_version_lines
    REGEX "^#define RIGIDBODIES_VERSION_(MAJOR|MINOR|PATCH) [0-9]+$"
)
foreach(rigidbodies_version_line IN LISTS rigidbodies_version_lines)
    string(REGEX MATCH "(MAJOR|MINOR|PATCH) ([0-9]+)$" rigidbodies_version_match "${rigidbodies_version_line}")
    set(rigidbodies_version_${CMAKE_MATCH_1} ${CMAKE_MATCH_2})
endforeach()

project(
    RigidBodies
    VERSION ${rigidbodies_version_MAJOR}.${rigidbodies_version_MINOR}.${rigidbodies_version_PATCH}
    LANGUAGES CXX
)

# Reconfigure existing build trees when the shared product version changes.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/RigidBodies.Core/include/rigidbodies/project_identity.hpp"
)

if(WIN32)
    enable_language(RC)
endif()

include(GNUInstallDirs)

option(RIGIDBODIES_BUILD_APP "Build the interactive desktop application" ON)
option(RIGIDBODIES_BUILD_TESTS "Build the test executables" OFF)
option(RIGIDBODIES_BUILD_TOOLS "Build command-line diagnostic tools" OFF)
option(RIGIDBODIES_WARNINGS_AS_ERRORS "Treat project compiler warnings as errors" OFF)
option(RIGIDBODIES_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(RIGIDBODIES_ENABLE_CLANG_TIDY "Enforce the repository clang-tidy policy during compilation" OFF)
option(RIGIDBODIES_BUILD_FUZZERS "Build instrumented Clang libFuzzer document targets" OFF)
option(RIGIDBODIES_ENABLE_RMLUI "Build the RmlUi document backend for the user interface" ON)
option(RIGIDBODIES_ENABLE_DEVELOPER_OVERLAY "Build the developer inspection overlay" OFF)

# The simulation and mathematics modules never link against a windowing, rendering, or interface
# dependency. Everything those modules need is in the standard library, which is what allows a
# headless build to configure and test the physics with no third-party package present at all.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Windows deliverables carry the MSVC runtime in each binary. This makes the folder and installer
# runnable on a clean supported Windows installation without a separate redistributable install.
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# One output directory receives the application and the files it needs beside it, with tests in
# tests/ and command-line tools in tools/. The presets point it at build/, the directory the native
# Visual Studio solution also builds into; a plain configure uses the binary directory.
set(RIGIDBODIES_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}" CACHE PATH "Directory receiving the application, its runtime files, tests and tools")
function(rigidbodies_set_output_directory target directory)
    set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${directory}")
    foreach(rigidbodies_config IN LISTS CMAKE_CONFIGURATION_TYPES)
        string(TOUPPER "${rigidbodies_config}" rigidbodies_config_upper)
        set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_${rigidbodies_config_upper} "${directory}")
    endforeach()
endfunction()
if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    # Runtime libraries built from source (a fetched SDL3) land beside the application.
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RIGIDBODIES_OUTPUT_DIRECTORY})
    foreach(rigidbodies_config IN LISTS CMAKE_CONFIGURATION_TYPES)
        string(TOUPPER "${rigidbodies_config}" rigidbodies_config_upper)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${rigidbodies_config_upper} ${RIGIDBODIES_OUTPUT_DIRECTORY})
    endforeach()
endif()

include(cmake/visual_studio_projects.cmake)

add_library(RigidBodies.BuildOptions INTERFACE)
target_compile_features(RigidBodies.BuildOptions INTERFACE cxx_std_17)

target_compile_options(
    RigidBodies.BuildOptions
    INTERFACE
    $<$<CXX_COMPILER_ID:MSVC>:/W4;/permissive-;/Zc:__cplusplus;/Zc:preprocessor;/EHsc;/fp:strict;/Brepro;/utf-8>
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic;-Wshadow;-Wformat=2;-Wundef;-ffp-contract=off>
)
target_link_options(
    RigidBodies.BuildOptions
    INTERFACE
    $<$<CXX_COMPILER_ID:MSVC>:/Brepro>
)

# MSVC deprecates several standard library functions in favour of its own alternatives. The
# deprecation is not part of the standard, and taking it up would make the code build on one
# platform only, so the portable spelling is kept and the warning is turned off.
target_compile_definitions(
    RigidBodies.BuildOptions
    INTERFACE
    $<$<PLATFORM_ID:Windows>:_CRT_SECURE_NO_WARNINGS>
)

if(RIGIDBODIES_WARNINGS_AS_ERRORS)
    target_compile_options(
        RigidBodies.BuildOptions
        INTERFACE
        $<$<CXX_COMPILER_ID:MSVC>:/WX>
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Werror>
    )
endif()

if(RIGIDBODIES_ENABLE_SANITIZERS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(
            RigidBodies.BuildOptions
            INTERFACE
            -fsanitize=address,undefined
            -fno-omit-frame-pointer
            -fno-sanitize-recover=all
        )
        target_link_options(
            RigidBodies.BuildOptions
            INTERFACE
            -fsanitize=address,undefined
            -fno-omit-frame-pointer
        )
    else()
        message(FATAL_ERROR "RIGIDBODIES_ENABLE_SANITIZERS requires GCC or Clang")
    endif()
endif()

if(RIGIDBODIES_ENABLE_CLANG_TIDY)
    find_program(RIGIDBODIES_CLANG_TIDY_EXECUTABLE NAMES clang-tidy REQUIRED)
    set(RIGIDBODIES_CLANG_TIDY_COMMAND "${RIGIDBODIES_CLANG_TIDY_EXECUTABLE}" "--warnings-as-errors=*")
    if(MSVC)
        # clang-tidy 22 does not infer exception support from cl.exe's /EHsc compile flag.
        # Its enum-cast analyzer also diagnoses the standard library's filesystem bitmask.
        list(APPEND RIGIDBODIES_CLANG_TIDY_COMMAND
            "--extra-arg=/EHsc"
            "--checks=-clang-analyzer-optin.core.EnumCastOutOfRange"
        )
    endif()
endif()

function(rigidbodies_configure_target target)
    target_link_libraries(${target} PRIVATE RigidBodies.BuildOptions)
    set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
    if(RIGIDBODIES_ENABLE_CLANG_TIDY)
        set_target_properties(${target} PROPERTIES CXX_CLANG_TIDY "${RIGIDBODIES_CLANG_TIDY_COMMAND}")
    endif()
endfunction()

# ----------------------------------------------------------------------------------------------
# Simulation modules. No dependency outside the standard library.
# ----------------------------------------------------------------------------------------------

rigidbodies_project_files(rigidbodies_math_sources RigidBodies.Math.vcxproj ClCompile)
add_library(RigidBodies.Math STATIC ${rigidbodies_math_sources})
target_include_directories(RigidBodies.Math PUBLIC src/RigidBodies.Math/include)
rigidbodies_configure_target(RigidBodies.Math)

rigidbodies_project_files(rigidbodies_physics_sources RigidBodies.Physics.vcxproj ClCompile)
add_library(RigidBodies.Physics STATIC ${rigidbodies_physics_sources})
target_include_directories(RigidBodies.Physics PUBLIC src/RigidBodies.Physics/include)
find_package(Threads REQUIRED)
target_link_libraries(RigidBodies.Physics PUBLIC RigidBodies.Math Threads::Threads)
target_compile_definitions(RigidBodies.Physics PRIVATE RIGIDBODIES_DEFAULT_SCENARIO_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/assets/scenarios")
rigidbodies_configure_target(RigidBodies.Physics)

if(RIGIDBODIES_BUILD_TOOLS)
    add_executable(RigidBodies.EnergyDrift tools/energy_drift.cpp)
    target_link_libraries(RigidBodies.EnergyDrift PRIVATE RigidBodies.Physics)
    set_target_properties(RigidBodies.EnergyDrift PROPERTIES OUTPUT_NAME rigid_bodies_energy_drift)
    rigidbodies_configure_target(RigidBodies.EnergyDrift)

    add_executable(RigidBodies.RestitutionComparison tools/restitution_comparison.cpp)
    target_link_libraries(RigidBodies.RestitutionComparison PRIVATE RigidBodies.Physics)
    set_target_properties(RigidBodies.RestitutionComparison PROPERTIES OUTPUT_NAME rigid_bodies_restitution_comparison)
    rigidbodies_configure_target(RigidBodies.RestitutionComparison)

    add_executable(RigidBodies.Benchmark tools/benchmark.cpp)
    target_link_libraries(RigidBodies.Benchmark PRIVATE RigidBodies.Physics)
    target_include_directories(RigidBodies.Benchmark PRIVATE src/RigidBodies.Core/include)
    target_compile_definitions(RigidBodies.Benchmark PRIVATE RIGIDBODIES_BENCHMARK_BUILD_CONFIGURATION="$<CONFIG>")
    set_target_properties(RigidBodies.Benchmark PROPERTIES OUTPUT_NAME rigid_bodies_benchmark)
    rigidbodies_configure_target(RigidBodies.Benchmark)
endif()

if(RIGIDBODIES_BUILD_TESTS)
    add_executable(RigidBodies.DeterminismProbe tools/determinism_probe.cpp)
    target_include_directories(RigidBodies.DeterminismProbe PRIVATE tests/support)
    target_link_libraries(RigidBodies.DeterminismProbe PRIVATE RigidBodies.Physics)
    set_target_properties(RigidBodies.DeterminismProbe PROPERTIES OUTPUT_NAME rigid_bodies_determinism_probe)
    rigidbodies_configure_target(RigidBodies.DeterminismProbe)
endif()

if(RIGIDBODIES_BUILD_FUZZERS)
    if(WIN32)
        message(FATAL_ERROR "Coverage-guided document fuzzing is supported on Linux; use the portable document_fuzz CTest suite on Windows")
    endif()
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR NOT RIGIDBODIES_ENABLE_SANITIZERS)
        message(FATAL_ERROR "Document fuzzing requires Clang and RIGIDBODIES_ENABLE_SANITIZERS=ON")
    endif()
    foreach(target IN ITEMS RigidBodies.Math RigidBodies.Physics)
        target_compile_options(${target} PRIVATE -fsanitize=fuzzer-no-link)
    endforeach()
    add_executable(RigidBodies.DocumentFuzzer fuzz/document_fuzzer.cpp fuzz/document_fuzz.cpp)
    target_include_directories(RigidBodies.DocumentFuzzer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(RigidBodies.DocumentFuzzer PRIVATE RigidBodies.Physics)
    target_compile_options(RigidBodies.DocumentFuzzer PRIVATE -fsanitize=fuzzer)
    target_link_options(RigidBodies.DocumentFuzzer PRIVATE -fsanitize=fuzzer)
    set_target_properties(RigidBodies.DocumentFuzzer PROPERTIES OUTPUT_NAME rigidbodies_document_fuzzer)
    rigidbodies_configure_target(RigidBodies.DocumentFuzzer)
endif()

# ----------------------------------------------------------------------------------------------
# Application modules. Only these depend on a platform or rendering package.
# ----------------------------------------------------------------------------------------------

if(RIGIDBODIES_BUILD_APP)
    include(cmake/dependencies.cmake)

    rigidbodies_project_files(rigidbodies_core_sources RigidBodies.Core.vcxproj ClCompile)
    add_library(RigidBodies.Core STATIC ${rigidbodies_core_sources})
    target_include_directories(RigidBodies.Core PUBLIC src/RigidBodies.Core/include)
    rigidbodies_configure_target(RigidBodies.Core)

    rigidbodies_project_files(rigidbodies_render_sources RigidBodies.Render.vcxproj ClCompile)
    add_library(RigidBodies.Render STATIC ${rigidbodies_render_sources})
    target_include_directories(RigidBodies.Render PUBLIC src/RigidBodies.Render/include)
    target_link_libraries(
        RigidBodies.Render
        PUBLIC RigidBodies.Core RigidBodies.Physics
        PRIVATE SDL3::SDL3 Freetype::Freetype
    )
    rigidbodies_configure_target(RigidBodies.Render)

    rigidbodies_project_files(rigidbodies_ui_sources RigidBodies.Ui.vcxproj ClCompile)
    if(NOT RIGIDBODIES_ENABLE_RMLUI)
        list(FILTER rigidbodies_ui_sources EXCLUDE REGEX "/(document_backend|plot_element)\\.cpp$")
    endif()
    add_library(RigidBodies.Ui STATIC ${rigidbodies_ui_sources})
    target_include_directories(RigidBodies.Ui PUBLIC src/RigidBodies.Ui/include)
    target_link_libraries(RigidBodies.Ui PUBLIC RigidBodies.Render)
    rigidbodies_configure_target(RigidBodies.Ui)

    if(RIGIDBODIES_ENABLE_RMLUI)
        target_compile_definitions(RigidBodies.Ui PUBLIC RIGIDBODIES_HAS_RMLUI=1)
        target_link_libraries(RigidBodies.Ui PRIVATE RmlUi::Core)
    endif()

    rigidbodies_project_files(rigidbodies_appcore_sources RigidBodies.AppCore.vcxproj ClCompile)
    add_library(RigidBodies.AppCore STATIC ${rigidbodies_appcore_sources})
    target_include_directories(RigidBodies.AppCore PUBLIC src/RigidBodies.App/include)
    target_link_libraries(RigidBodies.AppCore PUBLIC RigidBodies.Ui)
    rigidbodies_configure_target(RigidBodies.AppCore)

    # The order of these sources fixes the layout of the program, so it is the solution's order.
    rigidbodies_project_files(rigidbodies_app_sources RigidBodies.App.vcxproj ClCompile)
    add_executable(RigidBodies.App ${rigidbodies_app_sources})
    if(WIN32)
        rigidbodies_project_files(rigidbodies_app_resources RigidBodies.App.vcxproj ResourceCompile)
        target_sources(RigidBodies.App PRIVATE ${rigidbodies_app_resources})
        # The resource compiler reports the headers a script includes but not the files it embeds,
        # so a new icon would otherwise leave the program with the old one.
        set_source_files_properties(${rigidbodies_app_resources} PROPERTIES OBJECT_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/assets/branding/rigid-bodies.ico)
        set_target_properties(RigidBodies.App PROPERTIES WIN32_EXECUTABLE TRUE)
    endif()
    target_include_directories(RigidBodies.App PRIVATE src/RigidBodies.App/include)
    target_link_libraries(
        RigidBodies.App
        PRIVATE RigidBodies.AppCore RigidBodies.Ui RigidBodies.Render RigidBodies.Physics RigidBodies.Core SDL3::SDL3
    )
    set_target_properties(RigidBodies.App PROPERTIES OUTPUT_NAME rigid_bodies)
    rigidbodies_set_output_directory(RigidBodies.App "${RIGIDBODIES_OUTPUT_DIRECTORY}")
    rigidbodies_configure_target(RigidBodies.App)
    include(cmake/developer_overlay.cmake)

    # The application locates its assets and configuration relative to the executable at run time.
    # Copying them beside the binary keeps a build-tree run behaving the same way as an installed
    # one, which is what stops resource loading from being tested only after packaging.
    set(rigidbodies_stage_commands
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/assets
            $<TARGET_FILE_DIR:RigidBodies.App>/assets
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/config
            $<TARGET_FILE_DIR:RigidBodies.App>/config
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
            $<TARGET_FILE_DIR:RigidBodies.App>/LICENSE
    )
    if(RIGIDBODIES_SDL_RUNTIME)
        list(APPEND rigidbodies_stage_commands
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RIGIDBODIES_SDL_RUNTIME} $<TARGET_FILE_DIR:RigidBodies.App>
        )
    endif()
    add_custom_target(
        RigidBodies.Assets ALL
        ${rigidbodies_stage_commands}
        COMMENT "Staging license, assets, configuration and runtime libraries beside the executable"
        VERBATIM
    )
    add_dependencies(RigidBodies.App RigidBodies.Assets)

    install(TARGETS RigidBodies.App RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(FILES LICENSE DESTINATION ${CMAKE_INSTALL_BINDIR})
    if(RIGIDBODIES_SDL_RUNTIME)
        install(FILES ${RIGIDBODIES_SDL_RUNTIME} DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
    install(DIRECTORY assets DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# ----------------------------------------------------------------------------------------------
# Simulation tests stay available headlessly. Application builds also test rendering and sessions
# without opening a window.
# ----------------------------------------------------------------------------------------------

if(RIGIDBODIES_BUILD_TESTS)
    enable_testing()

    add_executable(RigidBodies.Math.Tests tests/RigidBodies.Math.Tests/math_tests.cpp)
    target_include_directories(RigidBodies.Math.Tests PRIVATE tests/support)
    target_link_libraries(RigidBodies.Math.Tests PRIVATE RigidBodies.Math)
    rigidbodies_configure_target(RigidBodies.Math.Tests)
    add_test(NAME RigidBodies.Math.Tests COMMAND RigidBodies.Math.Tests)

    add_executable(RigidBodies.Physics.Tests tests/RigidBodies.Physics.Tests/physics_tests.cpp)
    target_include_directories(RigidBodies.Physics.Tests PRIVATE tests/support)
    target_link_libraries(RigidBodies.Physics.Tests PRIVATE RigidBodies.Physics)
    rigidbodies_configure_target(RigidBodies.Physics.Tests)
    add_test(NAME RigidBodies.Physics.Tests COMMAND RigidBodies.Physics.Tests)
    set_tests_properties(RigidBodies.Physics.Tests PROPERTIES TIMEOUT 60)

    foreach(suite IN ITEMS interpolation world_state time_stepper force_channels body_force mass_extensions integrator_comparison kinematic_motion world_dynamics staged_state broad_phase_tree narrow_phase convex_distance contact_solver world_collision restitution_comparison continuous_collision aerodynamics spring physical_effects joint constraint_graph articulation shape_authoring authored_body authoring_scenario education education_accounting body_properties content_json shape_document scenario_catalogue scenario_document body_storage wake_graph parallel_collision simulation_island benchmark analytical_validation convergence property_validation determinism_validation document_fuzz)
        set(test_target RigidBodies.Physics.${suite}.Tests)
        add_executable(${test_target} tests/RigidBodies.Physics.Tests/${suite}_tests.cpp)
        target_include_directories(${test_target} PRIVATE tests/support)
        target_link_libraries(${test_target} PRIVATE RigidBodies.Physics)
        rigidbodies_configure_target(${test_target})
        add_test(NAME ${test_target} COMMAND ${test_target})
        set_tests_properties(${test_target} PROPERTIES TIMEOUT 60)
    endforeach()

    target_sources(RigidBodies.Physics.document_fuzz.Tests PRIVATE fuzz/document_fuzz.cpp)
    target_include_directories(RigidBodies.Physics.document_fuzz.Tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_definitions(RigidBodies.Physics.document_fuzz.Tests PRIVATE RIGIDBODIES_FUZZ_CORPUS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fuzz/corpus")
    foreach(suite IN ITEMS analytical_validation convergence property_validation determinism_validation document_fuzz)
        set_tests_properties(RigidBodies.Physics.${suite}.Tests PROPERTIES TIMEOUT 180 LABELS validation)
    endforeach()

    if(RIGIDBODIES_BUILD_APP)
        add_executable(RigidBodies.Render.Tests tests/RigidBodies.Render.Tests/interpolation_render_tests.cpp)
        target_include_directories(RigidBodies.Render.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.Render.Tests PRIVATE RigidBodies.Render)
        rigidbodies_configure_target(RigidBodies.Render.Tests)
        add_test(NAME RigidBodies.Render.Tests COMMAND RigidBodies.Render.Tests)

        add_executable(RigidBodies.CameraFocus.Tests tests/RigidBodies.Render.Tests/camera_focus_tests.cpp)
        target_include_directories(RigidBodies.CameraFocus.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.CameraFocus.Tests PRIVATE RigidBodies.Render)
        rigidbodies_configure_target(RigidBodies.CameraFocus.Tests)
        add_test(NAME RigidBodies.CameraFocus.Tests COMMAND RigidBodies.CameraFocus.Tests)

        add_executable(RigidBodies.Typography.Tests tests/RigidBodies.Render.Tests/font_atlas_tests.cpp)
        target_include_directories(RigidBodies.Typography.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.Typography.Tests PRIVATE RigidBodies.Render)
        target_compile_definitions(RigidBodies.Typography.Tests PRIVATE RIGIDBODIES_SOURCE_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
        rigidbodies_configure_target(RigidBodies.Typography.Tests)
        add_test(NAME RigidBodies.Typography.Tests COMMAND RigidBodies.Typography.Tests)

        foreach(suite IN ITEMS DrawCompiler SceneEffects)
            if(suite STREQUAL "DrawCompiler")
                set(suite_file draw_compiler_tests)
            else()
                set(suite_file scene_effects_tests)
            endif()
            add_executable(RigidBodies.${suite}.Tests tests/RigidBodies.Render.Tests/${suite_file}.cpp)
            target_include_directories(RigidBodies.${suite}.Tests PRIVATE tests/support)
            target_link_libraries(RigidBodies.${suite}.Tests PRIVATE RigidBodies.Render)
            rigidbodies_configure_target(RigidBodies.${suite}.Tests)
            add_test(NAME RigidBodies.${suite}.Tests COMMAND RigidBodies.${suite}.Tests)
        endforeach()

        add_executable(RigidBodies.FrameCapture.Tests
            tests/RigidBodies.App.Tests/frame_capture_tests.cpp src/RigidBodies.App/src/frame_capture.cpp)
        target_include_directories(RigidBodies.FrameCapture.Tests PRIVATE tests/support src/RigidBodies.App/include)
        target_link_libraries(RigidBodies.FrameCapture.Tests PRIVATE RigidBodies.AppCore RigidBodies.Render SDL3::SDL3)
        rigidbodies_configure_target(RigidBodies.FrameCapture.Tests)
        add_test(NAME RigidBodies.FrameCapture.Tests COMMAND RigidBodies.FrameCapture.Tests)

        add_executable(RigidBodies.GraphicsDevice.Tests tests/RigidBodies.Render.Tests/graphics_device_tests.cpp)
        target_include_directories(RigidBodies.GraphicsDevice.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.GraphicsDevice.Tests PRIVATE RigidBodies.Render SDL3::SDL3)
        target_compile_definitions(RigidBodies.GraphicsDevice.Tests PRIVATE RIGIDBODIES_SOURCE_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
        rigidbodies_configure_target(RigidBodies.GraphicsDevice.Tests)
        add_test(NAME RigidBodies.GraphicsDevice.Tests COMMAND RigidBodies.GraphicsDevice.Tests)
        set_tests_properties(RigidBodies.GraphicsDevice.Tests PROPERTIES TIMEOUT 120)

        add_executable(RigidBodies.PresentationGallery.Tests tests/RigidBodies.Render.Tests/presentation_gallery_tests.cpp)
        target_include_directories(RigidBodies.PresentationGallery.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.PresentationGallery.Tests PRIVATE RigidBodies.Ui SDL3::SDL3)
        target_compile_definitions(RigidBodies.PresentationGallery.Tests PRIVATE RIGIDBODIES_SOURCE_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
        rigidbodies_configure_target(RigidBodies.PresentationGallery.Tests)
        add_test(NAME RigidBodies.PresentationGallery.Tests COMMAND RigidBodies.PresentationGallery.Tests)
        set_tests_properties(RigidBodies.PresentationGallery.Tests PROPERTIES TIMEOUT 120)

        add_executable(RigidBodies.Ui.Tests tests/RigidBodies.Ui.Tests/force_reporting_tests.cpp)
        target_include_directories(RigidBodies.Ui.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.Ui.Tests PRIVATE RigidBodies.Ui)
        rigidbodies_configure_target(RigidBodies.Ui.Tests)
        add_test(NAME RigidBodies.Ui.Tests COMMAND RigidBodies.Ui.Tests)
        add_executable(RigidBodies.ControlKey.Tests tests/RigidBodies.Ui.Tests/control_key_tests.cpp)
        target_include_directories(RigidBodies.ControlKey.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.ControlKey.Tests PRIVATE RigidBodies.Ui)
        target_compile_definitions(RigidBodies.ControlKey.Tests PRIVATE RIGIDBODIES_SOURCE_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
        rigidbodies_configure_target(RigidBodies.ControlKey.Tests)
        add_test(NAME RigidBodies.ControlKey.Tests COMMAND RigidBodies.ControlKey.Tests)
        add_executable(RigidBodies.NumberSpec.Tests tests/RigidBodies.Ui.Tests/number_spec_tests.cpp)
        target_include_directories(RigidBodies.NumberSpec.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.NumberSpec.Tests PRIVATE RigidBodies.Ui)
        rigidbodies_configure_target(RigidBodies.NumberSpec.Tests)
        add_test(NAME RigidBodies.NumberSpec.Tests COMMAND RigidBodies.NumberSpec.Tests)
        add_executable(RigidBodies.ViewState.Tests tests/RigidBodies.Ui.Tests/view_state_tests.cpp)
        target_include_directories(RigidBodies.ViewState.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.ViewState.Tests PRIVATE RigidBodies.Ui)
        rigidbodies_configure_target(RigidBodies.ViewState.Tests)
        add_test(NAME RigidBodies.ViewState.Tests COMMAND RigidBodies.ViewState.Tests)
        add_executable(RigidBodies.Notification.Tests tests/RigidBodies.Ui.Tests/notification_tests.cpp)
        target_include_directories(RigidBodies.Notification.Tests PRIVATE tests/support src/RigidBodies.App/include)
        target_link_libraries(RigidBodies.Notification.Tests PRIVATE RigidBodies.AppCore)
        rigidbodies_configure_target(RigidBodies.Notification.Tests)
        add_test(NAME RigidBodies.Notification.Tests COMMAND RigidBodies.Notification.Tests)
        add_executable(RigidBodies.Format.Tests tests/RigidBodies.Ui.Tests/format_tests.cpp)
        target_include_directories(RigidBodies.Format.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.Format.Tests PRIVATE RigidBodies.Core)
        rigidbodies_configure_target(RigidBodies.Format.Tests)
        add_test(NAME RigidBodies.Format.Tests COMMAND RigidBodies.Format.Tests)
        add_executable(RigidBodies.EducationUi.Tests tests/RigidBodies.Ui.Tests/education_ui_tests.cpp)
        target_include_directories(RigidBodies.EducationUi.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.EducationUi.Tests PRIVATE RigidBodies.Ui)
        rigidbodies_configure_target(RigidBodies.EducationUi.Tests)
        add_test(NAME RigidBodies.EducationUi.Tests COMMAND RigidBodies.EducationUi.Tests)
        foreach(suite IN ITEMS Layout Reachability RowSnapshot)
            if(suite STREQUAL "RowSnapshot")
                set(suite_file row_snapshot_tests)
            else()
                string(TOLOWER ${suite} suite_file)
                string(APPEND suite_file "_tests")
            endif()
            add_executable(RigidBodies.${suite}.Tests tests/RigidBodies.Ui.Tests/${suite_file}.cpp)
            target_include_directories(RigidBodies.${suite}.Tests PRIVATE tests/support)
            target_link_libraries(RigidBodies.${suite}.Tests PRIVATE RigidBodies.Ui)
            target_compile_definitions(RigidBodies.${suite}.Tests PRIVATE RIGIDBODIES_SOURCE_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
            rigidbodies_configure_target(RigidBodies.${suite}.Tests)
            add_test(NAME RigidBodies.${suite}.Tests COMMAND RigidBodies.${suite}.Tests)
        endforeach()
        if(RIGIDBODIES_ENABLE_RMLUI)
            add_executable(RigidBodies.DocumentUi.Tests tests/RigidBodies.Ui.Tests/document_ui_tests.cpp)
            target_include_directories(RigidBodies.DocumentUi.Tests PRIVATE tests/support)
            target_link_libraries(RigidBodies.DocumentUi.Tests PRIVATE RigidBodies.Ui SDL3::SDL3)
            target_compile_definitions(RigidBodies.DocumentUi.Tests PRIVATE RIGIDBODIES_SOURCE_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
            rigidbodies_configure_target(RigidBodies.DocumentUi.Tests)
            add_test(NAME RigidBodies.DocumentUi.Tests COMMAND RigidBodies.DocumentUi.Tests)
            set_tests_properties(RigidBodies.DocumentUi.Tests PROPERTIES TIMEOUT 90)
            add_executable(RigidBodies.DocumentKeyed.Tests tests/RigidBodies.Ui.Tests/document_keyed_tests.cpp)
            target_include_directories(RigidBodies.DocumentKeyed.Tests PRIVATE tests/support)
            target_link_libraries(RigidBodies.DocumentKeyed.Tests PRIVATE RigidBodies.Ui SDL3::SDL3)
            target_compile_definitions(RigidBodies.DocumentKeyed.Tests PRIVATE RIGIDBODIES_SOURCE_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
            rigidbodies_configure_target(RigidBodies.DocumentKeyed.Tests)
            add_test(NAME RigidBodies.DocumentKeyed.Tests COMMAND RigidBodies.DocumentKeyed.Tests)
            set_tests_properties(RigidBodies.DocumentKeyed.Tests PROPERTIES TIMEOUT 90)
        endif()

        add_executable(RigidBodies.Session.Tests tests/RigidBodies.App.Tests/simulation_session_tests.cpp)
        target_include_directories(RigidBodies.Session.Tests PRIVATE tests/support src/RigidBodies.App/include)
        target_link_libraries(RigidBodies.Session.Tests PRIVATE RigidBodies.AppCore)
        rigidbodies_configure_target(RigidBodies.Session.Tests)
        add_test(NAME RigidBodies.Session.Tests COMMAND RigidBodies.Session.Tests)
        foreach(suite IN ITEMS RunModel ExperimentOpen)
            if(suite STREQUAL "RunModel")
                set(suite_file run_model_tests)
            else()
                set(suite_file experiment_open_tests)
            endif()
            add_executable(RigidBodies.${suite}.Tests tests/RigidBodies.App.Tests/${suite_file}.cpp)
            target_include_directories(RigidBodies.${suite}.Tests PRIVATE tests/support src/RigidBodies.App/include)
            target_link_libraries(RigidBodies.${suite}.Tests PRIVATE RigidBodies.AppCore)
            rigidbodies_configure_target(RigidBodies.${suite}.Tests)
            add_test(NAME RigidBodies.${suite}.Tests COMMAND RigidBodies.${suite}.Tests)
        endforeach()

        foreach(suite IN ITEMS AutoPause UndoCategory Measure GuideContent RunRecording)
            if(suite STREQUAL "AutoPause")
                set(suite_file auto_pause_tests)
            elseif(suite STREQUAL "UndoCategory")
                set(suite_file undo_category_tests)
            elseif(suite STREQUAL "GuideContent")
                set(suite_file guide_content_tests)
            elseif(suite STREQUAL "RunRecording")
                set(suite_file run_recording_tests)
            else()
                set(suite_file measure_tests)
            endif()
            add_executable(RigidBodies.${suite}.Tests tests/RigidBodies.App.Tests/${suite_file}.cpp)
            target_include_directories(RigidBodies.${suite}.Tests PRIVATE tests/support src/RigidBodies.App/include)
            target_link_libraries(RigidBodies.${suite}.Tests PRIVATE RigidBodies.AppCore)
            target_compile_definitions(RigidBodies.${suite}.Tests PRIVATE RIGIDBODIES_SOURCE_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
            rigidbodies_configure_target(RigidBodies.${suite}.Tests)
            add_test(NAME RigidBodies.${suite}.Tests COMMAND RigidBodies.${suite}.Tests)
            set_tests_properties(RigidBodies.${suite}.Tests PROPERTIES TIMEOUT 120)
        endforeach()

        add_executable(RigidBodies.Preferences.Tests tests/RigidBodies.App.Tests/preferences_tests.cpp)
        target_include_directories(RigidBodies.Preferences.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.Preferences.Tests PRIVATE RigidBodies.Core)
        rigidbodies_configure_target(RigidBodies.Preferences.Tests)
        add_test(NAME RigidBodies.Preferences.Tests COMMAND RigidBodies.Preferences.Tests)

        add_executable(RigidBodies.ControlSpecSession.Tests tests/RigidBodies.App.Tests/control_spec_session_tests.cpp)
        target_include_directories(RigidBodies.ControlSpecSession.Tests PRIVATE tests/support src/RigidBodies.App/include)
        target_link_libraries(RigidBodies.ControlSpecSession.Tests PRIVATE RigidBodies.AppCore)
        rigidbodies_configure_target(RigidBodies.ControlSpecSession.Tests)
        add_test(NAME RigidBodies.ControlSpecSession.Tests COMMAND RigidBodies.ControlSpecSession.Tests)

        add_executable(RigidBodies.Framing.Tests tests/RigidBodies.App.Tests/framing_tests.cpp)
        target_include_directories(RigidBodies.Framing.Tests PRIVATE tests/support src/RigidBodies.App/include)
        target_link_libraries(RigidBodies.Framing.Tests PRIVATE RigidBodies.AppCore)
        rigidbodies_configure_target(RigidBodies.Framing.Tests)
        add_test(NAME RigidBodies.Framing.Tests COMMAND RigidBodies.Framing.Tests)

        add_executable(RigidBodies.LearnerText.Tests tests/RigidBodies.Ui.Tests/learner_text_tests.cpp)
        target_include_directories(RigidBodies.LearnerText.Tests PRIVATE tests/support src/RigidBodies.App/include)
        target_link_libraries(RigidBodies.LearnerText.Tests PRIVATE RigidBodies.AppCore)
        rigidbodies_configure_target(RigidBodies.LearnerText.Tests)
        add_test(NAME RigidBodies.LearnerText.Tests COMMAND RigidBodies.LearnerText.Tests)
        set_tests_properties(RigidBodies.LearnerText.Tests PROPERTIES TIMEOUT 120)

        add_executable(RigidBodies.ShapeEditor.Tests tests/RigidBodies.App.Tests/shape_editor_tests.cpp)
        target_include_directories(RigidBodies.ShapeEditor.Tests PRIVATE tests/support src/RigidBodies.App/include)
        target_link_libraries(RigidBodies.ShapeEditor.Tests PRIVATE RigidBodies.AppCore)
        rigidbodies_configure_target(RigidBodies.ShapeEditor.Tests)
        add_test(NAME RigidBodies.ShapeEditor.Tests COMMAND RigidBodies.ShapeEditor.Tests)
        add_executable(RigidBodies.EducationSession.Tests tests/RigidBodies.App.Tests/education_session_tests.cpp)
        target_include_directories(RigidBodies.EducationSession.Tests PRIVATE tests/support src/RigidBodies.App/include)
        target_link_libraries(RigidBodies.EducationSession.Tests PRIVATE RigidBodies.AppCore)
        rigidbodies_configure_target(RigidBodies.EducationSession.Tests)
        add_test(NAME RigidBodies.EducationSession.Tests COMMAND RigidBodies.EducationSession.Tests)
        foreach(suite IN ITEMS EditHistory Interaction ContentSession)
            if(suite STREQUAL "EditHistory")
                set(suite_file edit_history_tests)
            elseif(suite STREQUAL "Interaction")
                set(suite_file interaction_tests)
            else()
                set(suite_file content_session_tests)
            endif()
            add_executable(RigidBodies.${suite}.Tests tests/RigidBodies.App.Tests/${suite_file}.cpp)
            target_include_directories(RigidBodies.${suite}.Tests PRIVATE tests/support src/RigidBodies.App/include)
            target_link_libraries(RigidBodies.${suite}.Tests PRIVATE RigidBodies.AppCore)
            rigidbodies_configure_target(RigidBodies.${suite}.Tests)
            add_test(NAME RigidBodies.${suite}.Tests COMMAND RigidBodies.${suite}.Tests)
            set_tests_properties(RigidBodies.${suite}.Tests PROPERTIES TIMEOUT 60)
        endforeach()
        add_executable(RigidBodies.Input.Tests tests/RigidBodies.App.Tests/input_tests.cpp src/RigidBodies.App/src/input.cpp)
        target_include_directories(RigidBodies.Input.Tests PRIVATE tests/support src/RigidBodies.App/include)
        target_link_libraries(RigidBodies.Input.Tests PRIVATE RigidBodies.AppCore SDL3::SDL3)
        rigidbodies_configure_target(RigidBodies.Input.Tests)
        add_test(NAME RigidBodies.Input.Tests COMMAND RigidBodies.Input.Tests)
        add_executable(RigidBodies.InputRouter.Tests tests/RigidBodies.App.Tests/input_router_tests.cpp)
        target_include_directories(RigidBodies.InputRouter.Tests PRIVATE tests/support src/RigidBodies.App/include)
        target_link_libraries(RigidBodies.InputRouter.Tests PRIVATE RigidBodies.AppCore)
        rigidbodies_configure_target(RigidBodies.InputRouter.Tests)
        add_test(NAME RigidBodies.InputRouter.Tests COMMAND RigidBodies.InputRouter.Tests)
        foreach(suite IN ITEMS Manipulation Present)
            string(TOLOWER ${suite} suite_lower)
            add_executable(RigidBodies.${suite}.Tests tests/RigidBodies.App.Tests/${suite_lower}_tests.cpp)
            target_include_directories(RigidBodies.${suite}.Tests PRIVATE tests/support src/RigidBodies.App/include)
            target_link_libraries(RigidBodies.${suite}.Tests PRIVATE RigidBodies.AppCore)
            rigidbodies_configure_target(RigidBodies.${suite}.Tests)
            add_test(NAME RigidBodies.${suite}.Tests COMMAND RigidBodies.${suite}.Tests)
        endforeach()
        add_executable(RigidBodies.StageOverlay.Tests tests/RigidBodies.App.Tests/stage_overlay_tests.cpp)
        target_include_directories(RigidBodies.StageOverlay.Tests PRIVATE tests/support src/RigidBodies.App/include)
        target_link_libraries(RigidBodies.StageOverlay.Tests PRIVATE RigidBodies.AppCore)
        rigidbodies_configure_target(RigidBodies.StageOverlay.Tests)
        add_test(NAME RigidBodies.StageOverlay.Tests COMMAND RigidBodies.StageOverlay.Tests)
        if(RIGIDBODIES_ENABLE_RMLUI)
            # These drive the real document interface and a session together through the
            # application's routing on SDL's window-free software renderer.
            foreach(suite IN ITEMS UiMonkey InterfaceIntegration)
                if(suite STREQUAL "UiMonkey")
                    set(suite_file ui_monkey_tests)
                else()
                    set(suite_file interface_integration_tests)
                endif()
                add_executable(RigidBodies.${suite}.Tests tests/RigidBodies.App.Tests/${suite_file}.cpp)
                target_include_directories(RigidBodies.${suite}.Tests PRIVATE tests/support src/RigidBodies.App/include)
                target_link_libraries(RigidBodies.${suite}.Tests PRIVATE RigidBodies.AppCore RigidBodies.Ui SDL3::SDL3)
                target_compile_definitions(RigidBodies.${suite}.Tests PRIVATE RIGIDBODIES_SOURCE_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/assets")
                rigidbodies_configure_target(RigidBodies.${suite}.Tests)
                add_test(NAME RigidBodies.${suite}.Tests COMMAND RigidBodies.${suite}.Tests)
            endforeach()
            set_tests_properties(RigidBodies.UiMonkey.Tests PROPERTIES TIMEOUT 150)
            set_tests_properties(RigidBodies.InterfaceIntegration.Tests PROPERTIES TIMEOUT 90)
        endif()
        add_executable(RigidBodies.CommandSearch.Tests tests/RigidBodies.Ui.Tests/command_search_tests.cpp)
        target_include_directories(RigidBodies.CommandSearch.Tests PRIVATE tests/support)
        target_link_libraries(RigidBodies.CommandSearch.Tests PRIVATE RigidBodies.Ui)
        rigidbodies_configure_target(RigidBodies.CommandSearch.Tests)
        add_test(NAME RigidBodies.CommandSearch.Tests COMMAND RigidBodies.CommandSearch.Tests)
        set_tests_properties(RigidBodies.Input.Tests PROPERTIES TIMEOUT 60)
        set_tests_properties(RigidBodies.Render.Tests RigidBodies.Ui.Tests RigidBodies.EducationUi.Tests RigidBodies.Session.Tests RigidBodies.ShapeEditor.Tests RigidBodies.EducationSession.Tests PROPERTIES TIMEOUT 60)
    endif()
endif()

# ----------------------------------------------------------------------------------------------
# Tests go to tests/ and command-line tools to tools/ inside the output directory, so the
# application stays the only executable at its root. Tests that open SDL find its runtime there.
# ----------------------------------------------------------------------------------------------

get_property(rigidbodies_all_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
set(rigidbodies_test_runtime_target "")
if(RIGIDBODIES_BUILD_TESTS AND RIGIDBODIES_SDL_RUNTIME)
    add_custom_target(
        RigidBodies.TestRuntime
        COMMAND ${CMAKE_COMMAND} -E make_directory ${RIGIDBODIES_OUTPUT_DIRECTORY}/tests
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RIGIDBODIES_SDL_RUNTIME} ${RIGIDBODIES_OUTPUT_DIRECTORY}/tests
        VERBATIM
    )
    set(rigidbodies_test_runtime_target RigidBodies.TestRuntime)
endif()
foreach(rigidbodies_target IN LISTS rigidbodies_all_targets)
    get_target_property(rigidbodies_target_type ${rigidbodies_target} TYPE)
    if(NOT rigidbodies_target_type STREQUAL "EXECUTABLE" OR rigidbodies_target STREQUAL "RigidBodies.App")
        continue()
    endif()
    if(rigidbodies_target MATCHES "\\.Tests$")
        rigidbodies_set_output_directory(${rigidbodies_target} "${RIGIDBODIES_OUTPUT_DIRECTORY}/tests")
        if(rigidbodies_test_runtime_target)
            add_dependencies(${rigidbodies_target} ${rigidbodies_test_runtime_target})
        endif()
    else()
        rigidbodies_set_output_directory(${rigidbodies_target} "${RIGIDBODIES_OUTPUT_DIRECTORY}/tools")
    endif()
endforeach()
