# Dear ImGui is an internal tool. Release and MinSizeRel never acquire it in a single-config
# build, and a multi-config build never compiles or links it outside Debug/RelWithDebInfo.
set(RIGIDBODIES_BUILD_DEVELOPER_OVERLAY OFF)
if(RIGIDBODIES_ENABLE_DEVELOPER_OVERLAY)
    if(CMAKE_CONFIGURATION_TYPES)
        foreach(development_configuration IN LISTS CMAKE_CONFIGURATION_TYPES)
            if(development_configuration MATCHES "^(Debug|RelWithDebInfo)$")
                set(RIGIDBODIES_BUILD_DEVELOPER_OVERLAY ON)
            endif()
        endforeach()
    elseif(CMAKE_BUILD_TYPE MATCHES "^(Debug|RelWithDebInfo)$")
        set(RIGIDBODIES_BUILD_DEVELOPER_OVERLAY ON)
    endif()
endif()

if(NOT RIGIDBODIES_BUILD_DEVELOPER_OVERLAY)
    return()
endif()

# Official v1.91.9b, kept in vendor/DearImGui and otherwise fixed to its immutable commit and
# archive digest. It is never installed or packaged.
set(rigidbodies_vendor_imgui "")
if(EXISTS "${PROJECT_SOURCE_DIR}/vendor/DearImGui/imgui.h")
    set(rigidbodies_vendor_imgui "${PROJECT_SOURCE_DIR}/vendor/DearImGui")
endif()
set(RIGIDBODIES_IMGUI_SOURCE_DIR "${rigidbodies_vendor_imgui}" CACHE PATH "Prepared Dear ImGui 1.91.9b source directory")
if(NOT RIGIDBODIES_IMGUI_SOURCE_DIR)
    if(NOT RIGIDBODIES_FETCH_DEPENDENCIES)
        message(FATAL_ERROR "Developer overlay requires RIGIDBODIES_IMGUI_SOURCE_DIR or dependency fetching")
    endif()
    FetchContent_Declare(
        DearImGui
        URL https://codeload.github.com/ocornut/imgui/zip/f5befd2d29e66809cd1110a152e375a7f1981f06
        URL_HASH SHA256=e17c195c7b98a5edf5be1d034114a3920beb3c182e49384ed04ef7fab35fa702
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(DearImGui)
    set(RIGIDBODIES_IMGUI_SOURCE_DIR "${dearimgui_SOURCE_DIR}")
endif()

# The application links this object target only in development configurations. The developer
# overlay test is configuration-independent, so the target itself must contain its sources in
# every configuration even though no Release application or package links them.
add_library(RigidBodies.DearImGui OBJECT EXCLUDE_FROM_ALL)
foreach(imgui_source IN ITEMS imgui.cpp imgui_draw.cpp imgui_tables.cpp imgui_widgets.cpp
        backends/imgui_impl_sdl3.cpp backends/imgui_impl_sdlrenderer3.cpp backends/imgui_impl_opengl3.cpp)
    target_sources(RigidBodies.DearImGui PRIVATE ${RIGIDBODIES_IMGUI_SOURCE_DIR}/${imgui_source})
endforeach()
target_include_directories(RigidBodies.DearImGui SYSTEM PUBLIC
    ${RIGIDBODIES_IMGUI_SOURCE_DIR}
    ${RIGIDBODIES_IMGUI_SOURCE_DIR}/backends
)
target_link_libraries(RigidBodies.DearImGui PUBLIC SDL3::SDL3 ${CMAKE_DL_LIBS})
target_compile_features(RigidBodies.DearImGui PUBLIC cxx_std_17)
