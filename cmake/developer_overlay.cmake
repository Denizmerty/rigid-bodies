if(NOT RIGIDBODIES_BUILD_DEVELOPER_OVERLAY)
    return()
endif()

set(rigidbodies_developer_config "$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>")
target_sources(RigidBodies.App PRIVATE
    "$<${rigidbodies_developer_config}:${CMAKE_CURRENT_SOURCE_DIR}/src/RigidBodies.App/src/developer_overlay.cpp>"
)
target_compile_definitions(RigidBodies.App PRIVATE
    "$<${rigidbodies_developer_config}:RIGIDBODIES_DEVELOPER_OVERLAY=1>"
)
target_link_libraries(RigidBodies.App PRIVATE
    "$<${rigidbodies_developer_config}:RigidBodies.DearImGui>"
)

if(RIGIDBODIES_BUILD_TESTS)
    add_executable(RigidBodies.DeveloperOverlay.Tests
        tests/RigidBodies.App.Tests/developer_overlay_tests.cpp
        src/RigidBodies.App/src/developer_overlay.cpp
    )
    target_include_directories(RigidBodies.DeveloperOverlay.Tests PRIVATE
        tests/support src/RigidBodies.App/include
    )
    target_link_libraries(RigidBodies.DeveloperOverlay.Tests PRIVATE RigidBodies.Ui RigidBodies.DearImGui)
    rigidbodies_configure_target(RigidBodies.DeveloperOverlay.Tests)
    set(rigidbodies_register_developer_test ON)
    if(CMAKE_CONFIGURATION_TYPES)
        # Do not introduce a developer test executable into Release's default build. Ninja and
        # Visual Studio expose different per-configuration target-exclusion mechanisms.
        if(CMAKE_GENERATOR STREQUAL "Ninja Multi-Config")
            set_property(TARGET RigidBodies.DeveloperOverlay.Tests PROPERTY
                EXCLUDE_FROM_ALL "$<NOT:${rigidbodies_developer_config}>")
        elseif(CMAKE_GENERATOR MATCHES "Visual Studio")
            foreach(configuration IN LISTS CMAKE_CONFIGURATION_TYPES)
                string(TOUPPER "${configuration}" upper_configuration)
                if(configuration MATCHES "^(Debug|RelWithDebInfo)$")
                    set(exclude_developer_test OFF)
                else()
                    set(exclude_developer_test ON)
                endif()
                set_property(TARGET RigidBodies.DeveloperOverlay.Tests PROPERTY
                    EXCLUDE_FROM_DEFAULT_BUILD_${upper_configuration} ${exclude_developer_test})
            endforeach()
        else()
            # Other multi-config generators may build this explicitly in a development config.
            set_property(TARGET RigidBodies.DeveloperOverlay.Tests PROPERTY EXCLUDE_FROM_ALL ON)
            set(rigidbodies_register_developer_test OFF)
        endif()
    endif()
    if(rigidbodies_register_developer_test)
        if(CMAKE_CONFIGURATION_TYPES)
            add_test(NAME RigidBodies.DeveloperOverlay.Tests COMMAND RigidBodies.DeveloperOverlay.Tests
                CONFIGURATIONS Debug RelWithDebInfo)
        else()
            add_test(NAME RigidBodies.DeveloperOverlay.Tests COMMAND RigidBodies.DeveloperOverlay.Tests)
        endif()
        set_tests_properties(RigidBodies.DeveloperOverlay.Tests PROPERTIES TIMEOUT 60)
    endif()
endif()
