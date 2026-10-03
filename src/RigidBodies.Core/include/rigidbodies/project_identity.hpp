#pragma once

// Shared product identity. The C++ code, the Windows resource
// script, the CMake build and the Visual Studio projects all read them from here.

#define RIGIDBODIES_VERSION_MAJOR 1
#define RIGIDBODIES_VERSION_MINOR 0
#define RIGIDBODIES_VERSION_PATCH 0
#define RIGIDBODIES_VERSION_STRING "1.0.0"
#define RIGIDBODIES_AUTHOR "Deniz Mert Yayla"
#define RIGIDBODIES_CONTACT "denizmerty@gmail.com"
#define RIGIDBODIES_COPYRIGHT "Copyright © 2026 Deniz Mert Yayla"
#define RIGIDBODIES_LICENSE "MIT License"

#ifndef RC_INVOKED

#include <string_view>

namespace rigidbodies
{

    inline constexpr std::string_view project_name = "RigidBodies";
    inline constexpr std::string_view project_version = RIGIDBODIES_VERSION_STRING;
    inline constexpr int project_version_major = RIGIDBODIES_VERSION_MAJOR;
    inline constexpr int project_version_minor = RIGIDBODIES_VERSION_MINOR;
    inline constexpr int project_version_patch = RIGIDBODIES_VERSION_PATCH;
    inline constexpr std::string_view project_author = RIGIDBODIES_AUTHOR;
    inline constexpr std::string_view project_contact = RIGIDBODIES_CONTACT;
    inline constexpr std::string_view project_copyright = RIGIDBODIES_COPYRIGHT;
    inline constexpr std::string_view project_license = RIGIDBODIES_LICENSE;

    // Shown in the window title and in the about panel.
    inline constexpr std::string_view display_name = "Rigid Bodies";

} // namespace rigidbodies

#endif
