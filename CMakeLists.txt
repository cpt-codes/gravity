# From 3.12 we gain C++20 support and useful features in CMake
cmake_minimum_required(VERSION 3.12...3.18)

# VERSION major.minor.patch.tweak
project(gravity
    VERSION 1.0.0.0
    DESCRIPTION "N-Body Simulation"
    LANGUAGES CXX)

# Only do these if this is the main project, and not if it is included
# through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    # Optionally set things like CMAKE_CXX_STANDARD,
    # CMAKE_POSITION_INDEPENDENT_CODE here

    # Reset the global standard of C++ to use
    set(CMAKE_CXX_STANDARD 20)

    # Support folders in IDEs
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif()

# Executable code is here
add_subdirectory(apps)
