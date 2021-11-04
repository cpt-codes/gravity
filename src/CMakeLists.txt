# Note that headers are optional, and do not affect add_library, but
# they will not show up in IDEs unless they are listed in
# add_library.

# Project include directory
set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

# List of headers
set(HEADER_LIST
    "${PROJECT_INCLUDE_DIR}/gravity/Vector.h"
    "${PROJECT_INCLUDE_DIR}/gravity/IPointMass.h"
    "${PROJECT_INCLUDE_DIR}/gravity/Body.h"
    "${PROJECT_INCLUDE_DIR}/gravity/barneshut/Orthant.h"
    "${PROJECT_INCLUDE_DIR}/gravity/barneshut/Hypercube.h"
    "${PROJECT_INCLUDE_DIR}/gravity/except/ErrorList.h"
    "${PROJECT_INCLUDE_DIR}/gravity/threads/ITask.h"
    "${PROJECT_INCLUDE_DIR}/gravity/threads/Task.h"
    "${PROJECT_INCLUDE_DIR}/gravity/threads/TaskQueue.h"
    "${PROJECT_INCLUDE_DIR}/gravity/threads/ThreadPool.h")

# Boost components to use
set(BOOST_COMPONENTS "")

# Use Boost static libraries and enable multithreading
set(Boost_USE_STATIC_LIBS ON)

if (BOOST_COMPONENTS STREQUAL "")
    # Fail if Boost not found
    find_package(Boost 1.77.0 REQUIRED)
else ()
    # Fail if any components not found
    find_package(Boost 1.77.0 COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
endif ()

# Make an automatic library - will be static or dynamic based on user setting
add_library(gravity ${HEADER_LIST}
    Body.cpp
    barneshut/Hypercube.cpp
    except/ErrorList.cpp
    threads/TaskQueue.cpp
    threads/ThreadPool.cpp)

# We need this directory, and users of our library will need it too
target_include_directories(gravity
    PUBLIC ${PROJECT_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

# All users of this library will need at least C++20
target_compile_features(gravity PUBLIC cxx_std_20)

# Link any libraries necessary
target_link_libraries(gravity ${Boost_LIBRARIES})

# Define a grouping for location files in IDE project generation
source_group(
    TREE "${PROJECT_INCLUDE_DIR}"
    PREFIX "Header Files"
    FILES ${HEADER_LIST})
