# Note that headers are optional, and do not affect add_library, but
# they will not show up in IDEs unless they are listed in
# add_library.

# Project include directory
set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

# List of headers
set(HEADER_LIST
    "${PROJECT_INCLUDE_DIR}/gravity/IGravitationalField.h"
    "${PROJECT_INCLUDE_DIR}/gravity/NewtonianField.h"
    "${PROJECT_INCLUDE_DIR}/gravity/Particle.h"
    "${PROJECT_INCLUDE_DIR}/gravity/PlummerField.h"
    "${PROJECT_INCLUDE_DIR}/gravity/except/ErrorList.h"
    "${PROJECT_INCLUDE_DIR}/gravity/geometry/Vector.h"
    "${PROJECT_INCLUDE_DIR}/gravity/geometry/Orthant.h"
    "${PROJECT_INCLUDE_DIR}/gravity/geometry/BoundingBox.h"
    "${PROJECT_INCLUDE_DIR}/gravity/geometry/Octree.h"
    "${PROJECT_INCLUDE_DIR}/gravity/threads/ITask.h"
    "${PROJECT_INCLUDE_DIR}/gravity/threads/Task.h"
    "${PROJECT_INCLUDE_DIR}/gravity/threads/TaskQueue.h"
    "${PROJECT_INCLUDE_DIR}/gravity/threads/ThreadPool.h")

# Find Boost
find_package(Boost 1.77.0 REQUIRED)

# Make an automatic library - will be static or dynamic based on user setting
add_library(gravity
    ${HEADER_LIST}
    except/ErrorList.cpp
    geometry/BoundingBox.cpp
    geometry/Octree.cpp
    threads/TaskQueue.cpp
    threads/ThreadPool.cpp)

# We need this directory, and users of our library will need it too
target_include_directories(gravity
    PUBLIC ${PROJECT_INCLUDE_DIR}
    PUBLIC ${Boost_INCLUDE_DIRS})

# All users of this library will need at least C++20
target_compile_features(gravity PUBLIC cxx_std_20)

# Define a grouping for location files in IDE project generation
source_group(
    TREE "${PROJECT_INCLUDE_DIR}"
    PREFIX "Header Files"
    FILES ${HEADER_LIST})
