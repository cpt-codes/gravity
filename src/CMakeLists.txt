# Note that headers are optional, and do not affect add_library, but
# they will not show up in IDEs unless they are listed in
# add_library.

# Project include directory
set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

# List of headers
set(HEADER_LIST
    "${PROJECT_INCLUDE_DIR}/gravity/BarnesHutAlgorithm.h"
    "${PROJECT_INCLUDE_DIR}/gravity/MassCalculator.h"
    "${PROJECT_INCLUDE_DIR}/gravity/Node.h"
    "${PROJECT_INCLUDE_DIR}/gravity/Octree.h"
    "${PROJECT_INCLUDE_DIR}/gravity/Particle.h"
    "${PROJECT_INCLUDE_DIR}/gravity/forces/IField.h"
    "${PROJECT_INCLUDE_DIR}/gravity/forces/IGravitationalField.h"
    "${PROJECT_INCLUDE_DIR}/gravity/forces/NewtonianField.h"
    "${PROJECT_INCLUDE_DIR}/gravity/forces/PlummerField.h"
    "${PROJECT_INCLUDE_DIR}/gravity/except/AsyncError.h"
    "${PROJECT_INCLUDE_DIR}/gravity/except/ErrorList.h"
    "${PROJECT_INCLUDE_DIR}/gravity/geometry/Vector.h"
    "${PROJECT_INCLUDE_DIR}/gravity/geometry/Orthant.h"
    "${PROJECT_INCLUDE_DIR}/gravity/geometry/BoundingBox.h"
    "${PROJECT_INCLUDE_DIR}/gravity/threads/ITask.h"
    "${PROJECT_INCLUDE_DIR}/gravity/threads/Task.h"
    "${PROJECT_INCLUDE_DIR}/gravity/threads/TaskQueue.h"
    "${PROJECT_INCLUDE_DIR}/gravity/threads/ThreadPool.h")

# Find Boost
find_package(Boost 1.77.0 REQUIRED)

# Find Intel TBB
find_package(TBB REQUIRED)

# Make an automatic library - will be static or dynamic based on user setting
add_library(gravity
    ${HEADER_LIST}
    BarnesHutAlgorithm.cpp
    MassCalculator.cpp
    Node.cpp
    Octree.cpp
    except/ErrorList.cpp
    forces/IField.cpp
    geometry/BoundingBox.cpp
    threads/TaskQueue.cpp
    threads/ThreadPool.cpp)

# We need this directory, and users of our library will need it too
target_include_directories(gravity
    PUBLIC ${PROJECT_INCLUDE_DIR}
    PUBLIC ${Boost_INCLUDE_DIRS})

# Link to required libraries
target_link_libraries(gravity PRIVATE TBB::tbb)

# All users of this library will need at least C++20
target_compile_features(gravity PUBLIC cxx_std_20)

# Define a grouping for location files in IDE project generation
source_group(
    TREE "${PROJECT_INCLUDE_DIR}"
    PREFIX "Header Files"
    FILES ${HEADER_LIST})
