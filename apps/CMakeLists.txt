# Create target executable
add_executable(gravity-simulation main.cpp)

# Require at least C++20 to compile
target_compile_features(gravity-simulation PRIVATE cxx_std_20)

# Link the library compiled from src
target_link_libraries(gravity-simulation PRIVATE gravity)
