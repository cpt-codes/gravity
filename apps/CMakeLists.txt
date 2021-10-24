# Create target executable
add_executable(grav-sim main.cpp)

# Require at least C++20 to compile
target_compile_features(grav-sim PRIVATE cxx_std_20)
