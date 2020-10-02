add_executable(shell-app main.cpp Vehicle.cpp PIDController.cpp AckermannVehicle.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
