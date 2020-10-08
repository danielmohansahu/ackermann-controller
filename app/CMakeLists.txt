add_executable(shell-app main.cpp AckermannVehicle.cpp Controller.cpp Limits.cpp PID.cpp Vehicle.cpp)
include_directories(
  ${CMAKE_SOURCE_DIR}/include
)
