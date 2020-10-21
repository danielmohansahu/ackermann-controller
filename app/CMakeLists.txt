add_executable(shell-app main.cpp Controller.cpp Limits.cpp PID.cpp Model.cpp)
include_directories(
  ${CMAKE_SOURCE_DIR}/include
)
