find_package(Threads REQUIRED)

add_executable(shell-app main.cpp Controller.cpp Limits.cpp PID.cpp Model.cpp)
target_link_libraries(shell-app Threads::Threads)
include_directories(
  ${CMAKE_SOURCE_DIR}/include
)
