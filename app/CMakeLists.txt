find_package(Threads REQUIRED)

# QT specific cmake requirements
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

# find QT5
find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(demo demo.cpp Controller.cpp Limits.cpp PID.cpp Model.cpp)
target_link_libraries(demo Threads::Threads)
include_directories(
  ${CMAKE_SOURCE_DIR}/include
)
