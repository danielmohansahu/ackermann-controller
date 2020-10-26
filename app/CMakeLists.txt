find_package(Threads REQUIRED)

# QT specific cmake requirements
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find QT5 and QCustomPlot
find_package(Qt5 COMPONENTS Core Widgets Charts REQUIRED)
find_package(QCustomPlot REQUIRED)

set(CPP_SOURCES
  demo.cpp
  Controller.cpp
  Limits.cpp PID.cpp
  Model.cpp
  demo/window.cpp
  fake/plant.cpp)

add_executable(demo ${CPP_SOURCES})

target_link_libraries(demo Threads::Threads Qt5::Widgets Qt5::Charts)
include_directories(
  ${CMAKE_SOURCE_DIR}/include
)
