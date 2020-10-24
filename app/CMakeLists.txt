find_package(Threads REQUIRED)

# QT specific cmake requirements
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

# find QT5 and QCustomPlot
find_package(Qt5 COMPONENTS Widgets PrintSupport REQUIRED)
find_package(QCustomPlot REQUIRED)

add_executable(demo demo.cpp Controller.cpp Limits.cpp PID.cpp Model.cpp)

target_link_libraries(demo Threads::Threads Qt5::Widgets Qt5::PrintSupport)
include_directories(
  ${CMAKE_SOURCE_DIR}/include
)
