set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    main.cpp
    # Unit level tests
    unit/AckermannVehicle.cpp
    unit/Controller.cpp
    unit/Limits.cpp
    unit/PID.cpp
    unit/State.cpp
    # System level tests
    system.cpp
)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cpp-test PUBLIC gtest)
