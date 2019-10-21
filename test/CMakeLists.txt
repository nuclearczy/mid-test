set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    ${CMAKE_SOURCE_DIR}/app/Human.cpp
    ${CMAKE_SOURCE_DIR}/app/Obstacle.cpp
    ${CMAKE_SOURCE_DIR}/app/Xingyun.cpp
    main.cpp
    testFrontStanding.cpp
    testSideStanding.cpp
)

#find_package(PythonLibs)
#find_package(GTest REQUIRED)
#include_directories(${GTEST_INCLUDE_DIRS})
target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cpp-test PUBLIC gtest)

