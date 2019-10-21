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

find_package(PythonLibs)
target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include 
                                           PRIVATE ${PYTHON_INCLUDE_DIR})
target_link_libraries(cpp-test PUBLIC gtest ${PYTHON_LIBRARY})

message(${PYTHON_INCLUDE_DIRS})
