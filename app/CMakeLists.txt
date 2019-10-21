find_package(  Boost 1.58 REQUIRED COMPONENTS program_options regex )
add_executable(shell-app main.cpp Human.cpp Obstacle.cpp Xingyun.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    /usr/include/boost
    ${Boost_INCLUDE_DIRS}
)

find_package(PythonLibs)
target_include_directories(shell-app PRIVATE ${PYTHON_INCLUDE_DIR})
target_link_libraries(shell-app ${PYTHON_LIBRARY})
message(${PYTHON_INCLUDE_DIRS})


