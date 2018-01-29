cmake_minimum_required(VERSION 3.5)
project(websocketExample)

set(CMAKE_CXX_STANDARD 11)

find_package(websocketpp REQUIRED)
find_package(Boost REQUIRED system)

add_executable(websocketExample websocket_example.cpp)
target_link_libraries(websocketExample ${Boost_LIBRARIES} pthread)
