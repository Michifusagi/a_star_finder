cmake_minimum_required(VERSION 3.16)
project(a_star_finder CXX)
set(CMAKE_CXX_STANDARD 17) # C++17を使用する
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # 共有ライブラリを作成するためのオプション

add_subdirectory(capi)
add_subdirectory(core)
add_subdirectory(tools/astar_cli)

enable_testing() # add_testを使用するために必要
add_subdirectory(tests/unit)
add_subdirectory(tests/c_api)

include(FetchContent)
FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip)
FetchContent_MakeAvailable(googletest)