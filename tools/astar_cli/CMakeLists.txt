add_executable(astar_cli main.cpp)
target_link_libraries(astar_cli PRIVATE planner_core)
target_compile_options(astar_cli PRIVATE -Wall -Wextra -Wpedantic)
