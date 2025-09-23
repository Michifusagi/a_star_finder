add_library(astar SHARED
    astar_c.cpp
)

target_include_directories(astar PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/include   # core のC++ヘッダ
    ${CMAKE_CURRENT_SOURCE_DIR}/include           # capi の astar_c.h
)

target_link_libraries(astar PRIVATE planner_core)

set_target_properties(astar PROPERTIES
    OUTPUT_NAME "astar"           # libastar.dylib / .so / .dll
    POSITION_INDEPENDENT_CODE ON
)
