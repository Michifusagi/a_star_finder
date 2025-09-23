add_library(planner_core
    src/grid.cpp
    src/astar.cpp
) # コンパイル対象はcppファイルのみ、ライブラリターゲットを作成

target_include_directories(planner_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
) # ターゲットに対してヘッダファイルの場所を指定

# 共有ライブラリからリンクされても安全に（静的でもPICに）
set_target_properties(planner_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_compile_options(planner_core PRIVATE -Wall -Wextra -Wpedantic) # ターゲットに対してコンパイルオプションを指定
