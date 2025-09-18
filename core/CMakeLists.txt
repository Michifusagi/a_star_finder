add_library(planner_core
    src/grid.cpp
    src/astar.cpp
) # コンパイル対象はcppファイルのみ、ライブラリターゲットを作成
target_include_directories(planner_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
) # ターゲットに対してヘッダファイルの場所を指定
target_compile_options(planner_core PRIVATE -Wall -Wextra -Wpedantic) # ターゲットに対してコンパイルオプションを指定
