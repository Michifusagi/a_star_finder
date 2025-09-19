include(FetchContent)
FetchContent_Declare(
    gtest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
) # gtestを宣言
FetchContent_MakeAvailable(gtest) # gtestをダウンロード

add_executable(test_astar test_astar.cpp) # テスト実行ファイルを作成（ターゲット作成）
target_link_libraries(test_astar PRIVATE planner_core gtest_main) # ターゲットに対してライブラリをリンク
add_test(NAME astar_tests COMMAND test_astar)

add_executable(test_status test_status.cpp) # ステータステスト
target_link_libraries(test_status PRIVATE planner_core gtest_main)
add_test(NAME astar_status_tests COMMAND test_status)

add_executable(test_grid_loader test_grid_loader.cpp) # load_csvテスト
target_link_libraries(test_grid_loader PRIVATE planner_core gtest_main)
add_test(NAME grid_loader_tests COMMAND test_grid_loader)

file(COPY ${PROJECT_SOURCE_DIR}/maps DESTINATION ${CMAKE_CURRENT_BINARY_DIR})