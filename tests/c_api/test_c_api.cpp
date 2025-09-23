#include <gtest/gtest.h>
#include <vector>
#include <cstring>
#include <string>

extern "C" {
#include "astar_c.h"
}

// ヘルパ: 空の占有率グリッド（0..100, 0=free）
static std::vector<int32_t> make_grid(int rows, int cols, int32_t fill = 0) {
    std::vector<int32_t> g((size_t)rows * (size_t)cols, fill);
    return g;
}
static inline size_t idx(int r, int c, int cols) { return (size_t)r * (size_t)cols + (size_t)c; }

TEST(CAPI, Ok_StraightLine_NeedLenThenFetch) {
    const int rows = 5, cols = 5;
    auto occ = make_grid(rows, cols, 0);
    // start(0,0) -> goal(4,0) 水平移動（4近傍）
    int32_t sx=0, sy=0, gx=4, gy=0;

    // まず長さだけ知る（path_out=nullptr / len=0）
    int need_len = 0;
    auto st = astar_plan_c(
        occ.data(), rows, cols,
        sx, sy, gx, gy,
        /*block_threshold=*/50, /*allow_diagonal=*/0,
        /*path_out=*/nullptr, &need_len,
        /*errbuf=*/nullptr, /*errlen=*/0
    );
    ASSERT_EQ(st, PLAN_OK);
    ASSERT_GT(need_len, 0);

    // 取得した長さでバッファを確保して再呼び出し
    std::vector<point_i32> path((size_t)need_len);
    int cap = need_len;
    char err[128] = {0};
    st = astar_plan_c(
        occ.data(), rows, cols,
        sx, sy, gx, gy,
        50, 0,
        path.data(), &cap,
        err, sizeof(err)
    );
    ASSERT_EQ(st, PLAN_OK);
    ASSERT_EQ(cap, need_len);
    // 終点は必ず goal
    ASSERT_EQ(path[(size_t)cap-1].x, gx);
    ASSERT_EQ(path[(size_t)cap-1].y, gy);
}

TEST(CAPI, StartEqualsGoal_LengthZero) {
    const int rows = 3, cols = 3;
    auto occ = make_grid(rows, cols, 0);
    int32_t sx=1, sy=1, gx=1, gy=1;

    int len = 123; // 入力値は何でも良い
    char err[64] = {0};
    auto st = astar_plan_c(
        occ.data(), rows, cols,
        sx, sy, gx, gy,
        50, 0,
        /*path_out=*/nullptr, &len,
        err, sizeof(err)
    );
    ASSERT_EQ(st, PLAN_OK);
    ASSERT_EQ(len, 0); // 長さ0で返る
    ASSERT_STREQ(err, ""); // エラーメッセージなし
}

TEST(CAPI, InvalidArg_StartOnObstacle) {
    const int rows = 4, cols = 4;
    auto occ = make_grid(rows, cols, 0);
    // start(0,0) を障害物化（block_threshold=50 を超える値）
    occ[idx(0,0,cols)] = 100;

    int len = 0;
    char err[64] = {0};
    auto st = astar_plan_c(
        occ.data(), rows, cols,
        /*sx,sy=*/0,0, /*gx,gy=*/3,3,
        50, 0,
        nullptr, &len,
        err, sizeof(err)
    );
    ASSERT_EQ(st, PLAN_INVALID_ARG);
    // 人間可読メッセージ（内容は厳密比較しない）
    ASSERT_GT(std::strlen(err), 0u);
}

TEST(CAPI, OutOfBounds) {
    const int rows = 3, cols = 3;
    auto occ = make_grid(rows, cols, 0);

    int len = 0;
    char err[64] = {0};
    // sx=-1 -> 範囲外
    auto st = astar_plan_c(
        occ.data(), rows, cols,
        -1, 0, 2, 2,
        50, 0,
        nullptr, &len,
        err, sizeof(err)
    );
    ASSERT_EQ(st, PLAN_OUT_OF_BOUNDS);
    ASSERT_GT(std::strlen(err), 0u);
}

TEST(CAPI, NoPath_WalledOff) {
    const int rows = 5, cols = 5;
    auto occ = make_grid(rows, cols, 0);
    // 行 y=2 を壁で完全遮断
    for (int c=0; c<cols; ++c) occ[idx(2,c,cols)] = 100;

    int need = 0;
    char err[64] = {0};
    auto st = astar_plan_c(
        occ.data(), rows, cols,
        /*sx,sy=*/0,0, /*gx,gy=*/4,4,
        50, 0,
        nullptr, &need,
        err, sizeof(err)
    );
    ASSERT_EQ(st, PLAN_NO_PATH);
    ASSERT_GT(std::strlen(err), 0u);
}

TEST(CAPI, TruncatedBuffer_EmitsMessage) {
    const int rows = 5, cols = 5;
    auto occ = make_grid(rows, cols, 0);
    int32_t sx=0, sy=0, gx=4, gy=4;

    // 必要長だけ先に取得
    int need = 0;
    auto st = astar_plan_c(
        occ.data(), rows, cols,
        sx, sy, gx, gy,
        50, 0,
        nullptr, &need,
        nullptr, 0
    );
    ASSERT_EQ(st, PLAN_OK);
    ASSERT_GT(need, 1);

    // わざと小さいバッファで再呼び出し
    int cap = need / 2; // 小さめ
    std::vector<point_i32> small((size_t)cap);
    char err[128] = {0};
    st = astar_plan_c(
        occ.data(), rows, cols,
        sx, sy, gx, gy,
        50, 0,
        small.data(), &cap,
        err, sizeof(err)
    );
    ASSERT_EQ(st, PLAN_OK);
    // cap は書き込まれた要素数（=元の容量まで）
    ASSERT_EQ(cap, (int)small.size());
    // メッセージに "truncated" を含む（実装の文言に合わせる）
    ASSERT_NE(std::string(err).find("truncated"), std::string::npos);
}
