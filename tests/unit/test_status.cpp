#include <gtest/gtest.h>
#include "engine/grid.hpp"
#include "engine/astar.hpp"

using namespace engine;

TEST(AstarStatus, OutOfBounds) {
    Grid g;
    g.rows = 3; g.cols = 3;
    g.occ = {0,0,0, 0,0,0, 0,0,0};

    AstarConfig cfg{true, Heuristic::Manhattan, 50};

    // goal が範囲外
    auto out = astar_plan_ex(g, {0,0}, {10,10}, cfg);
    EXPECT_EQ(out.status, PlanStatus::OutOfBounds);
    EXPECT_FALSE(out.result.has_value());
}

TEST(AstarStatus, InvalidArg) {
    Grid g;
    g.rows = 3; g.cols = 3;
    g.occ = {0,0,0, 0,100,0, 0,0,0};

    AstarConfig cfg{true, Heuristic::Manhattan, 50};

    // goal が障害物上
    auto out = astar_plan_ex(g, {0,0}, {1,1}, cfg);
    EXPECT_EQ(out.status, PlanStatus::InvalidArg);
}

TEST(AstarStatus, NoPath) {
    Grid g;
    g.rows = 3; g.cols = 3;
    g.occ = {
        0,100,0,
        100,100,100,
        0,100,0
    };

    AstarConfig cfg{true, Heuristic::Manhattan, 50};

    // ゴールにたどり着けない
    auto out = astar_plan_ex(g, {0,0}, {2,2}, cfg);
    EXPECT_EQ(out.status, PlanStatus::NoPath);
}

TEST(AstarStatus, Ok) {
    Grid g;
    g.rows = 3; g.cols = 3;
    g.occ = {
        0,0,0,
        0,0,0,
        0,0,0
    };

    AstarConfig cfg{true, Heuristic::Manhattan, 50};

    // 通れる
    auto out = astar_plan_ex(g, {0,0}, {2,2}, cfg);
    EXPECT_EQ(out.status, PlanStatus::Ok);
    ASSERT_TRUE(out.result.has_value());
    EXPECT_GT(out.result->stats.cost, 0.0);
}
