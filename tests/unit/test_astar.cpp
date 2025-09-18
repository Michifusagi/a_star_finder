#include <gtest/gtest.h>
#include "engine/grid.hpp"
#include "engine/astar.hpp"
using namespace engine;

TEST(Astar, Reachable) {
    auto g = load_csv("maps/simple.csv");
    ASSERT_TRUE(g.has_value());
    AstarConfig cfg{true, Heuristic::Manhattan, 50};
    auto r = astar_plan(*g, {0,0}, {7,9}, cfg); // (r,c)=(y,x)
    ASSERT_TRUE(r.has_value());
    EXPECT_GT(r->stats.cost, 0.0);
}

TEST(Astar, Unreachable) {
    Grid g; g.rows=3; g.cols=3; g.occ = {
        0,100,0,
        100,0,100,
        0,100,0
    };
    AstarConfig cfg{false, Heuristic::Manhattan, 50};
    auto r = astar_plan(g, {1,1}, {0,0}, cfg);
    EXPECT_FALSE(r.has_value());
}
