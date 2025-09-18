#include "engine/astar.hpp"
#include <queue>
#include <cmath>
#include <limits>
#include <chrono>
#include <algorithm>

namespace engine {

struct Node { int r,c; double g,h; };

// 比較
struct Cmp {
    // オーバーロード（関数オブジェクトにする）
    bool operator()(const Node& a, const Node& b) const {
        return (a.g + a.h) > (b.g + b.h); // f = g+h が小さいほど優先
    }
};

// ヒューリスティックコスト
static inline double hcost(int r,int c,int gr,int gc, Heuristic h) {
    int dr = std::abs(gr - r), dc = std::abs(gc - c);
    switch (h) {
        case Heuristic::Manhattan: return dr + dc;
        case Heuristic::Euclidean: return std::hypot(dr, dc); // 三平方の定理
        case Heuristic::Octile: {
            int dmin = std::min(dr, dc), dmax = std::max(dr, dc);
            return (std::sqrt(2.0) - 1.0) * dmin + dmax;
        }

    }
    return dr + dc;
}

std::optional<PlanResult>
// s: start, t: target(goal)
astar_plan(const Grid& g, Cell s, Cell t, const AstarConfig& cfg) {
    // 範囲外ならnullopt
    if (!g.in(s.r,s.c) || !g.in(t.r,t.c)) return std::nullopt;

    // 時間計測
    auto t0 = std::chrono::high_resolution_clock::now();
    // グリッドのサイズ
    const int N = g.rows * g.cols;

    auto idx = [&](int r,int c){ return r*g.cols + c; }; // occでのインデックス
    auto free_cell = [&](int r,int c){ return g.in(r,c) && g.at(r,c) < cfg.block_threshold; }; // freeかどうか

    std::priority_queue<Node, std::vector<Node>, Cmp> open; // 最小ヒープ
    std::vector<double> best(N, std::numeric_limits<double>::infinity()); // 各ノードの最小コスト
    std::vector<int> parent(N, -1); // 各ノードの親

    Node st{ s.r, s.c, 0.0, hcost(s.r,s.c,t.r,t.c,cfg.heuristic) }; // スタートノード
    open.push(st);
    best[idx(s.r,s.c)] = 0.0;

    std::vector<std::pair<int,int>> dirs = cfg.allow_diagonal
        ? std::vector<std::pair<int,int>>{{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}}
        : std::vector<std::pair<int,int>>{{1,0},{-1,0},{0,1},{0,-1}};

    int expanded = 0; // 展開したノード数

    while(!open.empty()){
        Node cur = open.top(); open.pop();
        if (cur.g > best[idx(cur.r,cur.c)]) continue; // 古いノードをスキップ

        if (cur.r == t.r && cur.c == t.c) {
            // 経路復元
            std::vector<Cell> path;
            int r = cur.r, c = cur.c;
            while (!(r==s.r && c==s.c)) {
                path.push_back({r,c});
                int p = parent[idx(r,c)];
                if (p < 0) break;
                r = p / g.cols; c = p % g.cols;
            }
            path.push_back({s.r,s.c});
            std::reverse(path.begin(), path.end());
            auto t1 = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            return PlanResult{ std::move(path), {cur.g, expanded, ms} };
        }

        ++expanded;
        for (auto [dr,dc] : dirs) {
            int nr = cur.r + dr, nc = cur.c + dc;
            if (!free_cell(nr,nc)) continue;
            double step = (dr && dc) ? std::sqrt(2.0) : 1.0;
            double ng = cur.g + step;
            int id = idx(nr,nc);
            if (ng < best[id]) {
                best[id] = ng; // bestの更新
                parent[id] = idx(cur.r,cur.c); // 親の更新
                open.push(Node{nr,nc,ng,hcost(nr,nc,t.r,t.c,cfg.heuristic)});
            }
        }
    }
    return std::nullopt; // 到達不能
}

} // namespace engine
