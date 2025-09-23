// astar_c.cpp
#include "astar_c.h"
#include "engine/astar.hpp"   // あなたの既存ヘッダに合わせて調整
#include "engine/grid.hpp"
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

using namespace engine;

// エラーメッセージをバッファに書き込む
static void put_err(char* buf, int32_t n, const std::string& msg) {
    if (!buf || n <= 0) return;
    int32_t len = std::min<int32_t>(n - 1, (int32_t)msg.size());
    std::memcpy(buf, msg.data(), len);
    buf[len] = '\0';
}

plan_status_t astar_plan_c(const int32_t* occ, int32_t rows, int32_t cols,
                            int32_t sx, int32_t sy, int32_t gx, int32_t gy,
                            int32_t block_threshold, int32_t allow_diagonal,
                            point_i32* path_out, int32_t* path_len_inout,
                            char* errbuf, int32_t errbuf_len)
{
    // 1) 引数バリデーション（最小限）
    if (!occ || rows <= 0 || cols <= 0 || !path_len_inout) {
        put_err(errbuf, errbuf_len, "invalid arguments");
        return PLAN_MAP_ERROR;
    }
    // start/goal 範囲
    if (sx < 0 || sx >= cols || gx < 0 || gx >= cols ||
        sy < 0 || sy >= rows || gy < 0 || gy >= rows) {
        put_err(errbuf, errbuf_len, "start/goal out of bounds");
        return PLAN_OUT_OF_BOUNDS;
    }

  // 2) Grid 構築（占有率→ブール通行可否へ変換はA*内部でやってもOK。ここではGridに占有率を詰める想定）
    Grid g;
    g.rows = rows;
    g.cols = cols;
    g.occ.resize((size_t)rows * (size_t)cols);
    for (int32_t r = 0; r < rows; ++r) {
        for (int32_t c = 0; c < cols; ++c) {
        int32_t v = occ[(size_t)r * (size_t)cols + (size_t)c];
        g.occ[(size_t)r * (size_t)cols + (size_t)c] = v; // 0..100 をそのまま格納
        }
    }

    // 3) コンフィグ
    AstarConfig cfg;
    cfg.allow_diagonal = (allow_diagonal != 0);
    cfg.block_threshold = block_threshold;
    cfg.heuristic = cfg.allow_diagonal ? Heuristic::Octile : Heuristic::Manhattan;

    // 4) 計画
    auto out = astar_plan_ex(
        g,
        /*start(y,x)*/ { sy, sx },
        /*goal (y,x)*/ { gy, gx },
        cfg
    );

    // 5) ステータス振り分け & エラーメッセージ
    auto to_c_status = [](PlanStatus s)->plan_status_t {
        switch (s) {
        case PlanStatus::Ok:          return PLAN_OK;
        case PlanStatus::NoPath:      return PLAN_NO_PATH;
        case PlanStatus::InvalidArg:  return PLAN_INVALID_ARG;
        case PlanStatus::OutOfBounds: return PLAN_OUT_OF_BOUNDS;
        case PlanStatus::MapError:    return PLAN_MAP_ERROR;
        }
        return PLAN_MAP_ERROR;
    };

    plan_status_t st = to_c_status(out.status);
    if (st != PLAN_OK) {
        put_err(errbuf, errbuf_len,
        (out.message.empty() ? std::string("planning failed") : out.message));
        *path_len_inout = 0;
        return st;
    }

    // 6) パス出力（start→goal）。start==goal は長さ0で返す設計。
    // 結果がない場合はエラー
    const auto& opt_path = out.result; // std::optional<PlanResult>
    if (!opt_path.has_value()) {
        put_err(errbuf, errbuf_len, "ok but empty result");
        *path_len_inout = 0;
        return PLAN_OK;
    }

    // 結果がある場合はパスを出力
    const auto& path_rc = opt_path->path; // (r,c) の配列 PlanResult の pathメンバ
    // start==goal のときは0
    if (path_rc.size() == 1 && path_rc.front().r == sy && path_rc.front().c == sx && sx == gx && sy == gy) {
        *path_len_inout = 0;
        return PLAN_OK;
    }

    // (r,c)->(x,y) に変換
    std::vector<point_i32> out_xy;
    out_xy.reserve(path_rc.size());
    for (auto& p : path_rc) {
        out_xy.push_back({ (int32_t)p.c, (int32_t)p.r });
    }

    int32_t cap = *path_len_inout;
    int32_t need = (int32_t)out_xy.size();
    if (path_out && cap > 0) {
        int32_t wr = std::min(cap, need);
        // .data()で連続メモリの先頭ポインタを取得
        std::memcpy(path_out, out_xy.data(), (size_t)wr * sizeof(point_i32));
        *path_len_inout = wr;
        if (wr < need) {
            put_err(errbuf, errbuf_len, "path buffer too small (truncated)");
        }
    } else {
        // 長さだけ知りたい場合
        *path_len_inout = need;
    }

    return PLAN_OK;
}
