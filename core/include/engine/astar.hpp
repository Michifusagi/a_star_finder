#pragma once
#include <vector>
#include <optional>
#include "grid.hpp"

namespace engine {

// グリッド座標
struct Cell { int r, c; }; // r=row, c=col

// ヒューリスティック
enum class Heuristic { Manhattan, Euclidean, Octile };

// 初期設定
struct AstarConfig {
    bool allow_diagonal = true; // 斜めがありか否か
    Heuristic heuristic = Heuristic::Octile;
    int block_threshold = 50; // 50以上で障害物認定
};

//　比較のための計測
struct PlanStats {
    double cost = 0.0; // 経路コスト
    int expanded = 0; // 展開したノード数
    double time_ms = 0.0; // 経過時間
};

// 結果
struct PlanResult {
    std::vector<Cell> path; // 経路
    PlanStats stats; // 計測
};

// 失敗理由の列挙
enum class PlanStatus {
    Ok = 0,
    NoPath,        // 経路が見つからない
    OutOfBounds,   // start/goal がグリッド外
    InvalidArg,    // start/goal が障害物 等
    MapError       // 行列サイズ不正などロード失敗系
};

// 結果+ステータス
struct PlanOutcome {
    PlanStatus status = PlanStatus::MapError;
    std::optional<PlanResult> result;  // Ok のときだけ値を持つ
};

// 上位互換API（新設）
PlanOutcome astar_plan_ex(const Grid& g, Cell start, Cell goal, const AstarConfig& cfg);

// メイン関数
std::optional<PlanResult> // これは戻り値の型
astar_plan(const Grid& g, Cell start, Cell goal, const AstarConfig& cfg);

} // namespace engine
