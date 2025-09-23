/**
 * @file astar_c.h
 * @brief C API for A* planning.
 */
// astar_c.h
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { int32_t x, y; } point_i32;

// CLI/内部のPlanStatusと1:1対応（終了コードにも使える値）
typedef enum {
    PLAN_OK            = 0,
    PLAN_NO_PATH       = 2,
    PLAN_INVALID_ARG   = 3,
    PLAN_OUT_OF_BOUNDS = 4,
    PLAN_MAP_ERROR     = 5
} plan_status_t;

/**
 * @brief A* 経路探索（占有率0–100、block_threshold超えは障害物）
 *
 * @param occ            rows*cols 要素の占有率グリッド（行優先, y行x列, 0..100）
 * @param rows, cols     グリッドサイズ
 * @param sx, sy         start (x,y)  左上原点, x=col, y=row
 * @param gx, gy         goal  (x,y)
 * @param block_threshold  これ「より大きい」セルを障害物扱い（例: 50）
 * @param allow_diagonal  0=4近傍, 非0=8近傍（斜めコスト=√2）
 * @param path_out       呼び出し側確保バッファ（point_i32配列）。NULL可（長さだけ知りたいとき）
 * @param path_len_inout 入力: path_outの最大要素数。出力: 実際に書き込んだ要素数。
 *                       start==goal の場合は 0（長さ0パス）。
 * @param errbuf         エラーメッセージ出力（NULL可）
 * @param errbuf_len     errbuf のサイズ（bytes）。0可。
 *
 * @return plan_status_t 成功時 PLAN_OK。失敗時は各種ステータス（errbufに理由文字列）。
 *
 * 備考:
 * - パスの並びは start→goal の順。
 * - allow_diagonal=0 のときヒューリスティックはマンハッタン、!=0 のときはオクタイル。
 * - スレッドセーフ（内部で静的状態を持たない）。
 */
plan_status_t astar_plan_c(const int32_t* occ, int32_t rows, int32_t cols,
                            int32_t sx, int32_t sy, int32_t gx, int32_t gy,
                            int32_t block_threshold, int32_t allow_diagonal,
                            point_i32* path_out, int32_t* path_len_inout,
                            char* errbuf, int32_t errbuf_len);

#ifdef __cplusplus
} // extern "C"
#endif
