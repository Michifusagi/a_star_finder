#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <optional>

namespace engine {

// 0=自由, 100=障害 などのグリッド
struct Grid {
    int rows = 0;
    int cols = 0;
    float resolution = 1.0f; // [m/cell] CSVでは使わなくてOK
    float origin_x = 0.0f;   // world原点
    float origin_y = 0.0f;
    std::vector<uint8_t> occ; // row-major: occ[r*cols + c] (occ = occupancy)

    // メンバ関数
    inline bool in(int r, int c) const { return r>=0 && c>=0 && r<rows && c<cols; }
    inline uint8_t at(int r, int c) const { return occ[r*cols + c]; }
};

// CSVローダ（M1はこれだけでOK）
std::optional<Grid> load_csv(const std::string& path);

// （M1後半〜任意）PGM+YAMLローダ
std::optional<Grid> load_pgm_yaml(const std::string& pgm_path, const std::string& yaml_path);

} // namespace engine
