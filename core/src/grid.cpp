#include "engine/grid.hpp"
#include <fstream>
#include <sstream>

namespace engine {

std::optional<Grid> load_csv(const std::string& path) {
    std::ifstream ifs(path); // ファイルを開く
    if (!ifs) return std::nullopt;

    std::vector<uint8_t> data;
    std::string line;
    int cols = -1, rows = 0;

    // 1行ずつ読み込む
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;
        int colcount = 0;
        // 1ノードずつ読み込む
        while (std::getline(ss, cell, ',')) {
            int v = std::stoi(cell);
            if (v < 0) v = 0; if (v > 255) v = 255;
            data.push_back(static_cast<uint8_t>(v));
            ++colcount;
        }
        if (cols == -1) cols = colcount;
        else if (cols != colcount) return std::nullopt; // 列不一致
        ++rows;
    }

    Grid g;
    g.rows = rows; g.cols = cols; g.occ = std::move(data);
    return g;
}

// ここに後で PGM+YAML を追加してOK
std::optional<Grid> load_pgm_yaml(const std::string&, const std::string&) {
  return std::nullopt; // いったん未実装
}

} // namespace engine
