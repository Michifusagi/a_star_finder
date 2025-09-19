#include "engine/grid.hpp"
#include <fstream>
#include <sstream>

namespace engine {

// 空字ではないか、符号はついていなか、少数ではないか、チェックしてintに変換
static bool parse_int_token(const std::string& cell, int& out) {
    // 前後空白トリム（簡易）
    size_t b = cell.find_first_not_of(" \t\r"); // beginの位置
    size_t e = cell.find_last_not_of(" \t\r"); // endの位置
    if (b == std::string::npos) return false; // 空白のみならfalse
    std::string t = cell.substr(b, e - b + 1); // 空白を取り除いた文字列
    // 正負記号は許容しない（CSVは0..100想定）
    for (char c : t) if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    try { out = std::stoi(t); } catch (...) { return false; }
    return true;
}

LoadResult load_csv_ex(const std::string& path) {
    LoadResult out;

    std::ifstream ifs(path); // ファイルを開く
    if (!ifs) return out;

    std::vector<std::vector<int>> rows; 
    std::string line;
    int line_no = 0;

    // 1行ずつ読み込む
    while (std::getline(ifs, line)) {
        ++line_no;
        if (line.find_first_not_of(" \t\r,") == std::string::npos) continue;

        std::vector<int> row;
        std::stringstream ss(line);
        std::string cell;
        int colcount = 0;
        // 1ノードずつ読み込む
        while (std::getline(ss, cell, ',')) {
            int v;
            ++colcount;
            if (!parse_int_token(cell, v)) {
                out.status = LoadStatus::NonIntegerToken;
                out.error_line = line_no;
                out.error_column = colcount;
                return out;
            }
            if (v < 0 || v > 100) {
                out.status = LoadStatus::OutOfRangeToken;
                out.error_line = line_no;
                out.error_column = colcount;
                return out;
            }
            row.push_back(v);
        }
        if (!row.empty()) rows.push_back(std::move(row));
    }

    if (rows.empty()){
        out.status = LoadStatus::EmptyFile;
        return out;
    }

    // 列数チェック
    const std::size_t cols = rows.front().size();
    for (std::size_t r = 1; r < rows.size(); ++r) {
        if (rows[r].size() != cols) {
            out.status = LoadStatus::RowLengthMismatch;
            out.error_line = static_cast<int>(r + 1); // size_tをintにcast
            out.error_column = -1; // 列不一致の場合は-1
            return out;
        }
    }

    Grid g;
    g.rows = static_cast<int>(rows.size());
    g.cols = static_cast<int>(cols);
    g.occ.reserve(g.rows * g.cols);
    for (const auto& r : rows){
        g.occ.insert(g.occ.end(), r.begin(), r.end());
    }
    
    out.status = LoadStatus::Ok;
    out.grid = std::move(g);
    return out;
}

std::optional<Grid> load_csv(const std::string& path) {
    auto r = load_csv_ex(path);
    if (r.status == LoadStatus::Ok) return std::move(r.grid);
    return std::nullopt;
}

// ここに後で PGM+YAML を追加してOK
std::optional<Grid> load_pgm_yaml(const std::string&, const std::string&) {
  return std::nullopt; // いったん未実装
}

} // namespace engine
