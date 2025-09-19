#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include "engine/grid.hpp"

using namespace engine;
namespace fs = std::filesystem;

// 簡単な一時ファイル作成ヘルパ
static std::string write_temp(const std::string& name, const std::string& content) {
    fs::path dir = fs::temp_directory_path() / "a_star_finder_tests";
    fs::create_directories(dir);
    fs::path p = dir / name;
    std::ofstream ofs(p.string(), std::ios::binary);
    ofs << content;
    ofs.close();
    return p.string();
}

TEST(GridLoaderStatus, Ok) {
    // 2行×3列の妥当なCSV
    std::string path = write_temp("ok.csv",
        "0,0,100\n"
        "0,50,0\n"
    );

    auto lr = load_csv_ex(path);
    EXPECT_EQ(lr.status, LoadStatus::Ok);
    ASSERT_TRUE(lr.grid.has_value());
    const auto& g = *lr.grid;
    EXPECT_EQ(g.rows, 2);
    EXPECT_EQ(g.cols, 3);
    ASSERT_EQ((int)g.occ.size(), 6);
    EXPECT_EQ(g.occ[0], 0);
    EXPECT_EQ(g.occ[2], 100);
    EXPECT_EQ(g.occ[4], 50);

    // 互換API（optional版）も成功すること
    auto gopt = load_csv(path);
    EXPECT_TRUE(gopt.has_value());
}

TEST(GridLoaderStatus, EmptyFile) {
    std::string path = write_temp("empty.csv", "");

    auto lr = load_csv_ex(path);
    EXPECT_EQ(lr.status, LoadStatus::EmptyFile);
    EXPECT_FALSE(lr.grid.has_value());

    // 互換APIは nullopt
    auto gopt = load_csv(path);
    EXPECT_FALSE(gopt.has_value());
}

TEST(GridLoaderStatus, RowLengthMismatch) {
    // 1行目は2列、2行目は3列
    std::string path = write_temp("mismatch.csv",
        "0,0\n"
        "0,0,0\n"
    );

    auto lr = load_csv_ex(path);
    EXPECT_EQ(lr.status, LoadStatus::RowLengthMismatch);
    EXPECT_FALSE(lr.grid.has_value());

    auto gopt = load_csv(path);
    EXPECT_FALSE(gopt.has_value());
}

TEST(GridLoaderStatus, NonIntegerToken) {
    std::string path = write_temp("nonint.csv",
        "0,X,0\n"
    );

    auto lr = load_csv_ex(path);
    EXPECT_EQ(lr.status, LoadStatus::NonIntegerToken);
    EXPECT_FALSE(lr.grid.has_value());
}

TEST(GridLoaderStatus, OutOfRangeToken) {
    std::string path = write_temp("range.csv",
        "0,101,0\n"
    );

    auto lr = load_csv_ex(path);
    EXPECT_EQ(lr.status, LoadStatus::OutOfRangeToken);
    EXPECT_FALSE(lr.grid.has_value());
}

TEST(GridLoaderStatus, FileOpenFailed) {
    // 存在しないパスを指定
    std::string bogus = "/this/path/does/not/exist.csv";

    auto lr = load_csv_ex(bogus);
    EXPECT_EQ(lr.status, LoadStatus::FileOpenFailed);
    EXPECT_FALSE(lr.grid.has_value());

    auto gopt = load_csv(bogus);
    EXPECT_FALSE(gopt.has_value());
}
