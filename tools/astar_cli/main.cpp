#include <iostream>
#include <string>
#include <optional>
#include "engine/grid.hpp"
#include "engine/astar.hpp"

using namespace engine;

int main(int argc, char** argv) {
    std::string csv, pgm, yaml, heur="octile", outpath;
    int sx=0, sy=0, gx=0, gy=0, block=50; bool diag=true, json=false, explain=false, print_path=false;

    auto need = [&]{ std::cerr <<
        "Usage: astar_cli --csv <file> --start x y --goal x y "
        "[--diag 0|1] [--heuristic manhattan|euclidean|octile] [--block 50] "
        "[--json] [--explain] [--print-path]\n"; };

    for (int i=1;i<argc;++i){
        std::string a = argv[i];
        auto nexts = [&](std::string& s){ if(++i>=argc) return; s = argv[i]; };
        auto nexti = [&](int& v){ if(++i>=argc) return; v = std::stoi(argv[i]); };
        if (a=="--csv") nexts(csv);
        else if (a=="--pgm") nexts(pgm);
        else if (a=="--yaml") nexts(yaml);
        else if (a=="--start") { nexti(sx); nexti(sy); }
        else if (a=="--goal")  { nexti(gx); nexti(gy); }
        else if (a=="--diag")  { int v; nexti(v); diag = (v!=0); }
        else if (a=="--heuristic") nexts(heur);
        else if (a=="--block") nexti(block);
        else if (a=="--json")  json = true;
        else if (a=="--explain") explain = true;
        else if (a=="--print-path") print_path = true;
    }
    if (csv.empty() && (pgm.empty() || yaml.empty())) { need(); return 2; }

    std::optional<Grid> g = !csv.empty() ? load_csv(csv) : load_pgm_yaml(pgm,yaml);
    if (!g) { std::cerr << "Failed to load map\n"; return 2; }

    AstarConfig cfg;
    cfg.allow_diagonal = diag;
    cfg.block_threshold = block;
    if (heur=="manhattan") cfg.heuristic = Heuristic::Manhattan;
    else if (heur=="euclidean") cfg.heuristic = Heuristic::Euclidean;
    else cfg.heuristic = Heuristic::Octile;

    // 注意：CLIは (x,y) 入力 → 内部は (r,c)=(y,x)
    auto out = astar_plan_ex(*g, {sy,sx}, {gy,gx}, cfg);

    if (json) {
        std::cout << "{"
        << "\"found\":" << (out.result.has_value()?"true":"false");
        if (out.result.has_value()) {
            std::cout << ",\"cost\":" << out.result->stats.cost
                    << ",\"expanded\":" << out.result->stats.expanded
                    << ",\"time_ms\":" << out.result->stats.time_ms
                    << ",\"length_cells\":" << out.result->path.size();
        }
        std::cout << "}\n";
        return 0;
    }

    auto exit_code = [](PlanStatus s) {
        switch (s) {
        case PlanStatus::Ok: return 0;
        case PlanStatus::NoPath: return 2;
        case PlanStatus::InvalidArg: return 3;
        case PlanStatus::OutOfBounds: return 4;
        case PlanStatus::MapError: return 5;
        }
        return 5;
    };

    // 経路が見つからない場合
    if(out.status != PlanStatus::Ok) {
        if(explain) {
            switch(out.status) {
                case PlanStatus::NoPath:
                    std::cerr << "No path found\n";
                    break;
                case PlanStatus::InvalidArg:
                    std::cerr << "Invalid argument\n";
                    break;
                case PlanStatus::OutOfBounds:
                    std::cerr << "Out of bounds\n";
                    break;
                case PlanStatus::MapError:
                    std::cerr << "Map error\n";
                    break;
            }
        }
        // 非JSONの失敗時はstderr側に統一
        std::cerr << "found: no\n";
        return exit_code(out.status);
    }

    // 経路が見つかった場合
    std::cout << "found: yes\n";

    // --print-path があれば path を (x,y) で出力（1行1座標）
    if (print_path && out.result.has_value()) {
        const auto& path = out.result->path; // vector<Cell>
        for (const auto& p : path) {
            std::cout << p.c << " " << p.r << "\n";
        }
    }

    // --explainがあればメッセージ
    if (explain) {
        const auto& stats = out.result->stats;
        std::cout << "expanded: " << stats.expanded << "\n";
        std::cout << "cost: " << stats.cost << "\n";
        std::cout << "time_ms: " << stats.time_ms << "\n";
        std::cout << "length_cells: " << out.result->path.size() << "\n";
    }

    return 0;
}
