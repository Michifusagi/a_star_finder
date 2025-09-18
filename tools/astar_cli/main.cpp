#include <iostream>
#include <string>
#include <optional>
#include "engine/grid.hpp"
#include "engine/astar.hpp"

using namespace engine;

int main(int argc, char** argv) {
    std::string csv, pgm, yaml, heur="octile", outpath;
    int sx=0, sy=0, gx=0, gy=0, block=50; bool diag=true, json=false;

    auto need = [&]{ std::cerr <<
        "Usage: astar_cli --csv <file> --start x y --goal x y "
        "[--diag 0|1] [--heuristic manhattan|euclidean|octile] [--block 50] [--json]\n"; };

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
    auto res = astar_plan(*g, {sy,sx}, {gy,gx}, cfg);

    if (json) {
        std::cout << "{"
        << "\"found\":" << (res.has_value()?"true":"false");
        if (res) {
            std::cout << ",\"cost\":" << res->stats.cost
                    << ",\"expanded\":" << res->stats.expanded
                    << ",\"time_ms\":" << res->stats.time_ms
                    << ",\"length_cells\":" << res->path.size();
        }
        std::cout << "}\n";
    } else {
        std::cout << "found: " << (res ? "yes" : "no") << "\n";
        if (res) {
            std::cout << "cost: " << res->stats.cost << "\n"
                    << "expanded: " << res->stats.expanded << "\n"
                    << "time_ms: " << res->stats.time_ms << "\n"
                    << "length_cells: " << res->path.size() << "\n";
        }
    }
    return 0;
}
