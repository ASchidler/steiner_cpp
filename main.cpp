//
// Created on 1/22/20.
//

#include "Graph.h"
#include "SteinerInstance.h"
#include "HsvSolver.h"
#include <fstream>
#include "Parser/DimacsParser.h"
#include "Algorithms/DualAscent.h"
#include "Reductions/Reducer.h"
#include "Reductions/DegreeReduction.h"
#include "Algorithms/ShortestPath.h"
#include "Reductions/LongEdgeReduction.h"
#include "Reductions/SdcReduction.h"
#include "Reductions/NtdkReduction.h"
#include "Reductions/TerminalDistanceReduction.h"
#include "Reductions/Degree3Reduction.h"
#include "Reductions/DualAscentReduction.h"
#include "Reductions/HeavyEdgeReduction.h"
#include "Reductions/ZeroEdgePreselection.h"
#include "Reductions/MstPreselection.h"
#include "Reductions/ShortLinksPreselection.h"
#include "Reductions/NearestVertexPreselection.h"
#include "Reductions/QuickCollection.h"
#include <chrono>
#include "Algorithms/LocalOptimization.h"
#include "dynamic/HsvSolver.h"
#include <random>
#include "HybridSolver.h"

using namespace steiner;
using namespace chrono;
// TODO: Voronoi bound reductions could be used for large instances...
int main(int argc, char* argv[]) {
    // Make reproducible
    std::srand(0);
    bool useReductions = true;
    int dualAscentLimit = 10000;

    if (argc == 1) {
        cerr << "Usage: steiner <filename>" << endl
            << "The following parameters are available:" << endl
             << "-r Use no reductions" << endl
             << "-d <number> sets the edge limit for dual ascent. Default is 10 000, 0 disables dual ascent" << endl;
        return 1;
    }

    // Parse arguments
    for (int i = 1; i < argc-1; ++i) {
        string arg = string(argv[i]);
        if (arg == "-r") {
            useReductions = false;
        }
        else if (arg == "-d") {
            if (argc <= i + 1) {
                cerr << "-d requires a number";
                return 1;
            }
            try {
                auto conv_result = stoi(argv[i+1]);
                if (conv_result < 0) {
                    cerr << "Dual ascent limit cannot be negative" << endl;
                    return 1;
                }
                dualAscentLimit = conv_result;
                i++;
            }
            catch(invalid_argument const &e)
            {
                cerr << "Invalid number entered for dual ascent limit "<< argv[i+1] << endl;
                return 1;
            }
            catch (std::out_of_range const &e) {
                cerr << "Invalid number entered for dual ascent limit "<< argv[i+1] << endl;
                return 1;
            }
        }
    }

    auto filename = string(argv[argc-1]);
    ifstream f(filename.c_str());
    if (not f.good()) {
        cerr << filename << " not found or not accessible" << endl;
        return 2;
    }

    auto parser = DimacsParser();
    SteinerInstance* s = parser.parse(filename);

    auto ts = unordered_set<node_id>();
    for (int i=0; i < s->getNumTerminals(); i++) {
        ts.emplace(i);
    }
    assert(s->checkGraphIntegrity());

    auto start = high_resolution_clock::now();
        if (!s->getGraph()->checkConnectedness(s->getNumTerminals(), false))
            cout << "Not Connected (start)" << endl;

    auto reductions = vector<Reduction *>();
    reductions.push_back(new ZeroEdgePreselection(s));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new TerminalDistanceReduction(s));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new LongEdgeReduction(s, true, 2000));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new NtdkReduction(s, 2000, true, 4));
    reductions.push_back(new SdcReduction(s, 2000));
    reductions.push_back(new DegreeReduction(s, false));
    // reductions.push_back(new Degree3Reduction(s));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new NtdkReduction(s, 2000, false, 4));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new DualAscentReduction(s));
    reductions.push_back(new DegreeReduction(s, true));
    reductions.push_back(new HeavyEdgeReduction(s, 2000));
    reductions.push_back(new MstPreselection(s));
    reductions.push_back(new steiner::QuickCollection(s, true));

    auto reducer = Reducer(reductions, s);

    if (useReductions) {
        reducer.reduce();

        if (!s->getGraph()->checkConnectedness(s->getNumTerminals(), false))
            cout << "Not Connected (after reduction)" << endl;
        s->checkGraphIntegrity();
    }

    cout << "Solving " << s->getGraph()->getNumNodes() << " nodes and " << s->getNumTerminals() << " terminals"<< endl;
    HybridSolver<uint128_type> slv;
    slv.solve(*s);
    SteinerResult* tree = nullptr;
    if (s->getNumTerminals() < 16) {
        auto solver = HsvSolver<uint16_t>(s, dualAscentLimit);
        tree = solver.solve();
    } else if (s->getNumTerminals() < 32) {
        auto solver = HsvSolver<uint32_t>(s, dualAscentLimit);
        tree = solver.solve();
    }  else if (s->getNumTerminals() < 64) {
        auto solver = HsvSolver<uint64_t>(s, dualAscentLimit);
        tree = solver.solve();
    } else if (s->getNumTerminals() < 128) {
        auto solver = HsvSolver<uint128_type>(s, dualAscentLimit);
        tree = solver.solve();
    } else {
        auto solver = DynamicHsvSolver(s, dualAscentLimit);
        tree = solver.solve();
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>((stop - start));
    cout << duration.count() / 1000000.0 << endl;
//
//    assert(tree != nullptr);
//
//    if (useReductions)
//        reducer.unreduce(tree);
//
//    cout << tree->cost << endl;
    delete s;

    return 0;
}