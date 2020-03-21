//
// Created by aschidler on 1/22/20.
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

using namespace steiner;
using namespace chrono;
// TODO: Voronoi bound reductions could be used for large instances...
int main(int argc, char* argv[]) {
    if (argc != 2) {
        cerr << "steiner expects only the filename as the first parameter." << endl;
        return 1;
    }

    auto filename = string(argv[1]);
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
    auto rsph = ShortestPath(10);
    rsph.findAndAdd(*s->getGraph(), s->getNumTerminals(), 10);
    for (int i=0; i < s->getNumTerminals() && i < 5; i++) {
        auto result = DualAscent::calculate(s->getGraph(), i, nullptr, s->getNumTerminals(), s->getGraph()->getMaxNode());
        delete result;
    }
    assert(s->checkGraphIntegrity());
    cout << "Before " << rsph.getLowest() << endl;
    rsph.optimize(*s->getGraph(), 5, s->getNumTerminals());
    cout << "After Optimize " << rsph.getLowest() << endl;
    assert(s->checkGraphIntegrity());
    rsph.recombine(5, s->getNumTerminals());
    cout << "After Recombine " << rsph.getLowest() << endl;
    assert(s->checkGraphIntegrity());
    rsph.optimize(*s->getGraph(), 5, s->getNumTerminals());
    cout << "After Optimize" << rsph.getLowest() << endl;
    assert(s->checkGraphIntegrity());
    cout <<"LB " << DualAscent::bestResult << endl;
    cout << "UB " << rsph.getBest()->cost << endl;

    auto start = high_resolution_clock::now();
    if (!s->getGraph()->checkConnectedness(s->getNumTerminals(), false))
        cout << "Not Connected (start)" << endl;

    auto reductions = vector<Reduction*>();
    reductions.push_back(new ZeroEdgePreselection(s));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new TerminalDistanceReduction(s));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new LongEdgeReduction(s, true, 2000));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new NtdkReduction(s, 2000, true, 4));
    reductions.push_back(new SdcReduction(s, 2000));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new Degree3Reduction(s));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new NtdkReduction(s, 2000, false, 4));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new DualAscentReduction(s));
    reductions.push_back(new DegreeReduction(s, true));
    reductions.push_back(new HeavyEdgeReduction(s, 2000));
    reductions.push_back(new MstPreselection(s));
    reductions.push_back(new steiner::QuickCollection(s));

    auto reducer = Reducer(reductions, s);
    reducer.reduce();

    if (!s->getGraph()->checkConnectedness(s->getNumTerminals(), false))
        cout << "Not Connected (after reduction)" << endl;
    s->checkGraphIntegrity();

    cout << "Solving " << s->getGraph()->getNumNodes() << " nodes and " << s->getNumTerminals() << " terminals"<< endl;
    auto solver = HsvSolver(s);
    auto tree = solver.solve();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>((stop - start));
    cout << duration.count() / 1000000.0 << endl;

    assert(tree != nullptr);


    reducer.unreduce(tree);
    cout << tree->cost << endl;
    delete s;

    return 0;
}