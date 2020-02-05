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

    for (int i=0; i < s->getNumTerminals() && i < 30; i++) {
        auto result = DualAscent::calculate(s->getGraph(), i, nullptr, s->getNumTerminals(), s->getGraph()->getMaxNode());
        delete result;
        auto result2 = ShortestPath::calculate(i, s->getGraph(), s->getNumTerminals(), s->getGraph()->getNumNodes());
        cout << "Original " << result2->bound << endl;
        LocalOptimization::keyVertexDeletion(*s->getGraph(), *result2, s->getNumTerminals());
        cout << "KV Delete " << result2->bound << endl;
        LocalOptimization::vertexInsertion(s->getGraph(), *result2);
        cout << "V Delete " << result2->bound << endl;
        LocalOptimization::pathExchange(*s->getGraph(), *result2, s->getNumTerminals());
        cout << "Path Exchange " << result2->bound << endl;
        delete result2;
    }
    cout <<"LB " << DualAscent::bestResult << endl;
    cout <<"UB " << ShortestPath::bestResult << endl;

    auto start = high_resolution_clock::now();
    if (!s->getGraph()->checkConnectedness(s->getNumTerminals(), false))
        cout << "Not Connected (start)" << endl;

    auto reductions = vector<Reduction*>();
    reductions.push_back(new ZeroEdgePreselection(s));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new TerminalDistanceReduction(s));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new LongEdgeReduction(s, true, 100));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new NtdkReduction(s, 100, true, 4));
    reductions.push_back(new SdcReduction(s, 100));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new Degree3Reduction(s));
    reductions.push_back(new DegreeReduction(s, false));
    reductions.push_back(new NtdkReduction(s, 100, false, 4));
    reductions.push_back(new DualAscentReduction(s));
    reductions.push_back(new DegreeReduction(s, true));
    reductions.push_back(new HeavyEdgeReduction(s, 100));
    reductions.push_back(new MstPreselection(s));
    reductions.push_back(new steiner::QuickCollection(s));
    reductions.push_back(new DegreeReduction(s, true));

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
    cout << tree->getCost() << endl;
    delete s;


    for(auto r: reductions)
        delete r;

    return 0;
}