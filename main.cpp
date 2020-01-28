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

using namespace steiner;

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

    for (int i=0; i < s->getNumTerminals() && i < 10; i++) {
        //TODO: Receive label for heuristics instead of list of terminals...
        auto result = DualAscent::calculate(s->getGraph(), i, &ts, s->getNumTerminals(), s->getGraph()->getMaxNode());
        delete result;
        auto result2 = ShortestPath::calculate(i, s->getGraph(), s->getNumTerminals(), s->getGraph()->getNumNodes());
        delete result2;
    }
    cout <<"LB " << DualAscent::bestResult << endl;
    cout <<"UB " << ShortestPath::bestResult << endl;

    if (! s->getGraph()->isConnected())
        cout << "Not Connected (start)" << endl;

    auto reductions = vector<Reduction*>();
    reductions.push_back(new DegreeReduction(s, false));
    auto reducer = Reducer(reductions, s);
    reducer.reduce();

    if (! s->getGraph()->isConnected())
        cout << "Not Connected (after reduction)" << endl;

    cout << "Solving " << s->getGraph()->getNumNodes() << " nodes and " << s->getNumTerminals() << " terminals"<< endl;
    auto solver = HsvSolver(s);
    auto tree = solver.solve();
    reducer.unreduce(tree);
    cout << tree->getCost() << endl;
    delete s;


    for(auto r: reductions)
        delete r;

    return 0;
}