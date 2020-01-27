//
// Created by aschidler on 1/22/20.
//

#include "Graph.h"
#include "SteinerInstance.h"
#include "HsvSolver.h"
#include <fstream>
#include "Parser/DimacsParser.h"
#include "Algorithms/DualAscent.h"

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
    int nT = 0;
    for (auto cT : *s->getTerminals()) {
        s->getGraph()->switchVertices(cT, nT);
        nT++;
    }
    s->getTerminals()->clear();
    nT--;
    for(; nT >= 0; nT--) {
        s->getTerminals()->insert(nT);
    }

    int i = 0;
    for(auto t: *s->getTerminals()) {
        auto result = DualAscent::calculate(s->getGraph(), t, s->getTerminals(), s->getTerminals()->size(), s->getGraph()->getNumNodes());
        cout << result->bound << endl;
        delete result;
        i++;
        if (i >= 0)
            break;
    }
    cout << DualAscent::bestResult << endl;

    auto solver = HsvSolver(s);
    auto tree = solver.solve();
    cout << tree->cost << endl;
    delete s;
    return 0;
}