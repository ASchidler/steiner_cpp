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

    for(auto t: *s->getTerminals()) {
        auto result = DualAscent::calculate(s->getGraph(), t, s->getTerminals());
        cout << result->bound << endl;
        delete result;
    }
    cout << DualAscent::bestResult << endl;

    auto solver = HsvSolver(s);
    solver.solver();

    delete s;
    return 0;
}