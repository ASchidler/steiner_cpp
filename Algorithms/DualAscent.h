//
// Created by aschidler on 1/24/20.
//

#ifndef STEINER_DUALASCENT_H
#define STEINER_DUALASCENT_H

#include "../Graph.h"
#include "../SteinerInstance.h"

namespace steiner {
    struct DualAscentResult {
        DualAscentResult(unsigned int bound, Graph* g, unsigned int root) :
                bound(bound), g(g), root(root)
        {}
        ~DualAscentResult() {
            delete g;
        }

        unsigned int bound;
        Graph* g;
        unsigned int root;
    };

    class DualAscent {
    public:
        static DualAscentResult* calculate(Graph* g, unsigned int root, unordered_set<unsigned int>* ts);

        static bool hasRun;
        static unsigned int bestRoot;
        static unsigned int bestResult;
    };
}


#endif //STEINER_DUALASCENT_H
