//
// Created by aschidler on 1/24/20.
//

#ifndef STEINER_DUALASCENT_H
#define STEINER_DUALASCENT_H

#include "../Graph.h"
#include "../SteinerInstance.h"
#include "../Steiner.h"

namespace steiner {
    struct DualAscentResult {
        DualAscentResult(cost_id bound, Graph* g, node_id root) :
                bound(bound), g(g), root(root)
        {}
        ~DualAscentResult() {
            delete g;
        }

        cost_id bound;
        Graph* g;
        node_id root;
    };

    class DualAscent {
    public:
        static DualAscentResult* calculate(Graph* g, node_id root, const dynamic_bitset<>* ts, node_id nTerminals, node_id nNodes);
        inline static cost_id findCut(Graph *dg, node_id n, bool* active, vector<Edge> *edges, bool* cut, node_id nTerminals);
        static bool hasRun;
        static node_id bestRoot;
        static cost_id bestResult;
    };
}


#endif //STEINER_DUALASCENT_H
