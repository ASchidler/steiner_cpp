//
// Created by aschidler on 1/24/20.
//

#ifndef STEINER_DUALASCENT_H
#define STEINER_DUALASCENT_H

#include "../Graph.h"
#include "../SteinerInstance.h"
#include "../Steiner.h"

namespace steiner {
    struct DualAscentEdge {
        DualAscentEdge(node_id u, node_id v, cost_id* c) : u(u), v(v), c(c)
        {}
        node_id u;
        node_id v;
        cost_id* c;
    };

    class DualAscent {
    public:
        static HeuristicResult* calculate(Graph* g, node_id root, const dynamic_bitset<>* ts, node_id nTerminals, node_id nNodes);
        inline static cost_id findCut(Graph& dg, node_id n, bool* active, vector<DualAscentEdge>& edges, bool* cut, node_id nTerminals);
        static bool hasRun;
        static node_id bestRoot;
        static cost_id bestResult;
    };
}


#endif //STEINER_DUALASCENT_H
