//
// Created by aschidler on 1/24/20.
//

#ifndef STEINER_SHORTESTPATH_H
#define STEINER_SHORTESTPATH_H

#include "../SteinerTree.h"

namespace steiner {
    // TODO: Add local improvement
    class ShortestPath {
    public:
        static steiner::HeuristicResult* calculate(node_id root, Graph* g, node_id nTerminals, node_id nNodes);
        static bool hasRun;
        static node_id bestRoot;
        static cost_id bestResult;
    };
}

#endif //STEINER_SHORTESTPATH_H
