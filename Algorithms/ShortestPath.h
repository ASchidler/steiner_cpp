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
        static SteinerTree *calculate(node_id root, Graph* g, node_id nTerminals, node_id nNodes);
        static bool hasRun;
        static node_id bestRoot;
        static cost_id bestResult;
    };
}

/*
 *     // Now fill the rest up with non-terminals
    node_id nonTerminalStart = rootsSelected;
    for(; rootsSelected < numRoots; rootsSelected++) {
        node_id n = instance->getNumTerminals() + (random() % (instance->getGraph()->getNumNodes() - instance->getNumTerminals()));
        for(auto j=nonTerminalStart; j < rootsSelected; j++) {
            // Find next existing node
            while (instance->getGraph()->getNodes()->count(n) == 0) {
                n++;
                if (n > instance->getGraph()->getMaxNode())
                    n=instance->getNumTerminals();
            }
            // Ensure that it has not been chosen yet
            if (roots[j] == n) {
                n++;
                j = nonTerminalStart;
            }
        }
        roots[rootsSelected] = n;
    }
 */

#endif //STEINER_SHORTESTPATH_H
