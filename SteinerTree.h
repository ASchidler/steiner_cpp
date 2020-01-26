//
// Created by andre on 25.01.20.
//

#ifndef STEINER_STEINERTREE_H
#define STEINER_STEINERTREE_H
#include "Steiner.h"
#include "Graph.h"

using namespace std;

namespace steiner {
    class SteinerTree {
    public:
        explicit SteinerTree(node_id root) : root(root)
        {}

        vector<Edge> edges;
        bool isEmpty = false;
        node_id root;
        cost_id cost = 0;
    };
}
#endif //STEINER_STEINERTREE_H
