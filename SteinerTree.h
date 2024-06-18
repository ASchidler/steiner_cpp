//
// Created by andre on 25.01.20.
//

#ifndef STEINER_STEINERTREE_H
#define STEINER_STEINERTREE_H
#include "Steiner.h"
#include "Graph.h"

namespace steiner {
    class SteinerTree {
    public:
        explicit SteinerTree(node_id root) : root_(root)
        {}
        void addEdge(node_id u, node_id v, cost_id w) {
            assert(u != v);
            if (v < u) {
                auto tmp = v;
                v = u;
                u = tmp;
            }

            auto n = edges[u].find(v);
            if (n == edges[u].end() || n->second > w) {
                if (n != edges[u].end())
                    cost_ -= edges[u][v];
                edges[u][v] = w;
                cost_ += w;
            }
        }
        bool removeEdge(node_id u, node_id v, cost_id w) {
            assert(u != v);
            if (v < u) {
                auto tmp = v;
                v = u;
                u = tmp;
            }
            auto n = edges[u].find(v);
            if (n != edges[u].end() && n->second == w) {
                cost_ -= w;
                edges[u].erase(n);
                return true;
            }
            return false;
        }

        node_id getRoot() {
            return root_;
        }

        cost_id getCost() {
            return cost_;
        }
    private:
        unordered_map<node_id, unordered_map<node_id, cost_id>> edges;
        node_id root_;
        cost_id cost_ = 0;
    };
}
#endif //STEINER_STEINERTREE_H
