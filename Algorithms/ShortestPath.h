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
        explicit ShortestPath(node_id poolSize) : poolSize_(poolSize) {
        }

        ~ShortestPath() {
            for(auto s: resultPool_)
                delete s;
        }

        void findAndAdd(Graph& g, node_id nTerminals, node_id nSolutions);
        void recombine(node_id nSolutions, node_id nTerminals);
        void optimize(Graph& g, node_id nSolutions, node_id nTerminals);

        bool hasResults() {
            return !resultPool_.empty();
        }
        SteinerResult* getBest() {
            if (!resultPool_.empty())
                return resultPool_[0];
            return nullptr;
        }

        cost_id getLowest() {
            return lowestBound_;
        }

        static steiner::SteinerResult* calculate(node_id root, Graph& g, node_id nTerminals);
        static bool hasRun;
        static node_id bestRoot;
        static cost_id bestResult;
        void resetPool(node_id nTerminals);
        void addToPool(SteinerResult* result);
    private:
        cost_id lowestBound_ = MAXCOST;
        node_id poolSize_;
        vector<SteinerResult*> resultPool_;
        node_id terminalRoots[5] = {0, 1, 2, 3, 4};
        node_id nonTerminalRoots[3] = {MAXNODE, MAXNODE, MAXNODE};
        unordered_set<node_id> selectRoots(steiner::Graph &g, node_id nTerminals, node_id nSolutions);
    };
}

#endif //STEINER_SHORTESTPATH_H
