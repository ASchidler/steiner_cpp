//
// Created by aschidler on 1/22/20.
//

#ifndef STEINER_STEINERINSTANCE_H
#define STEINER_STEINERINSTANCE_H
#include "Graph.h"
#include <bits/stdc++.h>
#include "Algorithms/ShortestPath.h"
#include <utility>
#include "SteinerTree.h"

using namespace std;

namespace steiner {
    class SteinerInstance {
    public:
        SteinerInstance(Graph *g, vector<node_id> *terminals);
        SteinerInstance(Graph *g, node_id nTerminals) : g_(g), nTerminals(nTerminals) {

        }
        ~SteinerInstance() {
            clearCache();
        }

        node_id getNumTerminals() {
            return nTerminals;
        }

        bool addEdge(node_id u, node_id v, cost_id c);
        unordered_set<node_id>::iterator removeNode(node_id u);
        unordered_set<node_id>::iterator removeNode(unordered_set<node_id>::iterator u);
        void removeEdge(node_id u, node_id v);
        Graph::EdgeIterator removeEdge(Graph::EdgeIterator);
        unordered_set<node_id>::iterator contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result);
        unordered_set<node_id>::iterator contractEdge(unordered_set<node_id>::iterator target, node_id remove, vector<ContractedEdge>* result);
        NodeWithCost* getClosestTerminals(node_id n);
        cost_id getDistance(node_id n1, node_id n2);
        void moveTerminal(node_id t, node_id target);
        void invalidateTerminals();
        cost_id getSteinerDistance(node_id u, node_id v);
        enum ValueState { lower, exact, higher, invalid};
        void shrink();

        ShortestPath& getApproximation() {
            if (approximationState_ != exact) {
                approximation_.resetPool(nTerminals);
                // TODO: Find a good number of roots...
                approximation_.findAndAdd(*g_, nTerminals, 30);
                approximation_.recombine(10, nTerminals);
                approximation_.optimize(*g_, 10, nTerminals);

                approximationState_ = exact;
            }
            return approximation_;
        }

        cost_id getUpperBound() {
            return getApproximation().getLowest();
        }

        void setDistanceState(ValueState s) {
            if (distanceState_ == invalid)
                return;
            if (distanceState_ == exact)
                distanceState_ = s;
            else if ((distanceState_ == lower && s == higher) || (distanceState_ == higher && s == lower))
                distanceState_ = invalid;
            else
                distanceState_ = s;
        }

        void requestDistanceState(ValueState s) {
            if (s == invalid ||
                (s == exact && distanceState_ != exact) ||
                (distanceState_ == lower && s == higher) ||
                (distanceState_ == higher && s == lower))
                distanceState_ = invalid;
        }

        void setSteinerDistanceState(ValueState s) {
            if (steinerDistanceState_ == invalid)
                return;
            if (steinerDistanceState_ == exact)
                steinerDistanceState_ = s;
            else if ((steinerDistanceState_ == lower && s == higher) || (steinerDistanceState_ == higher && s == lower))
                steinerDistanceState_ = invalid;
            else
                steinerDistanceState_ = s;
        }

        void requestSteinerDistanceState(ValueState s) {
            if (s == invalid ||
                (s == exact && steinerDistanceState_ != exact) ||
                (steinerDistanceState_ == lower && s == higher) ||
                (steinerDistanceState_ == higher && s == lower))
                steinerDistanceState_ = invalid;
        }

        void setApproximationState(ValueState s) {
            if (approximationState_ == invalid)
                return;
            if (approximationState_ == exact)
                approximationState_ = s;
            if ((approximationState_ == lower && s == higher) || (approximationState_ == higher && s == lower))
                approximationState_ = invalid;
            else
                approximationState_ = s;
        }

        void requestApproximationState(ValueState s) {
            if (s == invalid ||
            (s == exact && approximationState_ != exact) ||
            (approximationState_ == lower && s == higher) ||
            (approximationState_ == higher && s == lower))
                approximationState_ = invalid;
        }


        Graph *getGraph() {
            return this->g_;
        }
        bool checkGraphIntegrity() {
            bool integer = true;
            for (auto n: g_->getNodes()) {
                for(auto v: g_->nb[n]) {
                    if (g_->nb[v.first].count(n) == 0) {
                        cout << "ERROR, one sided" << endl;
                        integer = false;
                    }
                    else if (g_->nb[v.first][n] != v.second) {
                        cout << "ERROR! " << v.first << " " << n << " have different costs" << endl;
                        integer = false;
                    }
                }

            }
            return integer;
        }
    private:
        Graph *g_;
        NodeWithCost** closest_terminals_ = nullptr;
        node_id closestTerminalsInit = 0;
        cost_id** terminalSteinerDistances_ = nullptr;
        node_id terminalSteinerDistanceInit_ = 0;
        node_id nTerminals = 0;

        inline node_id removeNode_(node_id u);
        void calculateSteinerDistance();

        ValueState distanceState_ = invalid;
        ValueState steinerDistanceState_ = invalid;
        ValueState approximationState_ = invalid;
        ShortestPath approximation_ = ShortestPath(10);

        void clearCache() {
            if (closest_terminals_ != nullptr) {
                for (int n = 0; n < closestTerminalsInit; n++) {
                    if (closest_terminals_[n] != nullptr)
                        delete[] closest_terminals_[n];
                }
                delete[] closest_terminals_;
                closest_terminals_ = nullptr;
            }

            if (terminalSteinerDistances_ != nullptr) {
                for (int i=0; i < terminalSteinerDistanceInit_; i++) {
                    delete[] terminalSteinerDistances_[i];
                    terminalSteinerDistances_[i] = nullptr;
                }
                delete[] terminalSteinerDistances_;
                terminalSteinerDistances_ = nullptr;
            }
        }
        void clearDistance() {
            g_->discardDistances();
            clearCache();

            distanceState_ = exact;
            steinerDistanceState_ = exact;
        }
    };

    struct MergedEdges {
        MergedEdges(node_id removed, node_id u, node_id v, cost_id cu, cost_id cv) :
            newEdge(u, v, cu + cv),
            oldEdge1(removed, u, cu),
            oldEdge2(removed, v, cv) {
        }
        Edge newEdge;
        Edge oldEdge1;
        Edge oldEdge2;
    };
}
#endif //STEINER_STEINERINSTANCE_H
