//
// Created by aschidler on 1/22/20.
//

#ifndef STEINER_STEINERINSTANCE_H
#define STEINER_STEINERINSTANCE_H
#include "Graph.h"
#include <bits/stdc++.h>

#include <utility>

using namespace std;

namespace steiner {
    class SteinerInstance {
    public:
        SteinerInstance(Graph *g, vector<node_id> *terminals);
        ~SteinerInstance() {
            for(int n=0; n < g_->getMaxNode(); n++) {
                if (closest_terminals_[n] != nullptr)
                    delete[] closest_terminals_[n];
            }
            delete[] closest_terminals_;

            if (terminalSteinerDistances_ != nullptr) {
                for (int i=0; i < maxTerminals; i++) {
                    delete[] terminalSteinerDistances_[i];
                }
                delete[] terminalSteinerDistances_;
            }
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
        NodeWithCost* getClosestTerminals(node_id n);
        cost_id getDistance(node_id n1, node_id n2);
        void moveTerminal(node_id t, node_id target);

        cost_id getSteinerDistance(node_id u, node_id v);
        enum ValueState { lower, exact, higher, invalid};

        void setDistanceState(ValueState s) {
            if (distanceState_ == exact)
                distanceState_ = s;
            if ((distanceState_ == lower && s == higher) || (distanceState_ == higher && s == lower))
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
            if (steinerDistanceState_ == exact)
                steinerDistanceState_ = s;
            if ((steinerDistanceState_ == lower && s == higher) || (steinerDistanceState_ == higher && s == lower))
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
        void checkGraphIntegrity() {
            for (auto n: *g_->getNodes()) {
                for(auto v: g_->nb[n]) {
                    if (g_->nb[v.first].count(n) == 0)
                        cout << "ERROR, one sided" << endl;
                    else if (g_->nb[v.first][n] != v.second) {
                        cout << "ERROR! " << v.first << " " << n << " have different costs" << endl;
                    }
                }

            }
        }
    private:
        Graph *g_;
        NodeWithCost** closest_terminals_ = nullptr;
        cost_id** terminalSteinerDistances_ = nullptr;
        node_id nTerminals = 0;
        node_id maxTerminals = 0;

        inline node_id removeNode_(node_id u);
        void calculateSteinerDistance();

        ValueState distanceState_ = invalid;
        ValueState steinerDistanceState_ = invalid;
        ValueState approximationState_ = invalid;


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
