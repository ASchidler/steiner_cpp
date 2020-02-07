//
// Created by aschidler on 1/30/20.
//

#ifndef STEINER_DUALASCENTREDUCTION_H
#define STEINER_DUALASCENTREDUCTION_H
#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"
#include "../Algorithms/DualAscent.h"
#include <random>

namespace  steiner {
    class DualAscentReduction : public Reduction {
    public:
        explicit DualAscentReduction(SteinerInstance *s) : Reduction(s) {
        }

        node_id reduce(node_id currCount, node_id prevCount) override;

        string getName() override {
            return "Dual Ascent";
        }
    private:
        node_id reduceGraph(SteinerResult* r);
        node_id bestRoots[2] = {0, 1};

        void chooseRoots(node_id* roots, node_id numRoots);
        void selectRoots(SteinerResult** results, node_id numSolutions, const node_id* track);
        struct Voronoi {
            Voronoi(node_id nTerminals, node_id nNodes) {
                closest = new vector<NodeWithCost>[nTerminals];
                second.resize(nNodes);
            }
            ~Voronoi() {
                delete[] closest;
            }
            vector<NodeWithCost>* closest;
            vector<NodeWithCost> second;
        };
        // TODO: We can switch it around and look at the closest to the node, would ease edge calculation
        static inline Voronoi* voronoi(Graph* g, node_id nTerminals) {
            auto result = new Voronoi(nTerminals, g->getMaxNode());

            node_id visited[g->getMaxNode()];
            node_id visited2[g->getMaxNode()];
            priority_queue<DoubleNodeEntry> q;

            for(node_id t=0; t < nTerminals; t++)
                q.emplace(t, t, 0);
            for(node_id n=0; n < g->getMaxNode(); n++) {
                visited[n] = false;
                visited2[n] = false;
            }

            while(!q.empty()) {
                auto elem = q.top();
                q.pop();

                if (visited2[elem.n1]) {
                    continue;
                }
                else if (visited[elem.n1]) {
                    result->second[elem.n1].node = elem.n2;
                    result->second[elem.n1].cost = elem.cost;
                    visited2[elem.n1] = true;
                } else {
                    result->closest[elem.n2].emplace_back(elem.n1, elem.cost);
                    visited[elem.n1] = true;
                }

                for (auto& v: g->nb[elem.n1]) {
                    if(! visited2[v.first]) {
                        q.emplace(v.first, elem.n2, elem.cost + g->nb[v.first][elem.n1]);
                    }
                }
            }
            return result;
        }
    };
}


#endif //STEINER_DUALASCENTREDUCTION_H
