//
// Created by aschidler on 1/30/20.
//

#ifndef STEINER_DUALASCENTREDUCTION_H
#define STEINER_DUALASCENTREDUCTION_H
#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"
#include "../Algorithms/DualAscent.h"

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
        node_id reduceGraph(DualAscentResult* r);

        static inline vector<NodeWithCost>* voronoi(Graph* g, node_id nTerminals) {
            auto vor = new vector<NodeWithCost>[nTerminals];
            node_id visited[g->getMaxNode()];
            priority_queue<DoubleNodeEntry> q;
            node_id nVisited = 0;

            for(node_id t=0; t < nTerminals; t++)
                q.emplace(t, t, 0);
            for(node_id n=0; n < g->getMaxNode(); n++)
                visited[n] = false;

            while(nVisited < g->getNumNodes()) {
                auto elem = q.top();
                q.pop();

                if (visited[elem.n1])
                    continue;

                vor[elem.n2].emplace_back(elem.n1, elem.cost);
                visited[elem.n1] = true;
                nVisited++;

                for (auto& v: g->nb[elem.n1]) {
                    if(! visited[v.first]) {
                        q.emplace(v.first, elem.n2, elem.cost + g->nb[v.first][elem.n1]);
                    }
                }
            }
            return vor;
        }
    };
}


#endif //STEINER_DUALASCENTREDUCTION_H
