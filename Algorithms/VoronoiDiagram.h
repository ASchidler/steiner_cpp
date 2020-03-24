//
// Created by andre on 22.03.20.
//

#ifndef STEINER_VORONOIDIAGRAM_H
#define STEINER_VORONOIDIAGRAM_H
#include "../Steiner.h"
#include "../Graph.h"

namespace steiner{
    struct VoronoiDiagram {
        VoronoiDiagram(node_id nTerminals, node_id nNodes) {
            closest.resize(nNodes);
            second.resize(nNodes);
        }

        vector<NodeWithCost> closest;
        vector<NodeWithCost> second;

        cost_id getRadiusSum(node_id nTerminals) {
            cost_id sum = 0;
            cost_id max1 = 0;
            cost_id max2 = 0;

            for (node_id t=0; t < nTerminals; t++) {
                auto cCost = second[t].cost;
                sum += cCost;

                if (cCost >= max1) {
                    max2 = max1;
                    max1 = cCost;
                } else if (cCost > max2) {
                    max2 = cCost;
                }
            }

            return sum - max1 - max2;
        }

        static VoronoiDiagram* create(Graph* g, node_id nTerminals) {
            auto result = new VoronoiDiagram(nTerminals, g->getMaxNode());

            node_id visited[g->getMaxNode()];
            node_id visited2[g->getMaxNode()];
            priority_queue<DoubleNodeEntry> q;

            for(node_id t=0; t < nTerminals; t++)
                q.emplace(t, t, 0);
            for(node_id n=0; n < g->getMaxNode(); n++) {
                visited[n] = g->getMaxNode();
                visited2[n] = g->getMaxNode();
            }

            while(!q.empty()) {
                auto elem = q.top();
                q.pop();

                // Skip done vertices, and do not use the closest terminal as second closest
                if (visited2[elem.n1] != g->getMaxNode() || visited[elem.n1] == elem.n2) {
                    continue;
                }
                    // We already found the closest terminal, and it is a different one than the current
                else if (visited[elem.n1] != g->getMaxNode()) {
                    result->second[elem.n1].node = elem.n2;
                    result->second[elem.n1].cost = elem.cost;
                    visited2[elem.n1] = elem.n2;
                    // First terminal arriving at this node, set closest
                } else {
                    result->closest[elem.n1].node = elem.n2;
                    result->closest[elem.n1].cost = elem.cost;
                    visited[elem.n1] = elem.n2;
                }

                for (auto& v: g->nb[elem.n1]) {
                    if(elem.n2 != visited[v.first] && visited2[v.first] == g->getMaxNode()) {
                        q.emplace(v.first, elem.n2, elem.cost + g->nb[v.first][elem.n1]);
                    }
                }
            }
            return result;
        }
    };
}
#endif //STEINER_VORONOIDIAGRAM_H
