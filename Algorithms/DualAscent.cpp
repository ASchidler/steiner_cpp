//
// Created by aschidler on 1/24/20.
//

#include "DualAscent.h"
#include "../Structures/Queue.h"
using namespace steiner;

bool DualAscent::hasRun = false;
cost_id DualAscent::bestResult = 0;
node_id DualAscent::bestRoot = 0;

SteinerResult* steiner::DualAscent::calculate(Graph *g, node_id root, const dynamic_bitset<>* ts, node_id nTerminals, node_id nNodes) {
    auto dg = new Graph(*g, false);
    unsigned int bound = 0;

    auto q = Queue<NodeWithCost>(g->getOriginalNumEdges());
    bool active[nTerminals];
    bool* cut[nTerminals];
    vector<DualAscentEdge> edges[nTerminals];

    // Initialize active components and queue
    for(node_id t=0; t < nTerminals; t++) {
        cut[t] = nullptr;
        if (ts == nullptr || t == root || !ts->test(t)) {
            active[t] = true;
            if (t != root) {
                q.emplace(dg->nb[t].size(), t, dg->nb[t].size());
                cut[t] = new bool[nNodes];
                for(node_id i=0; i < nNodes; i++)
                    cut[t][i] = false;
                for(auto& nb: dg->nb[t]) {
                    edges[t].emplace_back(nb.first, t, &(dg->nb[nb.first][t]));
                }

                cut[t][t] = true;
            }
        } else {
            active[t] = false;
        }
    }

    // run until not active components
    while (!q.empty()) { // main loop
        auto elem = q.dequeue();

        // Find cut, i.e. vertices in the weakly connected component of t and edges leading in
        auto minCost = findCut(*dg, elem.node, active, edges[elem.node], cut[elem.node], nTerminals);

        // Min Cost 0 means hit an active component
        if (minCost > 0) {
            // This is not necessary for correctness, but this ensures that the estimated weight is about right
            // and leads got generally better bounds
            if (!q.empty()) {
                auto& elem2 = q.peek();
                if (edges[elem.node].size() > elem2.cost) {
                    elem.cost = edges[elem.node].size();
                    q.push(elem.cost, elem);
                    continue;
                }
            }
            // Increment bound
            bound += minCost;
            cost_id cost = 0;

            // Update edge costs and estimate new weight, i.e. number of incoming edges
            for(auto& ce: edges[elem.node]) {
                if ((*ce.c -= minCost) == 0) { // subtract and check if traversable
                    cost++;
                    if (ce.u < nTerminals && active[ce.u]) {
                        active[elem.node] = false;
                    }
                }
            }

            // Add back to queue
            if (active[elem.node]) {
                elem.cost = edges[elem.node].size() + cost;
                q.push(elem.cost, elem);
            }
        }
    } // end main loop

    for(node_id t=0; t < nTerminals; t++) {
        delete[] cut[t];
    }

    // Store best result and especially best root
    DualAscent::hasRun = true;
    if (bound > DualAscent::bestResult) {
        DualAscent::bestResult = bound;
        if (root < nTerminals)
            DualAscent::bestRoot = root;
    }

    return new SteinerResult(bound, dg, root);
}

cost_id DualAscent::findCut(Graph& dg, node_id n, bool* active, vector<DualAscentEdge>& edges, bool* cut, node_id nTerminals) {
    vector<node_id> bfs_queue;
    bfs_queue.reserve(edges.size());

    // Find new nodes for the cut and then trace them
    for(auto& e: edges) {
        if (!cut[e.u] && *e.c == 0) {
            bfs_queue.push_back(e.u);
            cut[e.u] = true;
            if (e.u < nTerminals && active[e.u]) {
                active[n] = false;
                return 0;
            }
            // The vector will be cleaned of unnecessary edges below
        }
    }

    while (!bfs_queue.empty()) { // cut loop
        auto v = bfs_queue.back();
        bfs_queue.pop_back();

        for (auto u: dg.nb[v]) {
            if (!cut[u.first]) {
                // get the pred distance
                auto cost = &(dg.nb[u.first][v]);

                // 0 means traversable edge
                if (*cost == 0) {
                    // Found active vertex? Stop, component is connected
                    if (u.first < nTerminals && active[u.first]) {
                        active[n] = false;
                        return 0;
                    }
                    bfs_queue.push_back(u.first);
                    cut[u.first] = true;
                } else {
                    edges.emplace_back(u.first, v, cost);
                }
            }
        }
    }// end cut loop

    // Find minimum cost and remove edges that are inside the cut
    cost_id minCost = MAXCOST;
    for(size_t i=0; i < edges.size(); i++) {
        auto& cEdge = edges[i];
        if (cut[cEdge.u]) { // This automatically includes zero cost edges
            swap(edges[i], edges.back()); // Popping from the back is a lot more efficient
            edges.pop_back();
            i--;
        } else {
            minCost = min(minCost, *(cEdge.c));
        }
    }

    return minCost;
}

