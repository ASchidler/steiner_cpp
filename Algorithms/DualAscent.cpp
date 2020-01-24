//
// Created by aschidler on 1/24/20.
//

#include "DualAscent.h"
using namespace steiner;

bool DualAscent::hasRun = false;
unsigned int DualAscent::bestResult = UINT_MAX;
unsigned int DualAscent::bestRoot = 0;

DualAscentResult* steiner::DualAscent::calculate(Graph *g, unsigned int root, unordered_set<unsigned int>* ts) {
    Graph *dg = g->copy();
    unsigned int bound = 0;
    auto q = priority_queue<NodeWithCost>();
    auto active = unordered_set<unsigned int>();

    for (auto t: *ts) {
        active.insert(t);
        if (t != root)
            q.emplace(t, 0);
    }

    while (!q.empty()) { // main loop
        auto elem = q.top();
        q.pop();

        // Find cut
        vector<unsigned int> bfs_queue;
        bfs_queue.push_back(elem.node);
        vector<Edge> edges;
        unordered_set<unsigned int> cut;
        bool tFound = false;

        while (!bfs_queue.empty() && !tFound) { // cut loop
            auto n = bfs_queue.back();
            bfs_queue.pop_back();

            for (auto nb: dg->nb[n]) {
                if (cut.find(nb.first) == cut.end()) {
                    // TODO: Introduce pred dictionary into graph?
                    // Strictly speaking it is not necessary, as we always have symmetric directed graphs, but this causes cache missies
                    auto cost = dg->nb[nb.first][n];

                    // 0 means traversable edge
                    if (cost == 0) {
                        // Found active vertex? Stop, component is connected
                        if (active.find(nb.first) != active.end()) {
                            tFound = true;
                            break;
                        }
                        bfs_queue.push_back(nb.first);
                        cut.insert(nb.first);
                    } else {
                        edges.emplace_back(n, nb.first, cost);
                    }
                }
            }
        }// end cut loop

        // Cut is inactive? Ignore
        if (tFound) {
            active.erase(elem.node);
            continue;
        }

        unsigned int minCost = UINT_MAX;
        // TODO: Is this efficient?
        auto cEdge = edges.begin();
        // Find minimum cost and remove edges that are inside the cut
        while (cEdge != edges.end()) {
            if (cut.find(cEdge->v) != cut.end()) {
                edges.erase(cEdge);
            } else {
                if (cEdge->cost < minCost)
                    minCost = cEdge->cost;
                ++cEdge;
            }
        }

        // Check if the queue weight was about accurate
        if (! q.empty()) {
            auto elem2 = q.top();
            if (edges.size() > 1.25 * elem2.cost) {
                elem.cost = edges.size();
                q.push(elem);
                continue;
            }
        }

        // Increment bound
        bound += minCost;

        // Update weights
        unsigned int newWeight = 0;
        for (auto ce: edges) { // update weight loop
            dg->nb[ce.v][ce.u] -= minCost;
            // Is now zero
            if (ce.cost == minCost) {
                if (!tFound && active.find(ce.v) != active.end()) {
                    active.erase(ce.v);
                    tFound = true;
                }
                newWeight += g->nb[ce.v].size() - 1;
            }
        } // end update weight loop

        // Add back to queue
        if (! tFound) {
            elem.cost = newWeight;
            q.push(elem);
        }
    } // end main loop

    DualAscent::hasRun = true;
    if (bound < DualAscent::bestResult) {
        DualAscent::bestResult = bound;
        DualAscent::bestRoot = root;
    }
    return new DualAscentResult(bound, dg, root);
}