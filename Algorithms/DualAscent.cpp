//
// Created by aschidler on 1/24/20.
//

#include "DualAscent.h"
using namespace steiner;

bool DualAscent::hasRun = false;
unsigned int DualAscent::bestResult = 0;
unsigned int DualAscent::bestRoot = 0;

// TODO: Implement the 3 other variants...

DualAscentResult* steiner::DualAscent::calculate(Graph *g, unsigned int root, unordered_set<unsigned int>* ts) {
    Graph *dg = g->copy();
    unsigned int bound = 0;
    auto q = priority_queue<NodeWithCost>();
    auto active = unordered_set<unsigned int>();

    // Initialize active components and queue
    for (auto t: *ts) {
        active.insert(t);
        if (t != root)
            q.emplace(t, 0);
    }

    // run until not active components
    while (!q.empty()) { // main loop
        auto elem = q.top();
        q.pop();

        // Find cut, i.e. vertices in the weakly connected component of t and edges leading in
        vector<unsigned int> bfs_queue;
        bfs_queue.push_back(elem.node);
        unordered_set<unsigned int> cut;
        cut.insert(elem.node);

        vector<Edge> edges;
        edges.reserve(elem.cost); //Size according to cost, as this should approx fit.
        bool tFound = false;

        while (!bfs_queue.empty() && !tFound) { // cut loop
            auto v = bfs_queue.back();
            bfs_queue.pop_back();

            for (auto u: dg->nb[v]) {
                if (cut.find(u.first) == cut.end()) {
                    // TODO: Introduce pred dictionary into graph?
                    // Strictly speaking it is not necessary, as we always have symmetric directed graphs, but this causes cache misses (I guess)
                    auto cost = dg->nb[u.first][v];

                    // 0 means traversable edge
                    if (cost == 0) {
                        // Found active vertex? Stop, component is connected
                        if (active.find(u.first) != active.end()) {
                            tFound = true;
                            break;
                        }
                        bfs_queue.push_back(u.first);
                        cut.insert(u.first);
                    } else {
                        edges.emplace_back(u.first, v, cost);
                    }
                }
            }
        }// end cut loop

        // Cut is inactive? Ignore
        if (tFound) {
            active.erase(elem.node);
            continue;
        }

        // Find minimum cost and remove edges that are inside the cut
        // TODO: Is this efficient?
        unsigned int minCost = UINT_MAX;
        auto cEdge = edges.begin();

        while (cEdge != edges.end()) {
            if (cut.find(cEdge->u) != cut.end()) {
                edges.erase(cEdge);
            } else {
                if (cEdge->cost < minCost)
                    minCost = cEdge->cost;
                ++cEdge;
            }
        }

        // This is not necessary for correctness, but this ensures that the estimated weight is about right
        // and leads got generally better bounds
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

        // Update edge costs and estimate new weight, i.e. number of incoming edges
        unsigned int newWeight = 0;
        for (auto ce: edges) { // update weight loop
            dg->nb[ce.u][ce.v] -= minCost;

            // Is now zero
            if (ce.cost == minCost) {
                if (active.find(ce.u) != active.end()) {
                    active.erase(elem.node);
                    tFound = true;
                    break;
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

    // Store best result and especially best root
    DualAscent::hasRun = true;
    if (bound > DualAscent::bestResult) {
        DualAscent::bestResult = bound;
        DualAscent::bestRoot = root;
    }
    return new DualAscentResult(bound, dg, root);
}