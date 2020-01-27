//
// Created by aschidler on 1/24/20.
//

#include "DualAscent.h"
using namespace steiner;

bool DualAscent::hasRun = false;
cost_id DualAscent::bestResult = 0;
node_id DualAscent::bestRoot = 0;

// TODO: Implement the 2 other variants... at least number 3 -> More exact but slower

DualAscentResult* steiner::DualAscent::calculate(Graph *g, node_id root, unordered_set<node_id>* ts) {
    Graph *dg = g->copy(false);
    unsigned int bound = 0;
    auto q = priority_queue<NodeWithCost>();
    auto active = unordered_set<node_id>();

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
        unordered_set<node_id> cut;
        vector<Edge> edges; // TODO: Reserve with the weight may improve here.
        auto minCost = findCut(dg, elem.node, &active, &edges, &cut);

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

        // Min Cost 0 means hit an active component
        if (minCost > 0) {
            // Increment bound
            bound += minCost;

            // Update edge costs and estimate new weight, i.e. number of incoming edges
            cost_id newWeight = 0;
            bool tFound = false;
            for (auto ce: edges) { // update weight loop
                dg->nb[ce.u][ce.v] -= minCost;

                // Is now zero, i.e. is part of the component
                if (ce.cost == minCost) {
                    tFound = tFound || active.count(ce.u) > 0; // Do not stop here, finish updating the weights!
                    newWeight += g->nb[ce.u].size() - 1; // Estimate incoming edges gained from this node
                }
            } // end update weight loop

            // Add back to queue
            if (!tFound) {
                elem.cost = newWeight + edges.size();
                q.push(elem);
            } else {
                active.erase(elem.node);
            }
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

cost_id DualAscent::findCut(Graph *dg, node_id n, unordered_set<node_id> *active, vector<Edge> *edges, unordered_set<node_id>* cut) {
    vector<node_id> bfs_queue;
    bfs_queue.push_back(n); // The way it works its dfs... (always picking last first)
    cut->insert(n);

    while (!bfs_queue.empty()) { // cut loop
        auto v = bfs_queue.back();
        bfs_queue.pop_back();

        for (auto u: dg->nb[v]) {
            if (cut->count(u.first) == 0) {
                // Strictly speaking a pred relation is not necessary, as we always have symmetric directed graphs, but this causes cache misses (I guess)
                // The question is, if the stuff below would cause them anyways...
                auto cost = dg->nb[u.first][v];

                // 0 means traversable edge
                if (cost == 0) {
                    // Found active vertex? Stop, component is connected
                    if (active->count(u.first) > 0) {
                        active->erase(n);
                        return 0;
                    }
                    bfs_queue.push_back(u.first);
                    cut->insert(u.first);
                } else {
                    edges->emplace_back(u.first, v, cost);
                }
            }
        }
    }// end cut loop

    // Find minimum cost and remove edges that are inside the cut
    cost_id minCost = MAXCOST;
    auto cEdge = edges->begin();

    while (cEdge != edges->end()) {
        if (cut->count(cEdge->u) > 0) {
            edges->erase(cEdge);
        } else {
            if (cEdge->cost < minCost)
                minCost = cEdge->cost;
            ++cEdge;
        }
    }

    return minCost;
}

