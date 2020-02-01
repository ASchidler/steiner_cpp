//
// Created by aschidler on 1/24/20.
//

#include "DualAscent.h"
using namespace steiner;

bool DualAscent::hasRun = false;
cost_id DualAscent::bestResult = 0;
node_id DualAscent::bestRoot = 0;

// TODO: Implement the 2 other variants... at least number 3 -> More exact but slower
// TODO: Shrink graph before solving to get rid of excessive data structures...
DualAscentResult* steiner::DualAscent::calculate(Graph *g, node_id root, const dynamic_bitset<>* ts, node_id nTerminals, node_id nNodes) {
    Graph *dg = g->copy(false);
    unsigned int bound = 0;

    auto q = priority_queue<NodeWithCost>();
    bool active[nTerminals];
    bool cut[nNodes];

    // Initialize active components and queue
    for(node_id t=0; t < nTerminals; t++) {
        if (ts == nullptr || t == root || !ts->test(t)) {
            active[t] = true;
            if (t != root)
                q.emplace(t, 0);
        } else {
            active[t] = false;
        }
    }

    // run until not active components
    while (!q.empty()) { // main loop
        auto elem = q.top();
        q.pop();

        // Find cut, i.e. vertices in the weakly connected component of t and edges leading in
        for(node_id i=0; i < nNodes; i++)
            cut[i] = false;

        vector<Edge> edges;
        edges.reserve(elem.cost * 1.5);
        auto minCost = findCut(dg, elem.node, active, &edges, cut, nTerminals);

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
            for (auto& ce: edges) { // update weight loop
                dg->nb[ce.u][ce.v] -= minCost;

                // Is now zero, i.e. is part of the component
                if (ce.cost == minCost) {
                    if (ce.u < nTerminals && active[ce.u])
                        active[elem.node] = false; // Found active component
                    newWeight += g->nb[ce.u].size() - 1; // Estimate incoming edges gained from this node
                }
            } // end update weight loop

            // Add back to queue
            if (active[elem.node]) {
                elem.cost = newWeight + edges.size();
                q.push(elem);
            }
        }
    } // end main loop

    // Store best result and especially best root
    DualAscent::hasRun = true;
    if (bound > DualAscent::bestResult) {
        DualAscent::bestResult = bound;
        if (root < nTerminals)
            DualAscent::bestRoot = root;
    }

    return new DualAscentResult(bound, dg, root);
}

cost_id DualAscent::findCut(Graph *dg, node_id n, bool* active, vector<Edge> *edges, bool* cut, node_id nTerminals) {
    vector<node_id> bfs_queue;
    bfs_queue.push_back(n); // The way it works its dfs... (always picking last first)
    cut[n] = true;

    while (!bfs_queue.empty()) { // cut loop
        auto v = bfs_queue.back();
        bfs_queue.pop_back();

        for (auto u: dg->nb[v]) {
            if (!cut[u.first]) {
                // Strictly speaking a pred relation is not necessary, as we always have symmetric directed graphs, but this causes cache misses (I guess)
                // The question is, if the stuff below would cause them anyways...
                auto cost = dg->nb[u.first][v];

                // 0 means traversable edge
                if (cost == 0) {
                    // Found active vertex? Stop, component is connected
                    if (u.first < nTerminals && active[u.first]) {
                        active[n] = false;
                        return 0;
                    }
                    bfs_queue.push_back(u.first);
                    cut[u.first] = true;
                } else {
                    edges->emplace_back(u.first, v, cost);
                }
            }
        }
    }// end cut loop

    // Find minimum cost and remove edges that are inside the cut
    cost_id minCost = MAXCOST;
    for(size_t i=0; i < edges->size(); i++) {
        auto& cEdge = edges->at(i);
        if (cut[cEdge.u]) {
            swap((*edges)[i], edges->back());
            edges->pop_back();
            i--;
        } else {
            minCost = min(minCost, cEdge.cost);
        }
    }
//
//    auto cEdge = edges->begin();
//    while (cEdge != edges->end()) {
//        if (cut[cEdge->u]) {
//            edges->erase(cEdge);
//        } else {
//            if (cEdge->cost < minCost)
//                minCost = cEdge->cost;
//            ++cEdge;
//        }
//    }

    return minCost;
}

