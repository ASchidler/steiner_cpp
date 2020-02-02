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
    bool* cut[nTerminals];
    vector<DualAscentEdge> edges[nTerminals];

    // Initialize active components and queue
    for(node_id t=0; t < nTerminals; t++) {
        if (ts == nullptr || t == root || !ts->test(t)) {
            active[t] = true;
            if (t != root) {
                q.emplace(t, 0);
                cut[t] = new bool[nNodes];
                for(node_id i=0; i < nNodes; i++)
                    cut[t][i] = false;
                for(auto& nb: dg->nb[t])
                    edges[t].emplace_back(nb.first, t, &(dg->nb[nb.first][t]));

                cut[t][t] = true;
            }
        } else {
            active[t] = false;
        }
    }

    // run until not active components
    while (!q.empty()) { // main loop
        auto elem = q.top();
        q.pop();

        // Find cut, i.e. vertices in the weakly connected component of t and edges leading in
        auto minCost = findCut(dg, elem.node, active, &(edges[elem.node]), cut[elem.node], nTerminals);

        // This is not necessary for correctness, but this ensures that the estimated weight is about right
        // and leads got generally better bounds
        if (! q.empty()) {
            auto elem2 = q.top();
            if (edges[elem.node].size() > elem2.cost) {
                elem.cost = edges[elem.node].size();
                q.push(elem);
                continue;
            }
        }

        // Min Cost 0 means hit an active component
        if (minCost > 0) {
            // Increment bound
            bound += minCost;

            // Update edge costs and estimate new weight, i.e. number of incoming edges
            node_id oldCount = edges[elem.node].size();
            for(size_t i=0; i < oldCount; i++) {
                auto& ce = edges[elem.node][i];
                *ce.c -= minCost;

                // Is now zero, i.e. is part of the component
//                if (*ce.c == 0) {
//                    if (ce.u < nTerminals && active[ce.u]) {
//                        // TODO: delete cut...
//                        active[elem.node] = false; // Found active component
//                    } else if(active[elem.node] && !cut[elem.node][ce.u]) {
//                        cut[elem.node][ce.u] = true;
//                        node_id u =ce.u;
//                        for(auto& nb: dg->nb[ce.u]) {
//                            if (! cut[elem.node][nb.first]) {
//                                edges[elem.node].emplace_back(nb.first, u, &(dg->nb[nb.first][u]));
//                            }
//                        }
//                        // This will decrement newly added edges...
////                        swap(edges[elem.node][i], edges[elem.node].back());
////                        edges[elem.node].pop_back();
////                        i--;
//                    }
//                }
            }

            // Add back to queue
            if (active[elem.node]) {
                elem.cost = edges[elem.node].size();
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

cost_id DualAscent::findCut(Graph *dg, node_id n, bool* active, vector<DualAscentEdge> *edges, bool* cut, node_id nTerminals) {
    vector<node_id> bfs_queue;

    // Find "unexpected" new nodes for cut
    for(auto& e: *edges) {
        if (!cut[e.u] && *e.c == 0) {
            bfs_queue.push_back(e.u);
            if (e.u < nTerminals && active[e.u]) {
                active[n] = false;
                return 0;
            }

            cut[e.u] = true;
        }
    }

// TODO: Read all edges and than use references to the edge, to avoid costly hashtable lookup?
    while (!bfs_queue.empty()) { // cut loop
        auto v = bfs_queue.back();
        bfs_queue.pop_back();

        for (auto u: dg->nb[v]) {
            if (!cut[u.first]) {
                // Strictly speaking a pred relation is not necessary, as we always have symmetric directed graphs, but this causes cache misses (I guess)
                // The question is, if the stuff below would cause them anyways...
                auto cost = &(dg->nb[u.first][v]);

                // 0 means traversable edge
                if (*cost == 0) {
                    // Found active vertex? Stop, component is connected
                    if (u.first < nTerminals && active[u.first]) {
                        active[n] = false;
                        // TODO: delete cut...
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
        if (*cEdge.c == 0 || cut[cEdge.u]) {
            swap((*edges)[i], edges->back());
            edges->pop_back();
            i--;
        } else {
            minCost = min(minCost, *cEdge.c);
        }
    }

    return minCost;
}

