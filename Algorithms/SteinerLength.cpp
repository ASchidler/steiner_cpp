//
// Created by aschidler on 1/28/20.
//

#include "SteinerLength.h"

cost_id steiner::SteinerLength::calculateSteinerLength(node_id u, node_id v, Graph* g, cost_id cut_off, node_id depth_limit,
                                                       bool restrict, node_id nTerminals, node_id nNodes) {
    cost_id scanned1[nNodes];
    cost_id scanned2[nNodes];
    calculateSteinerLengthSub(u, v, g, cut_off, depth_limit, nTerminals, nNodes, scanned1);
    calculateSteinerLengthSub(v, u, g, cut_off, depth_limit, nTerminals, nNodes, scanned2);

    // If we found the other node use this dist. Scanned != Visited, therefore values may differ!
    cost_id sd = min(scanned1[v], scanned2[u]);

    for(node_id i=0; i < nNodes; i++) {
        if (scanned1[i] < MAXCOST && scanned2[i] < MAXCOST) {
            if (i < nTerminals) {
                sd = min(sd, max(scanned1[i], scanned2[i]));
            } else {
                sd = min(sd, scanned1[i] + scanned2[i]);
            }
        }
    }

    if (! restrict && g->nb[u].count(v) > 0) {
        if (sd > g->nb[u][v])
            sd = g->nb[u][v];
    }
    return sd;
}

 void steiner::SteinerLength::calculateSteinerLengthSub(node_id u, node_id v, Graph* g, cost_id cut_off, node_id depth_limit,
                                                        node_id nTerminals, node_id nNodes, cost_id* scanned) {
    for (node_id i=0; i < nNodes; i++) {
        scanned[i] = MAXCOST;
    }
    scanned[u] = 0;
    node_id scannedEdges = 0;
    priority_queue<NodeWithCost> q;

    for(auto& n: g->nb[u]) {
        if (n.first != v) { // Avoid the edge u,v
            q.emplace(n.first, n.second);
            scanned[n.first] = n.second;
        }
    }

    while (! q.empty() && scannedEdges < depth_limit) {
        auto elem = q.top();
        q.pop();
        assert(elem.node != u);
        // Stop on u, v and terminals or exceed cutoff
        if (elem.node < nTerminals || elem.node == u || elem.node == v)
            break;
        if (elem.cost > scanned[elem.node])
            continue;

        for(auto& n: g->nb[elem.node]) {
            if (scannedEdges++ > depth_limit)
                break;

            auto total = elem.cost + n.second;
            if (total < scanned[n.first] && total <= cut_off) {
                scanned[n.first] = total;
                q.emplace(n.first, total);
            }
        }
    }
}
