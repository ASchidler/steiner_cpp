//
// Created by aschidler on 1/24/20.
//

#include "ShortestPath.h"

//TODO: Remove non-terminal leafs and compute mst
//TODO: local improvement

bool steiner::ShortestPath::hasRun = false;
node_id steiner::ShortestPath::bestRoot = 0;
cost_id steiner::ShortestPath::bestResult = MAXCOST;

steiner::SteinerTree *steiner::ShortestPath::calculate(node_id root, Graph* g, node_id nTerminals, node_id nNodes) {
    auto result = new SteinerTree(root);
    node_id nRemaining = nTerminals;
    bool remaining[nTerminals];
    cost_id costs[nNodes];
    bool added[nNodes];
    node_id prev[nNodes];

    priority_queue<SPHEntry> q;

    for(int i=0; i < nTerminals; i++)
        remaining[i] = true;

    for(int i=0; i < nNodes; i++) {
        costs[i] = MAXCOST;
        added[i] = false;
    }

    if (root < nTerminals) {
        nRemaining--;
        remaining[root] = false;
    }
    added[root] = true;
    q.emplace(root, 0, 0);

    while (nRemaining > 0) {
        auto elem = q.top();
        q.pop();

        if (elem.node < nTerminals && remaining[elem.node]) {
            remaining[elem.node] = false;
            nRemaining--;

            // Backtrack path
            node_id cNode = elem.node;
            node_id cPred = prev[elem.node];
            vector<node_id> cache;
            while (!added[cNode]) {
                assert(g->nb[cNode].count(cPred) > 0);
                result->addEdge(cNode, cPred, g->nb[cNode][cPred]);
                added[cNode] = true;
                cache.push_back(cNode);

                cNode = cPred;
                cPred = prev[cNode];
            }
            // Reexpand with 0 base cost (separate step, otherwise prev will be overwritten while path finding)
            for(auto c: cache) {
                for(auto& v: g->nb[c]) {
                    if (v.second < costs[v.first] && !added[v.first]) {
                        q.emplace(v.first, v.second, v.second);
                        costs[v.first] = v.second;
                        prev[v.first] = c;
                    }
                }
            }

        } else {
            // Normal Dijkstra expand
            for(auto& v: g->nb[elem.node]) {
                auto total = elem.totalCost + v.second;
                if (total < costs[v.first] && !added[v.first]) {
                    q.emplace(v.first, total, v.second);
                    costs[v.first] = total;
                    prev[v.first] = elem.node;
                }
            }
        }
    }

    ShortestPath::hasRun = true;
    if (ShortestPath::bestResult > result->getCost()) {
        ShortestPath::bestResult = result->getCost();
        if (root < nTerminals)
            ShortestPath::bestRoot = root;
    }
    return result;
}
