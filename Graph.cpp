//
// Created by aschidler on 1/22/20.
//

#include "Graph.h"

//TODO: Directed  version??
//TODO: Create iterator over edges? Return only if u < v for undirected.
void steiner::Graph::addEdge(node_id u, node_id v, cost_id cost) {
    auto un = addNode(u);
    auto vn = addNode(v);
    this->nb[un].insert(pair<node_id, cost_id>(vn, cost));
    this->nb[vn].insert(pair<node_id, cost_id>(un, cost));
}

node_id steiner::Graph::getNodeMapping(node_id externalId) {
    return nodeMap_.find(externalId)->second;
}

node_id steiner::Graph::addNode(node_id u) {
    auto result = nodeMap_.find(u);

    if (result == nodeMap_.end()) {
        nb.emplace_back();
        nodeMap_.insert(pair<node_id, node_id>(u, nb.size() - 1));
        nodeReverseMap_.insert(pair<node_id, node_id>(nb.size() - 1, u));
        nodes_.push_back(nb.size() - 1);
        return nb.size() - 1;
    }

    return result->second;
}

void steiner::Graph::findDistances() {
    distances_ = new cost_id*[getNumNodes()];
    for(size_t i=0; i < getNumNodes(); i++) {
        distances_[i] = nullptr;
    }

    for(auto u: nodes_) {
        findDistances(u);
    }
}

void steiner::Graph::findDistances(node_id u) {
    // Init distances
    if (distances_ == nullptr) {
        distances_ = new cost_id*[getNumNodes()];
        for(size_t i=0; i < getNumNodes(); i++) {
            distances_[i] = nullptr;
        }
    }
    if (distances_[u] == nullptr) {
        distances_[u] = new cost_id[getNumNodes()];
    }
    for(size_t i=0; i < getNumNodes(); i++) {
        distances_[u][i] = MAXCOST;
    }

    // We could initialize with other known distances...

    // Dijkstra
    auto q = priority_queue<NodeWithCost>();
    auto visited = unordered_set<node_id>();
    q.emplace(u, 0);
    distances_[u][u] = 0;

    while(not q.empty()) {
        auto elem = q.top();
        q.pop();

        // already visited...
        if(visited.find(elem.node) != visited.end())
            continue;

        visited.insert(elem.node);

        for (auto v: nb[elem.node]) {
            if (visited.find(v.first) == visited.end() && distances_[u][v.first] > elem.cost + v.second) {
                distances_[u][v.first] = elem.cost + v.second;
                q.emplace(v.first, elem.cost + v.second);
            }
        }
    }
}

steiner::Graph *steiner::Graph::copy() {
    auto* cp = new Graph();
    for (auto n: nodes_) {
        // In case only one node, or disconnected...
        cp->addNode(n);
        for (auto v: nb[n]) {
            cp->addEdge(n, v.first, v.second);
        }
    }

    return cp;
}
