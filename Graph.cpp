//
// Created by aschidler on 1/22/20.
//

#include "Graph.h"

void steiner::Graph::addEdge(unsigned int u, unsigned int v, unsigned int cost) {
    auto un = addVertex(u);
    auto vn = addVertex(v);
    this->nb[un].emplace_back(vn, cost);
    this->nb[vn].emplace_back(un, cost);
}

unsigned int steiner::Graph::getNodeMapping(unsigned int externalId) {
    return nodeMap_.find(externalId)->second;
}

unsigned int steiner::Graph::addVertex(unsigned int u) {
    auto result = nodeMap_.find(u);

    if (result == nodeMap_.end()) {
        nodes_.push_back(u);
        nb.emplace_back();
        nodeMap_.insert(pair<unsigned int, unsigned int>(u, maxNodeId_));
        maxNodeId_++;
        return maxNodeId_ - 1;
    }

    return result->second;
}

void steiner::Graph::findDistances() {
    distances_ = new unsigned int*[getNumNodes()];
    for(size_t i=0; i < getNumNodes(); i++) {
        distances_[i] = nullptr;
    }

    for(auto u: nodes_) {
        findDistances(u);
    }
}

void steiner::Graph::findDistances(unsigned int u) {
    // Init distances
    if (distances_ == nullptr) {
        distances_ = new unsigned int*[getNumNodes()];
        for(size_t i=0; i < getNumNodes(); i++) {
            distances_[i] = nullptr;
        }
    }
    if (distances_[u] == nullptr) {
        distances_[u] = new unsigned int[getNumNodes()];
    }
    for(size_t i=0; i < getNumNodes(); i++) {
        distances_[u][i] = UINT_MAX;
    }

    // We could initialize with other known distances...

    // Dijkstra
    auto q = priority_queue<Neighbor>();
    auto visited = unordered_set<unsigned int>();
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
            if (visited.find(v.node) == visited.end() && distances_[u][v.node] > elem.cost + v.cost) {
                distances_[u][v.node] = elem.cost + v.cost;
                q.emplace(v.node, elem.cost + v.cost);
            }
        }
    }
}