//
// Created by aschidler on 1/22/20.
//

#include "Graph.h"

//TODO: Directed  version??
//TODO: Create iterator over edges? Return only if u < v for undirected.
void steiner::Graph::addEdge(unsigned int u, unsigned int v, unsigned int cost) {
    auto un = addNode(u);
    auto vn = addNode(v);
    this->nb[un].insert(pair<unsigned int, unsigned int>(vn, cost));
    this->nb[vn].insert(pair<unsigned int, unsigned int>(un, cost));
}

unsigned int steiner::Graph::getNodeMapping(unsigned int externalId) {
    return nodeMap_.find(externalId)->second;
}

unsigned int steiner::Graph::addNode(unsigned int u) {
    auto result = nodeMap_.find(u);

    if (result == nodeMap_.end()) {
        nb.emplace_back();
        nodeMap_.insert(pair<unsigned int, unsigned int>(u, maxNodeId_));
        nodes_.push_back(maxNodeId_);
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
