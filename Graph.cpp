//
// Created by aschidler on 1/22/20.
//

#include "Graph.h"
using namespace steiner;

//TODO: Create iterator over edges? Return only if u < v for undirected.
bool steiner::Graph::addEdge(node_id u, node_id v, cost_id cost) {
    auto un = addNode(u);
    auto vn = addNode(v);

    if (nb[un].count(vn) == 0 || nb[un][vn] > cost) {
        this->nb[un].insert(pair<node_id, cost_id>(vn, cost));
        this->nb[vn].insert(pair<node_id, cost_id>(un, cost));

        return true;
    }

    return false;
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
        nodes_.insert(nb.size() - 1);
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
    node_id visited[getNumNodes()];
    for(size_t i=0; i < getNumNodes(); i++) {
        distances_[u][i] = MAXCOST;
        visited[i] = false;
    }

    // We could initialize with other known distances...

    // Dijkstra
    auto q = priority_queue<NodeWithCost>();

    q.emplace(u, 0);
    distances_[u][u] = 0;

    while(not q.empty()) {
        auto elem = q.top();
        q.pop();

        // already visited...
        if(visited[elem.node])
            continue;

        visited[elem.node] = true;

        for (auto v: nb[elem.node]) {
            if (!visited[v.first] && distances_[u][v.first] > elem.cost + v.second) {
                distances_[u][v.first] = elem.cost + v.second;
                q.emplace(v.first, elem.cost + v.second);
            }
        }
    }
}

steiner::Graph *steiner::Graph::copy(bool copyMapping) {
    auto* cp = new Graph();
    // Add nodes first, guarantees that the mapping stays the same
    cp->nodes_ = nodes_;
    for(const auto& n: nb) {
        cp->nb.push_back(n);
    }

    if (copyMapping) {
        cp->nodeMap_ = nodeMap_;
        cp->nodeReverseMap_ = nodeReverseMap_;
    }

    return cp;
}

void steiner::Graph::removeNode(node_id u) {
    for(auto elem: nb[u]) {
        nb[elem.first].erase(u);
    }
    nb[u].clear();
    nodes_.erase(u);
}

void Graph::removeEdge(node_id u, node_id v) {
    nb[u].erase(v);
    nb[v].erase(u);
}

void Graph::contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result) {
    vector<Edge> ret;
    for(auto n: nb[remove]) {
        if (n.first != target) {
            if (addEdge(target, n.first, n.second) && result != nullptr) {
                result->emplace_back(remove, target, n.first, n.second);
            }
        }
    }

    removeNode(remove);
}

void Graph::switchVertices(node_id n1, node_id n2) {
    auto on1 = nodeReverseMap_[n1];
    auto on2 = nodeReverseMap_[n2];
    nodeMap_[on1] = n2;
    nodeMap_[on2] = n1;
    nodeReverseMap_[n1] = on2;
    nodeReverseMap_[n2] = on1;

    auto tmp = nb[n1];
    nb[n1] = nb[n2];
    nb[n2] = tmp;

    for(auto v: nodes_) {
        cost_id n1c = 0;
        cost_id n2c = 0;
        for (auto cb : nb[v]) {
            if (cb.first == n1) {
                n1c = cb.second;
            }
            else if (cb.first == n2) {
                n2c = cb.second;
            }
        }
        if (n1c > 0) {
            nb[v].erase(n1);
            nb[v].emplace(n2, n1c);
        }
        if (n2c > 0) {
            nb[v].erase(n2);
            nb[v].emplace(n1, n2c);
        }
    }
}
