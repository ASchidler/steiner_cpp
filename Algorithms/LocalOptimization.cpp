//
// Created by andre on 31.01.20.
//

#include "LocalOptimization.h"


void steiner::LocalOptimization::vertexInsertion(Graph* dg, HeuristicResult& tr) {
    for(auto n: dg->getNodes()) {
        if (tr.g->getNodes().count(n) == 0) {
            // Find neighbors that are in the solution
            bool exists = false;
            for(auto& b: dg->nb[n]) {
                if (tr.g->getNodes().count(b.first) > 0) {
                    exists = true;
                    break;
                }
            }
            // There exists at least one such neighbor
            if (exists) {
                auto cp = new Graph(*tr.g, false);
                // Check paths to neighbors

                for(auto& b : dg->nb[n]) {
                    if (tr.g->getNodes().count(n) > 0) {
                        auto path = dg->findPath(n, b.first);
                        auto maxEdge = Edge(path[0], path[1], dg->nb[path[0]][path[1]]);
                        for (size_t cNode = 2; cNode < path.size(); cNode++) {
                            auto c = tr.g->nb[path[cNode - 1]][path[cNode]];
                            if (c > maxEdge.cost) {
                                maxEdge.cost = c;
                                maxEdge.u = path[cNode - 1];
                                maxEdge.v = path[cNode];
                            }
                        }
                        // If the path cost is higher, using the found edge is cheaper
                        if (maxEdge.cost > b.second) {
                            // Iterator is on dg, not tr, so it stays valid
                            cp->addEdge(n, b.first, b.second);
                            cp->removeEdge(maxEdge.u, maxEdge.v);
                        }
                    }
                }

                // Check if we made progress:
                auto trCost = cp->getCost();
                if (tr.bound > trCost) {
                    delete tr.g;
                    tr.g = cp;
                    tr.bound = trCost;
                } else {
                    delete cp;
                }
            }
        }
    }
}

void steiner::LocalOptimization::pathExchange(Graph& g, HeuristicResult& tr, node_id numTerminals) {
    if (tr.g->getNumNodes() < 5 || numTerminals > 1500)
        return;

    vector<Bridge> bridges[g.getMaxNode()];
    auto vor = VoronoiPartition(g, tr);
    bool isKey[tr.g->getMaxNode()];

    for(node_id n=0; n < tr.g->getMaxNode(); n++) {
        isKey[n] = (n < numTerminals || tr.g->nb[n].size() > 2);
    }


    // Find bridging edges
    auto edgeIt = g.findEdges();
    while(edgeIt.hasElement()) {
        auto edge = *edgeIt;
        auto t1 = vor.getClosest(edge.u);
        auto t2 = vor.getClosest(edge.v);
        // Is indeed a bridge
        if (t1.t != t2.t) {
            auto total = edge.cost + t1.costEntry.cost + t2.costEntry.cost;
            bridges[t1.t].emplace_back(total, edge.u, edge.v, edge.cost);
            bridges[t2.t].emplace_back(total, edge.u, edge.v, edge.cost);
        }
        ++edgeIt;
    }

    for(node_id i=0; i < g.getMaxNode(); i++) {
        sort(bridges[i].begin(), bridges[i].end());
    }

    // Find key paths
    vector<node_id> q;
    node_id parents[g.getMaxNode()];
    parents[tr.root] = g.getMaxNode();
    vector<node_id> kvq;
    unordered_map<node_id, vector<node_id>> subsets;
    unordered_set<node_id> pinned;
    unordered_set<node_id> forbidden;

    // Establish tree relationships, find leafs and key vertex structure
    q.push_back(tr.root);
    while(!q.empty()) {
        auto v = q.back();
        q.pop_back();
        for(auto& nb: tr.g->nb[v]) {
            if (nb.first != parents[v]) {
                q.push_back(nb.first);
                parents[nb.first] = v;
                if (isKey[nb.first]) {
                    kvq.push_back(nb.first);
                }
            }
        }
    }

    while(! kvq.empty()) {
        // Get current node
        node_id n = kvq.back();
        kvq.pop_back();
        // find path
        unordered_set<node_id> intermediaries;
        vector<node_id> path;
        node_id p = parents[n];
        cost_id pathCost = 0;
        bool foundPinned = false;
        for(; !isKey[p]; p = parents[p]) {
            pathCost += g.nb[path.back()][p];
            path.push_back(p);
            intermediaries.insert(p);
            if (pinned.count(p) > 0)
                foundPinned = true;
        }
        pathCost += g.nb[path.back()][p];
        path.push_back(p);
        vector<Edge>* p1 = nullptr;
        vector<Edge>* p2 = nullptr;

        // Update subsets of parent and current node
        subsets[n].reserve(subsets[n].size() + path.size());
        subsets[n].insert(subsets[n].end(), std::make_move_iterator(path.begin()), std::make_move_iterator(path.end()));
        subsets[path.back()].reserve(subsets[path.back()].size() + subsets[n].size());

        // Do not move pinned elements
        if (foundPinned) {
            std::move(begin(path), end(path), back_inserter(subsets[path.back()]));
            continue;
        }

        // Repair diagram -> move nodes previously assigned to intermediaries to other nodes (simulate removal)
        vor.repair(intermediaries);

        // Find cheapest real bridge. I.e. after the Regions of the children have been merged
        Bridge minBridge(MAXCOST, 0, 0, 0);
        while (!bridges[n].empty()) {
            auto& cBridge = bridges[n].back();

            bool uIn = intermediaries.count(cBridge.e.u) > 0;
            bool vIn = intermediaries.count(cBridge.e.v) > 0;

            for(auto n2 : subsets[n]) {
                if (uIn && vIn)
                    break;
                uIn = uIn || vor.isInRegion(cBridge.e.u, n2) || vor.isInTmpRegion(cBridge.e.u, n2);
                vIn = vIn || vor.isInRegion(cBridge.e.v, n2) || vor.isInTmpRegion(cBridge.e.v, n2);
            }

            if (uIn && vIn)
                bridges[n].pop_back();
            else {
                p1 = vor.getPath(cBridge.e.u);
                p2 = vor.getPath(cBridge.e.v);

                if (forbidden.count(p1->back().v) == 0 && forbidden.count(p2->back().v)) {
                    minBridge = cBridge;
                }
                // Found bridge, cancel search
                break;
            }
        }

        // Repeat vor intermediaries. Slightly different from above
        for(auto im: intermediaries) {
            for(auto& cBridge:bridges[im]) {
                bool uIn = false;
                bool vIn = false;
                for(auto n2 : subsets[n]) {
                    if (uIn && vIn)
                        break;
                    uIn = uIn || vor.isInRegion(cBridge.e.u, n2) || vor.isInTmpRegion(cBridge.e.u, n2);
                    vIn = vIn || vor.isInRegion(cBridge.e.v, n2) || vor.isInTmpRegion(cBridge.e.v, n2);
                }

                // Is bridge?
                if (uIn ^ vIn) {
                    cost_id totalCost = cBridge.e.cost + vor.getRegionEntry(cBridge.e.u).cost + vor.getRegionEntry(cBridge.e.v).cost;
                    if (totalCost < minBridge.total) {
                        auto* tp1 = vor.getPath(cBridge.e.u);
                        auto* tp2 = vor.getPath(cBridge.e.v);

                        if (forbidden.count(tp1->back().v) == 0 && forbidden.count(tp2->back().v)) {
                            minBridge = cBridge;
                            minBridge.total = totalCost;
                            delete p1;
                            delete p2;
                            p1 = tp1;
                            p2 = tp2;
                        } else {
                            delete tp1;
                            delete tp2;
                        }
                    }
                }
            }
        }

        // Found cheaper key path
        // TODO: Favor new?
        if (minBridge.total < pathCost) {
            // Remove old path
            for(size_t idx=0; idx < path.size() - 1; idx++) {
                tr.g->removeEdge(path[idx], path[idx+1]);
            }
            // new path
            for(auto& e: *p1)
                tr.g->addEdge(e.u, e.v, e.cost);
            for(auto& e: *p2)
                tr.g->addEdge(e.u, e.v, e.cost);
            tr.g->addEdge(minBridge.e.u, minBridge.e.v, minBridge.e.cost);

            // Forbid subset
            for(auto f: subsets[n])
                forbidden.insert(f);

            // Pin endpoints
            pinned.insert(p1->back().v);
            pinned.insert(p2->back().v);
        }

        // Move to parent key vertex
        node_id np = path.back();
        std::move(begin(subsets[n]), end(subsets[n]), back_inserter(subsets[np]));
        bridges[np].reserve(bridges[np].size() + bridges[n].size());
        std::move(begin(bridges[n]), end(bridges[n]), back_inserter(bridges[np]));
        for(auto im: intermediaries) {
            std::move(begin(bridges[im]), end(bridges[im]), back_inserter(bridges[np]));
        }
        vor.reset();
    }
    tr.bound = tr.g->getCost();
}

void steiner::LocalOptimization::keyVertexDeletion(Graph& g, HeuristicResult& tr) {

}

steiner::VoronoiPartition::VoronoiPartition(steiner::Graph &g, steiner::HeuristicResult &tr) : g_(g) {
    regions_ = new unordered_map<node_id, NodeWithCost>[tr.g->getMaxNode()];
    closest_ = new ClosestEntry*[g.getMaxNode()];
    regionsTmp_ = new unordered_map<node_id, NodeWithCost>[tr.g->getMaxNode()];
    closestTmp_ = new ClosestEntry*[g.getMaxNode()];

    auto q = priority_queue<VoronoiQueueEntry>();
    bool visited[g.getMaxNode()];

    for(auto i=0; i < g.getMaxNode(); i++){
        visited[i] = false;
        closest_[i] = nullptr;
        closestTmp_[i] = nullptr;
    }

    for(auto n: tr.g->getNodes()) {
        q.emplace(0, n, n, n);
    }

    while(!q.empty()) {
        auto elem = q.top();
        q.pop();

        if (! visited[elem.n]) {
            visited[elem.n] = true;
            auto newElement = regions_[elem.start].emplace(std::piecewise_construct, forward_as_tuple(elem.n), forward_as_tuple(elem.predecessor, elem.c));
            closest_[elem.n] = new ClosestEntry(elem.start, newElement.first->second);

            for(auto& nb: g.nb[elem.n]) {
                if (!visited[nb.first]) {
                    q.emplace(elem.c + nb.second, nb.first, elem.start, elem.n);
                }
            }
        }
    }

}

void steiner::VoronoiPartition::reset() {
    for(auto i=0; i < g_.getMaxNode(); i++){
        regionsTmp_[i].clear();
        delete closestTmp_[i];
        closestTmp_[i] = nullptr;
    }
}

void steiner::VoronoiPartition::repair(unordered_set<node_id>& intermediaries) {
    reset();

    if (intermediaries.empty())
        return;

    unordered_set<node_id> repairNodes;
    bool visited[g_.getMaxNode()];
    for(node_id i=0; i < g_.getMaxNode(); i++) {
        visited[i] = false;
    }

    for(auto n: intermediaries) {
        for(auto& n2: regions_[n]) {
            repairNodes.insert(n2.first);
        }
    }

    priority_queue<VoronoiQueueEntry> q;

    // Initialize dijkstra. Boundary nodes are added with distance to center
    for(auto n: repairNodes) {
        for(auto& n2: g_.nb[n]) {
            auto& cl = closest_[n2.first];
            if (intermediaries.count(cl->t) == 0) {
                q.emplace(n2.second + cl->costEntry.cost, n, cl->t, n2.first);
            }
        }
    }

    // Dijkstra, limited to dangling (repair) nodes
    while(!q.empty()) {
        auto elem = q.top();
        q.pop();
        if (! visited[elem.n]) {
            visited[elem.n] = true;
            auto newElement = regions_[elem.start].emplace(std::piecewise_construct, forward_as_tuple(elem.n), forward_as_tuple(elem.predecessor, elem.c));
            closest_[elem.n] = new ClosestEntry(elem.start, newElement.first->second);

            for(auto& nb: g_.nb[elem.n]) {
                if (!visited[nb.first] && repairNodes.count(nb.first) > 0) {
                    q.emplace(elem.c + nb.second, nb.first, elem.start, elem.n);
                }
            }
        }
    }
}

vector<steiner::Edge>* steiner::VoronoiPartition::getPath(node_id n) {
    auto& t = getClosest(n);
    auto& r = !regionsTmp_[t.t].empty() ? regionsTmp_[t.t] : regions_[t.t];
    bool switched = regionsTmp_[t.t].empty();

    auto* path = new vector<Edge>;

    while(t.t != n) {
        // Switch from new assignment to old, if no new assignments are available...
        if (! switched && r.count(n) == 0) {
            r = regions_[t.t];
            switched = true;
        }

        auto& prev = r[n];
        path->emplace_back(n, prev.node, g_.nb[n][prev.cost]);
        n = prev.node;
    }

    return path;
}
