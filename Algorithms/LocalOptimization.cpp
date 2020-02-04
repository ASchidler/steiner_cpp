//
// Created by andre on 31.01.20.
//

#include "LocalOptimization.h"


void steiner::LocalOptimization::vertexInsertion(Graph* dg, HeuristicResult& tr) {
    for(auto n: dg->getNodes()) {
        if (tr.g->getNodes().count(n) == 0) {
            // Find neighbors that are in the solution
            vector<NodeWithCost> shared;
            for(auto& b: dg->nb[n]) {
                if (tr.g->getNodes().count(b.first) > 0) {
                    shared.emplace_back(b.first, b.second);
                }
            }
            // There exist at least two neighbors (1 would not have a chance to succeed)
            if (shared.size() > 1) {
                auto cp = new Graph(*tr.g, false);
                // Add first edge and try to improve
                cp->addEdge(n, shared.back().node, shared.back().cost);
                shared.pop_back();

                // Check all other shared neighbors for improvement
                for(auto b: shared) {
                    // Find shortest path from
                    auto path = cp->findPath(n, b.node);
                    auto maxEdge = Edge(path[0], path[1], dg->nb[path[0]][path[1]]);

                    for (size_t cNode = 2; cNode < path.size(); cNode++) {
                        auto c = cp->nb[path[cNode - 1]][path[cNode]];
                        if (c > maxEdge.cost) {
                            maxEdge.cost = c;
                            maxEdge.u = path[cNode - 1];
                            maxEdge.v = path[cNode];
                        }
                    }
                    // If the path cost is higher, using the found edge is cheaper
                    if (maxEdge.cost > b.cost) {
                        // Iterator is on dg, not tr, so it stays valid
                        cp->addEdge(n, b.node, b.cost);
                        cp->removeEdge(maxEdge.u, maxEdge.v);
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
    if (tr.g->getNumNodes() < 5)
        return;

    vector<Edge> bridges[g.getMaxNode()];
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
            bridges[t1.t].emplace_back(edge.u, edge.v, edge.cost);
            bridges[t2.t].emplace_back(edge.u, edge.v, edge.cost);
        }
        ++edgeIt;
    }

    // Find key paths. This is a long process.
    vector<node_id> q;
    node_id parents[g.getMaxNode()];
    parents[tr.root] = g.getMaxNode();
    vector<node_id> kvq;
    unordered_map<node_id, unordered_set<node_id>> subsets;
    unordered_set<node_id> pinned;
    unordered_set<node_id> closed;

    // Extract the key vertex tree. Establish child-parent between all vertices.
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

    // Iterate over the whole structure in DFS order
    while(! kvq.empty()) {
        // Get current node
        node_id n = kvq.back();
        kvq.pop_back();
        // find path
        unordered_set<node_id> intermediaries;
        vector<node_id> path;
        path.push_back(n);
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
        node_id p1end = 0;
        node_id p2end = 0;

        // Extend subset of parent key node
        auto np = path.back();
        subsets[n].insert(n);
        subsets[np].reserve(subsets[n].size() + path.size() + subsets[n].size());
        subsets[np].insert(subsets[n].begin(), subsets[n].end());

        // Do not move pinned elements
        if (foundPinned) {
            subsets[np].insert(path.begin(), path.end());
            continue;
        }

        // Repair diagram -> move nodes previously assigned to intermediaries to other nodes (simulate removal)
        vor.repair(intermediaries);

        // Treat n as in intermediary from here on! (Do not move before repair!)
        intermediaries.insert(n);
        Bridge minBridge(MAXCOST, 0, 0, 0);

        // Repeat vor intermediaries. Slightly different from above
        for(auto im: intermediaries) {
            for(size_t i=0; i < bridges[n].size(); i++) {
                auto& cBridge = bridges[n][i];
                auto r1 = vor.getClosest(cBridge.u);
                auto r2 = vor.getClosest(cBridge.v);
                // We know that one of the two nodes is in the current region, make sure not both (would disconnect graph)
                bool uIn = subsets[n].count(r1.t) > 0;
                bool vIn = subsets[n].count(r2.t) > 0;

                // Since the region is getting larger, remove invalid bridges for key vertices
                if (im == n && uIn && vIn) {
                    if (i != bridges[n].size() - 1)
                        std::swap(bridges[n][i], bridges[n].back());
                    bridges[n].pop_back();
                    i--;
                }
                else if (uIn ^ vIn) {
                    cost_id totalCost = cBridge.cost + r1.costEntry.cost + r2.costEntry.cost;
                    if (totalCost < minBridge.total && closed.count(r1.t) == 0 && closed.count(r2.t) == 0) {
                        delete p1;
                        delete p2;
                        p1 = vor.getPath(cBridge.u);
                        p2 = vor.getPath(cBridge.v);
                        p1end = r1.t;
                        p2end = r2.t;

                        minBridge = Bridge(totalCost, cBridge.u, cBridge.v, cBridge.cost);
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
            int total = 0;
            // new path
            for(auto& e: *p1) {
                tr.g->addEdge(e.u, e.v, e.cost);
                total += e.cost;
            }
            for(auto& e: *p2) {
                tr.g->addEdge(e.u, e.v, e.cost);
                total += e.cost;
            }
            tr.g->addEdge(minBridge.e.u, minBridge.e.v, minBridge.e.cost);
            total += minBridge.e.cost;
            assert(minBridge.total == total);

            // Forbid subset and intermediaries from  change
            for(auto f: subsets[n])
                closed.insert(f);
            for(auto im: intermediaries)
                closed.insert(im);

            // Pin endpoints
            pinned.insert(p1end);
            pinned.insert(p2end);
            assert(tr.g->checkConnectedness(0, false));
        } else {
            subsets[np].insert(path.begin(), path.end());
        }

        // Move to parent key vertex
        //TODO: Reserve for intermediaries as well? Maybe count them above?
        bridges[np].reserve(bridges[np].size() + bridges[n].size());
        for(auto im: intermediaries) {
            std::move(begin(bridges[im]), end(bridges[im]), back_inserter(bridges[np]));
        }
        // TODO: I'm not sure if this presorting is actually efficient
        sort(bridges[np].begin(), bridges[np].end());
        vor.reset();

        delete p1;
        delete p2;
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

    // Reassign all nodes associated with the "removed" voronoi centers
    for(auto n: intermediaries) {
        for(auto& n2: regions_[n]) {
            repairNodes.insert(n2.first);
        }
    }

    priority_queue<VoronoiQueueEntry> q;

    // Initialize dijkstra. Boundary nodes are added with distance to center
    for(auto n: repairNodes) {
        // Find a neighbor that is not in the same region and try if it's associated center fits
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
            auto newElement = regionsTmp_[elem.start].emplace(std::piecewise_construct, forward_as_tuple(elem.n), forward_as_tuple(elem.predecessor, elem.c));
            closestTmp_[elem.n] = new ClosestEntry(elem.start, newElement.first->second);

            for(auto& nb: g_.nb[elem.n]) {
                if (!visited[nb.first] && repairNodes.count(nb.first) > 0) {
                    q.emplace(elem.c + nb.second, nb.first, elem.start, elem.n);
                }
            }
        }
    }
}

vector<steiner::Edge>* steiner::VoronoiPartition::getPath(node_id n) {
    auto& t = getClosest(n).t;
    auto* r = (!regionsTmp_[t].empty()) ? &regionsTmp_[t] : &regions_[t];
    bool switched = regionsTmp_[t].empty();

    auto* path = new vector<Edge>;

    while(t != n) {
        // Switch from new assignment to old, if no new assignments are available...
        if (! switched && r->count(n) == 0) {
            r = &regions_[t];
            switched = true;
        }

        assert(r->count(n) > 0);
        auto& prev = (*r)[n];
        path->emplace_back(n, prev.node, g_.nb[n][prev.node]);
        n = prev.node;
    }

    return path;
}
