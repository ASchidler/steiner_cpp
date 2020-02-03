//
// Created by aschidler on 1/29/20.
//

#include "Degree3Reduction.h"

node_id steiner::Degree3Reduction::reduce(node_id currCount, node_id prevCount) {
    node_id track = 0;

    auto n = instance->getGraph()->getNodes().begin();
    while (n != instance->getGraph()->getNodes().end()) {
        auto nb = instance->getGraph()->nb[*n];

        if (*n >= instance->getNumTerminals() && nb.size() == 3) { // Degree == 3
            cost_id edgeSum = 0;
            node_id nbs[3];
            node_id idx = 0;
            unordered_set<node_id> ignoreNodes;
            ignoreNodes.insert(*n);

            for (auto& v : nb) {
                edgeSum += v.second;
                nbs[idx++] = v.first;
            }

            node_id distInput0[] = {nbs[0]};

            auto cDist = Degree3Distances(instance, distInput0, 1, &ignoreNodes);
            cost_id dist[] = {
                    cDist.get(nbs[1], edgeSum),
                    cDist.get(nbs[2], edgeSum),
                    SubDijkstra(nbs[1], nbs[2], &ignoreNodes, edgeSum)
            };

            for (int i=0; i < 3; i++) {
                auto p = nbs[i];;
                auto x = nbs[(i+1) % 3];
                auto y = nbs[(i+2) % 3];
                auto up = nb[p];
                auto ux = nb[x];
                auto uy = nb[y];
                auto xp = dist[(3 - i) % 3]; // 0 2 1
                auto yp = dist[(4 - i) % 3]; // 1 0 2
                auto xy = dist[2 - i]; // 2 1 0
                // Alternative is longer than using the two edges, no chance
                if (xp > ux + up || yp > uy + up)
                    continue;

                // If the mst (first part) is cheaper than the edges. this is actually the same expression in all 3 iterations...
                if (xp + yp + xy - max(xy, max(xp, yp)) <= ux + uy + up) {
                    track++;
                    instance->removeEdge(*n, p);
                    break;
                // See if we can find another way by using a path instead of an age
                } else {
                    bool del = false;
                    node_id distInput[] {x, y};
                    cDist = Degree3Distances(instance, distInput, 2, &ignoreNodes);
                    while (true) { // while true
                        // Replace edge by path
                        if (cDist.get(p, up + 1) <= up) {
                            del = true;
                            break;
                        }
                        if (p < instance->getNumTerminals())
                            break;
                        ignoreNodes.insert(p);

                        // Restart because ignore changed
                        cDist = Degree3Distances(instance, distInput, 2, &ignoreNodes);
                        int numps = 0;
                        node_id np = 0;
                        cost_id pnp = 0;

                        for(auto& q: instance->getGraph()->nb[p]) {
                            if (numps < 2 && ignoreNodes.count(q.first) == 0
                                && cDist.get(q.first, max(up, q.second) + 1) > max(up, q.second)) {
                                np = q.first;
                                pnp = q.second;
                                numps++;
                            }
                        }

                        if (numps > 1)
                            break;
                        else if (numps == 1) {
                            p = np;
                            up = pnp;
                        } else {
                            del = true;
                            break;
                        }
                    }  // while true

                    if (del) {
                        instance->removeEdge(*n, nbs[i]);
                        track++;
                        break;
                    }
                }
            }
        } // Degree == 3
        ++n;
    }
    enabled = track > 0;
    return track;
}

cost_id steiner::Degree3Reduction::SubDijkstra(node_id u, node_id v, unordered_set<node_id>* ignoreNodes, cost_id limit) {
    priority_queue<NodeWithCost> q;
    unordered_map<node_id, cost_id> dist;

    q.emplace(u, 0);
    dist[u] = 0;

    while (! q.empty()) {
        auto elem = q.top();
        q.pop();

        if (elem.cost > limit)
            return limit;

        if (elem.node == v)
            return elem.cost;

        if (elem.cost > dist[elem.node])
            continue;

        for (auto& nb: instance->getGraph()->nb[elem.node]) {
            auto nCost = elem.cost + nb.second;
            if (nCost < limit and ignoreNodes->count(nb.first) == 0) {
                auto entry = dist.find(nb.first);
                if (entry == dist.end()) {
                    dist.emplace(nb.first, nCost);
                    q.emplace(nb.first, nCost);
                } else if (entry->second > nCost) {
                    entry->second = nCost;
                    q.emplace(nb.first, nCost);
                }
            }
        }
    }

    return limit;
}

cost_id steiner::Degree3Distances::get(node_id target, cost_id limit) {
    auto existing = dist_.find(target);
    // Weight may be suboptimal
    if (existing != dist_.end() && existing->second <= cMax_)
        return existing->second;

    if (limit <= cMax_)
        return limit;

    while (!q_.empty()) {
        auto elem = q_.top();
        q_.pop();
        cMax_ = elem.cost;

        if (elem.node == target)
            return elem.node;

        if (elem.cost > dist_[elem.node])
            continue;

        for (auto& nb: instance_->getGraph()->nb[elem.node]) {
            auto nCost = elem.cost + nb.second;
            if (nCost < limit and ignore_->count(nb.first) == 0) {
                auto entry = dist_.find(nb.first);
                if (entry == dist_.end()) {
                    dist_[nb.first] = nCost;
                    q_.emplace(nb.first, nCost);
                }
                else if (entry->second > nCost) {
                    entry->second = nCost;
                    q_.emplace(nb.first, nCost);
                }
            }
        }
    }

    return limit;
}
