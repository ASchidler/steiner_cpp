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

            for (auto &v : nb) {
                edgeSum += v.second;
                nbs[idx++] = v.first;
            }

            vector<node_id> distInput0{nbs[0]};
            vector<node_id> distInput1{nbs[1]};

            auto cDist = Degree3Distances(instance, distInput0, ignoreNodes);
            auto cDist2 = Degree3Distances(instance, distInput1, ignoreNodes);

            cost_id dist[] = {
                    cDist.get(nbs[1], edgeSum),
                    cDist.get(nbs[2], edgeSum),
                    cDist2.get(nbs[2], edgeSum)
            };

            for (int i = 0; i < 3; i++) {
                auto p = nbs[i]; // 0, 1 , 2
                auto x = nbs[(i + 1) % 3]; // 1, 2, 0
                auto y = nbs[(i + 2) % 3]; // 2, 0, 1
                auto up = nb[p];
                auto ux = nb[x];
                auto uy = nb[y];
                auto xp = dist[(3 - i) % 3]; // 0 2 1
                auto yp = dist[(4 - i) % 3]; // 1 0 2
                auto xy = dist[2 - i]; // 2 1 0
                // Alternative is longer than traveling over u
                if (xp > ux + up || yp > uy + up)
                    continue;

                // If the mst (first part) is cheaper than the edges. this is actually the same expression in all 3 iterations...
                if (xp + yp + xy - max(xy, max(xp, yp)) <= ux + uy + up) {
                    track++;
                    instance->removeEdge(*n, p);
                    break;
                    // See if we can find another way by using a path instead of an age
                }
                else {
                    break;
                    bool del = false;
                    vector<node_id> distInput{x, y};
                    auto dd = new Degree3Distances(instance, distInput, ignoreNodes);
                    while (true) { // while true
                        // Replace edge by path
                        if (dd->get(p, up + 1) <= up) {
                            del = true;
                            break;
                        }
                        if (p < instance->getNumTerminals())
                            break;
                        ignoreNodes.insert(p);

                        // Restart because ignore changed
                        delete dd;
                        dd = new Degree3Distances(instance, distInput, ignoreNodes);
                        int numps = 0;
                        node_id np = 0;
                        cost_id pnp = 0;

                        for (auto &q: instance->getGraph()->nb[p]) {
                            if (numps < 2 && ignoreNodes.count(q.first) == 0
                                && dd->get(q.first, max(up, q.second) + 1) > max(up, q.second)) {
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
                    delete dd;
              }
            }
        } // Degree == 3
        ++n;
    }
    enabled = track > 0;
    return track;
}

cost_id steiner::Degree3Distances::get(node_id target, cost_id limit) {
    auto existing = dist_.find(target);
    if (existing != dist_.end())
        return existing->second;

    if (limit <= cMax_)
        return limit;

    while (!q_.empty()) {
        auto elem = q_.top();
        q_.pop();
        cMax_ = elem.cost;

        if (elem.node == target)
            return elem.cost;

        if (elem.cost > dist_[elem.node])
            continue;

        for (auto& nb: instance_->getGraph()->nb[elem.node]) {
            auto nCost = elem.cost + nb.second;
            if (ignore_.count(nb.first) == 0) {
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

        if (elem.cost > limit)
            break;
    }

    return limit;
}
