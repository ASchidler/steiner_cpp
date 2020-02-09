//
// Created by aschidler on 1/30/20.
//

#include "DualAscentReduction.h"
#include "../Reductions/Reducer.h"

node_id steiner::DualAscentReduction::reduce(node_id currCount, node_id prevCount) {
    if (instance->getNumTerminals() < 3)
        return 0;

    node_id track = 0;

    // Always request a new approximation for this reduction
    instance->requestApproximationState(SteinerInstance::invalid);

    // TODO: Define control parameters according to instance size

    node_id numRoots = min((node_id)50, instance->getNumTerminals());
    node_id roots[numRoots];
    chooseRoots(roots, numRoots);
    SteinerResult* results[numRoots];
    Voronoi* vors[numRoots];

    for(node_id t=0; t < numRoots; t++) {
        results[t] = DualAscent::calculate(instance->getGraph(), roots[t], nullptr, instance->getNumTerminals(),
                                           instance->getGraph()->getMaxNode());
        results[t]->g->findDistances(results[t]->root);
        vors[t] = voronoi(instance->getGraph(), instance->getNumTerminals());
    }

    cout << "Before Prune: " << instance->getApproximation().getLowest() << endl;
    for(node_id t=0; t < numRoots && t < 5; t++){
        prune(results[t], vors[t]);
    }

    node_id tracks[numRoots];
    for(node_id t=0; t < numRoots; t++) {
        tracks[t] = reduceGraph(results[t], vors[t]);
        track += tracks[t];
    }

    selectRoots(results, numRoots, tracks);
    for(node_id t=0; t < numRoots; t++) {
        delete results[t];
        delete vors[t];
    }

    if (track > 0) {
        instance->setSteinerDistanceState(SteinerInstance::invalid);
        instance->setDistanceState(SteinerInstance::invalid);
        instance->setSteinerDistanceState(SteinerInstance::invalid);
    }
    enabled = track > 0;
    return track;
}

// TODO: Maybe get rid of dynamic bitset? Change it to integers and only if > 128 use some other, non-expensive form?
node_id steiner::DualAscentReduction::reduceGraph(steiner::SteinerResult* r, Voronoi* vor) {
    r->g->findDistances(r->root);
    cost_id* dist = r->g->getDistances()[r->root];
    node_id track = 0;

    cost_id limit = instance->getUpperBound();
    limit -= r->cost;

    // Use r->g since the instance's nodes may change during iteration
    for(const auto n : r->g->getNodes()) {
        // Ensure that the node is still there, otherwise NTDK may add ghost edges...
        if (instance->getGraph()->nb[n].empty())
            continue;
        if (dist[n] + vor->closest[n].cost > limit) {
            instance->removeNode(n);
            track++;
        }
        // NTDK test, does the node is a non-terminal and have maximum degree 2 in any steiner tree?
        else if (n >= instance->getNumTerminals() && dist[n] + vor->second[n].cost > limit and
            instance->getGraph()->nb[n].size() <= 6 && instance->getGraph()->getNodes().count(dist[n]) > 0) {
            // Insert replacement edges
            for (const auto &b: instance->getGraph()->nb[n]) {
                for (const auto &b2: instance->getGraph()->nb[n]) {
                    if (b.first < b2.first) {
                        if (instance->addEdge(b.first, b2.first, b.second + b2.second)) {
                            merge(n, b.first, b2.first, b.second, b2.second);
                        }
                    }
                }
            }

            instance->removeNode(n);
            track++;
        }
        // Can we remove some if the edges?
        else {
            auto nb = r->g->nb[n];
            for(const auto& n2: nb) {
                // Try to avoid doing the check twice
                if (n < n2.first) {
                    // Must be over the lower bound from both sides
                    auto edgeCost1 = dist[n2.first] + r->g->nb[n2.first][n] + vor->closest[n].cost;
                    auto edgeCost2 = dist[n] + n2.second + vor->closest[n2.first].cost;

                    if (edgeCost1 > limit && edgeCost2 > limit) {
                        instance->removeEdge(n, n2.first);
                        track++;
                    }
                }
            }
        }
    }

    return track;
}

void steiner::DualAscentReduction::chooseRoots(node_id *roots, node_id numRoots) {
    assert(numRoots <= instance->getNumTerminals());
    node_id rootsSelected = 0;
    node_id selectBest = min(2, numRoots/2);
    for(auto i=0; i < selectBest && rootsSelected < numRoots; i++) {
        if (bestRoots[i] < instance->getNumTerminals()) {
            roots[rootsSelected] = bestRoots[i];
            rootsSelected++;
        }
    }

    // Now select terminals
    for(; rootsSelected < numRoots; rootsSelected++) {
        node_id t = random() % instance->getNumTerminals();
        // Select next t that has not been selected yet
        for(auto j=0; j < rootsSelected; j++) {
            if (roots[j] == t) {
                t++;
                if (t >= instance->getNumTerminals())
                    t = 0;
                j=0;
            }
        }

        roots[rootsSelected] = t;
    }
}

void steiner::DualAscentReduction::selectRoots(steiner::SteinerResult** results, node_id numSolutions, const node_id *track) {
    // Okay so the idea is that we choose the two best roots in terms of bounds and the two best roots in terms of elimination
    cost_id bestBound = 0;
    node_id bestTrack = 0;

    for(node_id i=0; i < numSolutions; i++) {
        if(results[i]->cost > bestBound) {
            bestRoots[0] = results[i]->root;
            bestBound = results[i]->cost;
        } else if (track[i] > bestTrack) {
            bestRoots[1] = results[i]->root;
            bestTrack = track[i];
        }
    }
}

void steiner::DualAscentReduction::prune(steiner::SteinerResult *r, steiner::Voronoi *vor) {
    Graph g = Graph();
    auto edgeIt = r->g->findEdges();

    while(edgeIt.hasElement()) {
        auto e = *edgeIt;
        if (e.cost == 0)
            g.addEdge(e.u, e.v, instance->getGraph()->nb[e.u][e.v]);
        ++edgeIt;
    }

    SteinerInstance s(&g, instance->getNumTerminals());
    auto red = Reducer::getMinimalReducer(&s);
    red.reduce();

    unsigned int cnt = 0;
    do {
        // Get upper bound
        for(auto t=0; t < s.getNumTerminals() && t < 5; t++) {
            auto rt = random() % s.getNumTerminals();
            auto result = ShortestPath::calculate(rt, g, s.getNumTerminals());
            result->g->remap(g);
            red.reset();
            red.unreduce(result);
            instance->getApproximation().addToPool(result);
        }
        cost_id cBound = instance->getApproximation().getLowest() * (1.0 - cnt * 0.1);
        cost_id limit = cBound - r->cost;

        auto dist = r->g->getDistances()[r->root];
        for(const auto n : r->g->getNodes()) {
            // Ensure that the node is still there, otherwise NTDK may add ghost edges...
            if (s.getGraph()->nb[n].empty())
                continue;

                // NTDK test, does the node is a non-terminal and have maximum degree 2 in any steiner tree?
            else if (n >= instance->getNumTerminals() && dist[n] + vor->second[n].cost > limit and
                     instance->getGraph()->nb[n].size() <= 6 && instance->getGraph()->getNodes().count(dist[n]) > 0) {
                // Insert replacement edges
                for (const auto &b: instance->getGraph()->nb[n]) {
                    for (const auto &b2: instance->getGraph()->nb[n]) {
                        if (b.first < b2.first) {
                            if (instance->addEdge(b.first, b2.first, b.second + b2.second)) {
                                merge(n, b.first, b2.first, b.second, b2.second);
                            }
                        }
                    }
                }

                instance->removeNode(n);
                track++;
            }
                // Can we remove some if the edges?
            else {
                auto nb = r->g->nb[n];
                for(const auto& n2: nb) {
                    // Try to avoid doing the check twice
                    if (n < n2.first) {
                        // Must be over the lower bound from both sides
                        auto edgeCost1 = dist[n2.first] + r->g->nb[n2.first][n] + vor->closest[n].cost;
                        auto edgeCost2 = dist[n] + n2.second + vor->closest[n2.first].cost;

                        if (edgeCost1 > limit && edgeCost2 > limit) {
                            instance->removeEdge(n, n2.first);
                            track++;
                        }
                    }
                }
            }
        }


        // Use r->g since the instance's nodes may change during iteration
        for(const auto n : r->g->getNodes()) {
            // Ensure that the node is still there, otherwise NTDK may add ghost edges...
            if (instance->getGraph()->nb[n].empty())
                continue;
            if (dist[n] + vor->closest[n].cost > limit) {
                instance->removeNode(n);
                track++;
            }

        cnt++;
    } while(cnt < 4 && g.checkConnectedness(s.getNumTerminals(), true));

}
