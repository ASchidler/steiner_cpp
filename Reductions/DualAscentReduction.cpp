//
// Created by aschidler on 1/30/20.
//

#include "DualAscentReduction.h"

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
    for(node_id t=0; t < numRoots; t++)
        results[t] = DualAscent::calculate(instance->getGraph(), roots[t], nullptr, instance->getNumTerminals(), instance->getGraph()->getMaxNode());

    node_id tracks[numRoots];
    for(node_id t=0; t < numRoots; t++) {
        tracks[t] = reduceGraph(results[t]);
        track += tracks[t];
    }

    selectRoots(results, numRoots, tracks);
    for(node_id t=0; t < numRoots; t++)
        delete results[t];

    if (track > 0) {
        instance->setSteinerDistanceState(SteinerInstance::invalid);
        instance->setDistanceState(SteinerInstance::invalid);
        instance->setSteinerDistanceState(SteinerInstance::invalid);
    }
    enabled = track > 0;
    return track;
}

bool pairSort(const pair<int,int> &a,
              const pair<int,int> &b)
{
    return (a.first > b.first || (a.first == b.first && a.second > b.second));
}
// TODO: Maybe get rid of dynamic bitset? Change it to integers and only if > 128 use some other, non-expensive form?
node_id steiner::DualAscentReduction::reduceGraph(steiner::SteinerResult* r) {
    r->g->findDistances(r->root);
    cost_id* dist = r->g->getDistances()[r->root];
    node_id track = 0;

    cost_id limit = instance->getUpperBound();
    limit -= r->cost;
    auto vor = voronoi(r->g, instance->getNumTerminals());

    // Use r->g since the instance's nodes may change during iteration
    for(const auto n : r->g->getNodes())
    for(node_id t=0; t < instance->getNumTerminals(); t++) {
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

    delete vor;
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
