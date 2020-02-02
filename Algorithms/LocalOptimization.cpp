//
// Created by andre on 31.01.20.
//

#include "LocalOptimization.h"


void steiner::LocalOptimization::vertexInsertion(Graph* dg, Graph* tr) {
    for(auto n: *dg->getNodes()) {
        if (tr->getNodes()->count(n) == 0) {
            // Find neighbors that are in the solution
            bool exists = false;
            for(auto& b: dg->nb[n]) {
                if (tr->getNodes()->count(b.first) > 0) {
                    exists = true;
                    break;
                }
            }
            // There exists at least one such neighbor
            if (exists) {
                auto cp = tr->copy(false);
                for(auto& b: dg->nb[n]) {
                    if (tr->getNodes()->count(n) > 0) {
                        auto path = dg->findPath(n, b.first);
                    }
                }
            }
        }
    }
}

void steiner::LocalOptimization::pathExchange() {

}

void steiner::LocalOptimization::keyVertexDeletion() {

}
