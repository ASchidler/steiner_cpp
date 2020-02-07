//
// Created by andre on 24.01.20.
//

#ifndef STEINER_REDUCTION_H
#define STEINER_REDUCTION_H
#include "../SteinerInstance.h"
#include "../Steiner.h"
#include "../SteinerTree.h"
namespace steiner {
    class Reduction {
    public:
        explicit Reduction(SteinerInstance* instance) : instance(instance) {

        }
        virtual ~Reduction() = default;
        virtual node_id reduce(node_id currCount, node_id prevCount) = 0;
        virtual void reset() {
            enabled = true;
            addedPreselected_ = false;
        }
        virtual bool postProcess(SteinerResult* solution) {
            bool change = false;
            if (! addedPreselected_) {
                addedPreselected_ = true;
                for (auto e: preselected) {
                    solution->g->addEdge(e.u, e.v, e.cost);
                    change = true;
                }
            }

            for (auto n : merged) {
                if (solution->g->getMaxNode() > n.newEdge.u) {
                    auto e = solution->g->nb[n.newEdge.u].find(n.newEdge.v);
                    if (e != solution->g->nb[n.newEdge.u].end() && e->second == n.newEdge.cost) {
                        solution->g->addEdge(n.oldEdge1.u, n.oldEdge1.v, n.oldEdge1.cost);
                        solution->g->addEdge(n.oldEdge2.u, n.oldEdge2.v, n.oldEdge2.cost);
                        solution->g->removeEdge(n.newEdge.u, n.newEdge.v);
                        change = true;
                    }
                }
            }

            for (auto n: contracted) {
                if (solution->g->getMaxNode() > n.target) {
                    auto e = solution->g->nb[n.target].find(n.n);
                    if (e != solution->g->nb[n.target].end() && e->second == n.c) {
                        solution->g->addEdge(n.removed, n.n, n.c);
                        solution->g->removeEdge(n.target, n.n);
                        change = true;
                    }
                }
            }

            return change;
        }
        virtual string getName() = 0;
        bool enabled = true;
    protected:
        vector<ContractedEdge> contracted;
        SteinerInstance* instance;
        virtual inline void preselect(node_id u, node_id v, cost_id c) {
            auto un = instance->getGraph()->getReverseMapping(u);
            auto vn = instance->getGraph()->getReverseMapping(v);
            preselected.emplace_back(un, vn, c);
        }

        virtual inline void merge(node_id removed, node_id u, node_id v, cost_id cu, cost_id cv) {
            auto rn = instance->getGraph()->getReverseMapping(removed);
            auto un = instance->getGraph()->getReverseMapping(u);
            auto vn = instance->getGraph()->getReverseMapping(v);
            merged.emplace_back(rn, un, vn, cu, cv);
        }
    private:
        bool addedPreselected_ = false;

        vector<Edge> preselected;
        vector<MergedEdges> merged;
    };
}


// TODO: Bound NTDK?

#endif //STEINER_REDUCTION_H
