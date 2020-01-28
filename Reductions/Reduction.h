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
        virtual bool postProcess(SteinerTree* solution) {
            bool change = false;
            if (! addedPreselected_) {
                addedPreselected_ = true;
                for (auto e: preselected) {
                    solution->addEdge(e.u, e.v, e.cost);
                    change = true;
                }
            }

            for (auto n : merged) {
                if (solution->removeEdge(n.newEdge.u, n.newEdge.v, n.newEdge.cost)) {
                    solution->addEdge(n.oldEdge1.u, n.oldEdge1.v, n.oldEdge1.cost);
                    solution->addEdge(n.oldEdge2.u, n.oldEdge2.v, n.oldEdge2.cost);
                    change = true;
                }
            }

            for (auto n: contracted) {
                if (solution->removeEdge(n.target, n.n, n.c)) {
                    solution->addEdge(n.removed, n.n, n.c);
                    change = true;
                }

            }

            return change;
        }
        virtual string getName() = 0;
    protected:

        vector<Edge> preselected;
        vector<ContractedEdge> contracted;
        vector<MergedEdges> merged;
        SteinerInstance* instance;
        virtual inline void preselect(node_id u, node_id v, cost_id c) {
            auto un = instance->getGraph()->getReverseMapping(u);
            auto vn = instance->getGraph()->getReverseMapping(v);
            preselected.emplace_back(un, vn, c);
        }
        virtual inline void contract(node_id removed, node_id target, node_id n, cost_id c) {
            auto rn = instance->getGraph()->getReverseMapping(removed);
            auto tn = instance->getGraph()->getReverseMapping(target);
            auto nn = instance->getGraph()->getReverseMapping(n);
        }

        virtual inline void merge(node_id removed, node_id u, node_id v, cost_id cu, cost_id cv) {
            auto rn = instance->getGraph()->getReverseMapping(removed);
            auto un = instance->getGraph()->getReverseMapping(u);
            auto vn = instance->getGraph()->getReverseMapping(v);
        }
    private:
        bool addedPreselected_ = false;
    };
}


// TODO: Bound NTDK?

#endif //STEINER_REDUCTION_H
