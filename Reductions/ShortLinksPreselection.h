//
// Created on 1/30/20.
//

#ifndef STEINER_SHORTLINKSPRESELECTION_H
#define STEINER_SHORTLINKSPRESELECTION_H

#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"

namespace  steiner {
    class ShortLinksPreselection : public Reduction {
    public:
        explicit ShortLinksPreselection(SteinerInstance *s) : Reduction(s) {
        }

        node_id reduce(node_id currCount, node_id prevCount) override;

        string getName() override {
            return "Short Links";
        }
    private:
        struct BridgingEdge{
            BridgingEdge() : c(MAXCOST), t(0), u(0),  v(0) {}


            node_id t;
            node_id u;
            node_id v;
            cost_id c;
        };
    };
}


#endif //STEINER_SHORTLINKSPRESELECTION_H
