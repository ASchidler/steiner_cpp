//
// Created by andre on 25.01.20.
//

#ifndef STEINER_DEGREEREDUCTION_H
#define STEINER_DEGREEREDUCTION_H

#include "Reduction.h"
#include "../Graph.h"

namespace steiner {
    class DegreeReduction : Reduction {
    public:
        node_id reduce(SteinerInstance *instance, node_id currCount, node_id prevCount) override;
        void postProcess(Graph *solution) override;
    private:
        bool ran_ = false;
        vector<Edge> contracted;
    };
}


#endif //STEINER_DEGREEREDUCTION_H
