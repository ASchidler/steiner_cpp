//
// Created by andre on 24.01.20.
//

#ifndef STEINER_REDUCTION_H
#define STEINER_REDUCTION_H
#include "../SteinerInstance.h"
#include "../Steiner.h"
namespace steiner {
    class Reduction {
        virtual node_id reduce(SteinerInstance* instance, node_id currCount, node_id prevCount) = 0;
        virtual void postProcess(Graph* solution) = 0;
    };
}

#endif //STEINER_REDUCTION_H
