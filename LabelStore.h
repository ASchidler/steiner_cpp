//
// Created by aschidler on 1/22/20.
//

#ifndef STEINER_CPP_LABELSTORE_H
#define STEINER_CPP_LABELSTORE_H
#include <cstdint>
#include <boost/dynamic_bitset.hpp>
#include <list>
#include "LabelIterator.h"

using namespace boost;

namespace steiner {
    class LabelStore {
    public:
        LabelStore(unsigned int width, unsigned int nNodes) : width(width), nNodes(nNodes) {}
        virtual void addLabel(unsigned int node, dynamic_bitset<> *newLabel) = 0;
        virtual LabelIterator* findLabels(unsigned int node, dynamic_bitset<> *target) = 0;
    protected:
        unsigned int width;
        unsigned int nNodes;
    };
}
#endif //STEINER_CPP_LABELSTORE_H
