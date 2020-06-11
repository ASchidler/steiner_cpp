//
// Created by aschidler on 1/23/20.
//

#ifndef DYNAMIC_STEINER_LABELITERATOR_H
#define DYNAMIC_STEINER_LABELITERATOR_H

#include "Steiner.h"

using namespace boost;

namespace steiner {
    class DynamicLabelIterator {
    public:
        virtual ~DynamicLabelIterator() {}
        virtual const dynamic_bitset<> &operator*() = 0;
        virtual dynamic_bitset<> *operator->() = 0;
        virtual DynamicLabelIterator &operator++() = 0;
        virtual bool hasNext() = 0;
        //virtual const ConstLabelIterator operator++(int) = 0;
    };
}

#endif //DYNAMIC_STEINER_LABELITERATOR_H
