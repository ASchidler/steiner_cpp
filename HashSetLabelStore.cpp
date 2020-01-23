//
// Created by aschidler on 1/22/20.
//

#include "HashSetLabelStore.h"

void steiner::HashSetLabelStore::addLabel(unsigned int node, dynamic_bitset<>* newLabel) {
    this->labels_[node].insert(*newLabel);
}

HashSetLabelIterator* steiner::HashSetLabelStore::findLabels(unsigned int node, dynamic_bitset<>* target) {
    return new HashSetLabelIterator(labels_[node].begin(), labels_[node].end(), target);
}

// TODO: Create labels centrally and then just link to them? Would that be feasible... (how to identify them...)
// TODO: Create datastructures that are cache aware for iterating through the values...

const dynamic_bitset<>& steiner::HashSetLabelIterator::operator*() {
    return *pos;
}

dynamic_bitset<>* HashSetLabelIterator::operator->() {
    auto* ptr = const_cast<dynamic_bitset<> *>(&(*pos));
    return ptr;
}

LabelIterator& steiner::HashSetLabelIterator::operator++() {
    pos++;
    findNext();
    return *this;
}

void HashSetLabelIterator::findNext() {
    while(pos != end) {
        bool disjoint = true;

        auto cbit = target->find_first();
        while (cbit < target->size()) {
            if (pos->test(cbit)) {
                disjoint = false;
                break;
            }
            cbit = target->find_next(cbit);
        }

        if (disjoint) {
            break;
        }
        pos++;
    }
}

bool HashSetLabelIterator::hasNext() {
    return pos != end;
}
