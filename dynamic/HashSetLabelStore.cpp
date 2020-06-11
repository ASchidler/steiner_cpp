//
// Created by aschidler on 1/22/20.
//

#include "HashSetLabelStore.h"

void steiner::DynamicHashSetLabelStore::addLabel(node_id node, const dynamic_bitset<>* newLabel) {
    this->labels_[node].emplace(*newLabel);
}

DynamicHashSetLabelIterator* steiner::DynamicHashSetLabelStore::findLabels(node_id node, const dynamic_bitset<>* target) {
    return new DynamicHashSetLabelIterator(labels_[node].begin(), labels_[node].end(), target);
}

// TODO: Create labels centrally and then just link to them? Would that be feasible... (how to identify them...)
// TODO: Create datastructures that are cache aware for iterating through the values...

const dynamic_bitset<>& steiner::DynamicHashSetLabelIterator::operator*() {
    return *pos;
}

dynamic_bitset<>* DynamicHashSetLabelIterator::operator->() {
    auto* ptr = const_cast<dynamic_bitset<> *>(&(*pos));
    return ptr;
}

DynamicLabelIterator& steiner::DynamicHashSetLabelIterator::operator++() {
    pos++;
    findNext();
    return *this;
}
// TODO: Change everything possible to references and use const where possible
// TODO: Store a pointer to the costs, so we can immediately deliver the costs?
void DynamicHashSetLabelIterator::findNext() {
    while(pos != end) {
        bool disjoint = true;
        for (size_t i = 0; i < target->num_blocks(); ++i) {
            if ((target->m_bits[i] & pos->m_bits[i]) > 0) {
                disjoint = false;
                break;
            }
        }

        if (disjoint)
            break;
        pos++;
    }
}

bool DynamicHashSetLabelIterator::hasNext() {
    return pos != end;
}
