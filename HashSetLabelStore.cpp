//
// Created by aschidler on 1/22/20.
//

#include "HashSetLabelStore.h"

void steiner::HashSetLabelStore::addLabel(node_id node, const dynamic_bitset<>* newLabel) {
    this->labels_[node].emplace(*newLabel);
}

HashSetLabelIterator* steiner::HashSetLabelStore::findLabels(node_id node, const dynamic_bitset<>* target) {
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
// TODO: Change everything possible to references and use const where possible
// TODO: Store a pointer to the costs, so we can immediately deliver the costs?
void HashSetLabelIterator::findNext() {
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

bool HashSetLabelIterator::hasNext() {
    return pos != end;
}
