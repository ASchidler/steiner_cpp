//
// Created on 1/22/20.
//

#include "HashSetLabelStore.h"

template <typename T>
void steiner::HashSetLabelStore<T>::addLabel(node_id node, const T newLabel) {
    this->labels_[node].emplace(newLabel);
}

template <typename T>
HashSetLabelIterator<T>* steiner::HashSetLabelStore<T>::findLabels(node_id node, const T target) {
    return new HashSetLabelIterator<T>(labels_[node].begin(), labels_[node].end(), target);
}

template<typename T>
void HashSetLabelStore<T>::removeLabel(node_id node, const T newLabel) {
    this->labels_[node].erase(newLabel);
}

// TODO: Create labels centrally and then just link to them? Would that be feasible... (how to identify them...)
// TODO: Create datastructures that are cache aware for iterating through the values...
template <typename T>
const T steiner::HashSetLabelIterator<T>::operator*() {
    return *pos;
}

template <typename T>
const T HashSetLabelIterator<T>::operator->() {
    return *pos;
}

template <typename T>
LabelIterator<T>& steiner::HashSetLabelIterator<T>::operator++() {
    pos++;
    findNext();
    return *this;
}
// TODO: Change everything possible to references and use const where possible
// TODO: Store a pointer to the costs, so we can immediately deliver the costs?
template <typename T>
void HashSetLabelIterator<T>::findNext() {
    while(pos != end) {
        if ((target & *pos) == 0)
            break;
        pos++;
    }
}

template <typename T>
bool HashSetLabelIterator<T>::hasNext() {
    return pos != end;
}

template class steiner::HashSetLabelStore<uint16_t>;
template class steiner::HashSetLabelStore<uint32_t>;
template class steiner::HashSetLabelStore<uint64_t>;
template class steiner::HashSetLabelStore<boost::uint128_type>;

template class steiner::HashSetLabelIterator<uint16_t>;
template class steiner::HashSetLabelIterator<uint32_t>;
template class steiner::HashSetLabelIterator<uint64_t>;
template class steiner::HashSetLabelIterator<boost::uint128_type>;
