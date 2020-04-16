//
// Created by asc on 16.04.20.
//

#ifndef STEINER_LINKEDLIST_H
#define STEINER_LINKEDLIST_H
#include "../Steiner.h"
namespace steiner {
    template<class T>
    class LinkedListNode {
    public:
        template<typename... Ts>
        explicit LinkedListNode(Ts&&... args) : payload_(std::forward<Ts>(args)...) {
        }
    private:
        T payload_;
    };

    template<class T>
    class LinkedList {
        public LinkedList() {

        }
    private:
        LinkedListN
    };
}
#endif //STEINER_LINKEDLIST_H
