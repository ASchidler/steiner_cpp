//
// Created by asc on 16.04.20.
//

#ifndef STEINER_LINKEDSTACK_H
#define STEINER_LINKEDSTACK_H
#include "../Steiner.h"
namespace steiner {
    template<class T>
    class LinkedStack {
    public:
        ~LinkedStack() {
            while (cNode_ != nullptr) {
                auto cn = cNode_;
                cNode_ = cn->prev_;
                delete cn;
            }
        }
        bool hasNext() {
            return cNode_ != nullptr;
        }
        T pop() {
            auto elem = cNode_->payload_;
            auto cn = cNode_;
            cNode_ = cNode_->prev_;
            delete cn;
            return elem;
        }

        template<typename... Ts>
        void emplace(Ts&&... args) {
            auto n = new LinkedStackNode<T>(cNode_, std::forward<Ts>(args)...);
            if (cNode_ == nullptr || n->payload_ > cNode_->payload_)
                cNode_ = n;
            else {
                n->prev_ = cNode_->prev_;
                cNode_->prev_ = n;
            }
        }

        void push(T& elem) {
            auto n = new LinkedStackNode<T>(cNode_, elem);
            if (cNode_ == nullptr || n->payload_ > cNode_->payload_)
                cNode_ = n;
            else {
                n->prev_ = cNode_->prev_;
                cNode_->prev_ = n;
            }
        }

        T& peek() {
            return cNode_->payload_;
        }
    private:
        template<class T2>
        class LinkedStackNode {
        public:
            template<typename... Ts2>
            explicit LinkedStackNode(LinkedStackNode<T2>* prev, Ts2&&... args) :prev_(prev), payload_(std::forward<Ts2>(args)...) {
            }
            T2 payload_;
            LinkedStackNode<T2>* prev_;
        };

        LinkedStackNode<T>* cNode_ = nullptr;
    };
}
#endif //STEINER_LINKEDSTACK_H
