//
// Created by asc on 15.04.20.
//

#ifndef STEINER_QUEUE_H
#define STEINER_QUEUE_H

#define BUCKET_LIMIT 10000
#include "../Steiner.h"
#include "BucketQueue.h"

namespace steiner {
    template<typename T>
    class Queue {
    public:
        explicit Queue(cost_id upperBound) : pqueue_(priority_queue<T>()),
        bqueue_(BucketQueue<T>(upperBound < BUCKET_LIMIT ? upperBound : 0)) {
            bucket_ = upperBound < BUCKET_LIMIT;
        }

        template<typename... Ts>
        void emplace(cost_id cost, Ts&&... args) {
            if (bucket_) {
                bqueue_.enqueue(cost, std::forward<Ts>(args)...);
            } else {
                pqueue_.emplace(std::forward<Ts>(args)...);
            }
        }

        bool empty() {
            return (bucket_ && !bqueue_.hasNext()) || (!bucket_ && pqueue_.empty());
        }

        T dequeue() {
            if (bucket_)
                return bqueue_.dequeue();
            else {
                auto elem = pqueue_.top();
                pqueue_.pop();
                return elem;
            }
        }
    private:
        bool bucket_;
        priority_queue<T> pqueue_;
        BucketQueue<T> bqueue_;
    };
}
#endif //STEINER_QUEUE_H
