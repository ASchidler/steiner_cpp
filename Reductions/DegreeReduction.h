//
// Created by andre on 25.01.20.
//

#ifndef STEINER_DEGREEREDUCTION_H
#define STEINER_DEGREEREDUCTION_H

#include "Reduction.h"
#include "../Graph.h"

using std::string;

namespace steiner {
    class DegreeReduction : public Reduction {
    public:
        explicit DegreeReduction(SteinerInstance* instance, bool contractFirst) : Reduction(instance), ran_(contractFirst) {

        }
        node_id reduce(node_id currCount, node_id prevCount) override;
        string getName() override {
            return "Degree Reduction";
        }
    private:
        bool ran_;
    };
}


#endif //STEINER_DEGREEREDUCTION_H
