//
// Created by aschidler on 1/23/20.
//

#ifndef STEINER_STEINERPARSER_H
#define STEINER_STEINERPARSER_H

#include "../SteinerInstance.h"
#include <string>

namespace steiner {
    class SteinerParser {
    public:
        virtual ~SteinerParser() {}
        virtual steiner::SteinerInstance* parse(std::string& path) = 0;
    };
}
#endif //STEINER_STEINERPARSER_H
