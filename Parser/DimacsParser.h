//
// Created by aschidler on 1/23/20.
//

#ifndef STEINER_DIMACSPARSER_H
#define STEINER_DIMACSPARSER_H

#include "SteinerParser.h"
#include "../SteinerInstance.h"

namespace steiner {
    class DimacsParser : SteinerParser {
    public:
        ~DimacsParser() {

        }
        steiner::SteinerInstance* parse(string& path) override;


    private:
        static inline std::string &ltrim(std::string &s) {
            s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                            std::not1(std::ptr_fun<int, int>(std::isspace))));
            return s;
        }
    };
}
#endif //STEINER_DIMACSPARSER_H
