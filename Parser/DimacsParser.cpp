//
// Created by aschidler on 1/23/20.
//

#include "DimacsParser.h"
#include "../SteinerInstance.h"
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "../Steiner.h"

steiner::SteinerInstance* steiner::DimacsParser::parse(std::string& file) {
    fstream fl;

    fl.open(file, ios::in);
    auto* g = new Graph();
    auto ts = vector<node_id>();
    cost_id lineResult[3];

    //TODO: Error Handling
    if (fl.is_open()) {
        string ln;

        // Read file
        while (getline(fl, ln)) {
            boost::algorithm::to_lower(ln);

            bool is_terminal = false;
            bool is_edge = false;
            int token = 0;

            if (ln.length() > 0) {
                istringstream iss(ltrim(ln));
                string tok;

                for (long i = 0; getline(iss, tok, ' '); i++) {
                    tok = ltrim(tok);
                    // Skip leading spaces
                    if (tok.length() == 0)
                        continue;

                    if (is_edge || is_terminal) {
                        lineResult[token] = std::stoul(tok);
                        token++;
                    }

                    if (tok == "e") {
                        is_edge = true;
                    } else if (tok == "t") {
                        is_terminal = true;
                    }
                }
                if (is_terminal) {
                    ts.emplace_back((node_id)lineResult[0]);
                } else if (is_edge) {
                    g->addMappedEdge((node_id) lineResult[0], (node_id) lineResult[1], lineResult[2]);
                }
            }
        }
    }

    return new SteinerInstance(g, &ts);
}