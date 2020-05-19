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
    cost_id lineResult[10]; // 3 should suffice, but use 10 for tolerance

    //TODO: Error Handling
    if (fl.is_open()) {
        string ln;

        // Read file
        while (getline(fl, ln)) {
            boost::algorithm::to_lower(ln);

            bool is_terminal = false;
            bool is_edge = false;
            bool is_edge_count = false;
            int token = 0;

            if (ln.length() > 0) {
                istringstream iss(ltrim(ln));
                string tok;

                for (long i = 0; getline(iss, tok, ' '); i++) {
                    tok = ltrim(tok);
                    // Skip leading spaces
                    if (tok.length() == 0)
                        continue;

                    if (is_edge || is_terminal || is_edge_count) {
                        lineResult[token] = std::stoul(tok);
                        token++;
                    }

                    if (i == 0) {
                        if (tok == "e" || tok == "a") {
                            is_edge = true;
                        } else if (tok == "t") {
                            is_terminal = true;
                        } else if (tok == "edges") {
                            is_edge_count = true;
                        }
                    }
                }
                if (is_terminal) {
                    ts.emplace_back((node_id)lineResult[0]);
                } else if (is_edge) {
                    g->addMappedEdge((node_id) lineResult[0], (node_id) lineResult[1], lineResult[2]);
                } else if (is_edge_count) {
                    if (lineResult[0] >= MAXNODE) {
                        cout << "Cannot process instances with more than "<< MAXNODE << " edges " << endl;
                        exit(10);
                    }
                }
            }
        }
    }

    return new SteinerInstance(g, &ts);
}