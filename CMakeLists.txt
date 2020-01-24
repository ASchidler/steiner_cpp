project(steiner LANGUAGES CXX)
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)

add_executable(steiner Graph.h LabelStore.h SteinerSolver.h SteinerInstance.h main.cpp Graph.cpp HsvSolver.cpp HsvSolver.h HashSetLabelStore.cpp HashSetLabelStore.h Parser/SteinerParser.h Parser/DimacsParser.cpp Parser/DimacsParser.h LabelIterator.h Heuristic/SteinerHeuristic.h Heuristic/MstHeuristic.cpp Heuristic/MstHeuristic.h Heuristic/DualAscentHeuristic.cpp Heuristic/DualAscentHeuristic.h)
SET_TARGET_PROPERTIES(steiner PROPERTIES LINKER_LANGUAGE CXX)