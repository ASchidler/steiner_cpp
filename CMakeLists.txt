cmake_minimum_required(VERSION 3.12)
project(steiner LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

option(STEINER_BUILD_STATIC       "whether or not to link statically (if supported)" OFF)

if (STEINER_BUILD_STATIC)
  # force static runtime
  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++")
  string(REGEX REPLACE "/MD" "/MT" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
endif()

find_package(Boost REQUIRED)

add_executable(steiner Graph.h LabelStore.h SteinerInstance.h main.cpp Graph.cpp HsvSolver.cpp HsvSolver.h HashSetLabelStore.cpp
        HashSetLabelStore.h Parser/SteinerParser.h Parser/DimacsParser.cpp Parser/DimacsParser.h LabelIterator.h
        Heuristic/SteinerHeuristic.h Heuristic/MstHeuristic.cpp Heuristic/MstHeuristic.h Heuristic/DualAscentHeuristic.cpp
        Heuristic/DualAscentHeuristic.h Algorithms/DualAscent.cpp Algorithms/DualAscent.h Algorithms/ShortestPath.cpp
        Algorithms/ShortestPath.h Steiner.h Reductions/Reduction.h Reductions/DegreeReduction.cpp Reductions/DegreeReduction.h SteinerInstance.cpp SteinerTree.h Reductions/Reducer.cpp Reductions/Reducer.h Algorithms/SteinerLength.cpp Algorithms/SteinerLength.h Reductions/LongEdgeReduction.cpp Reductions/LongEdgeReduction.h Reductions/SdcReduction.cpp Reductions/SdcReduction.h Reductions/NtdkReduction.cpp Reductions/NtdkReduction.h Reductions/TerminalDistanceReduction.cpp Reductions/TerminalDistanceReduction.h Reductions/Degree3Reduction.cpp Reductions/Degree3Reduction.h Reductions/DualAscentReduction.cpp Reductions/DualAscentReduction.h Reductions/HeavyEdgeReduction.cpp Reductions/HeavyEdgeReduction.h Reductions/ZeroEdgePreselection.cpp Reductions/ZeroEdgePreselection.h Reductions/MstPreselection.cpp Reductions/MstPreselection.h Algorithms/UnionFind.h Reductions/NearestVertexPreselection.cpp Reductions/NearestVertexPreselection.h Reductions/ShortLinksPreselection.cpp Reductions/ShortLinksPreselection.h Reductions/QuickCollection.h Algorithms/LocalOptimization.cpp Algorithms/LocalOptimization.h Structures/BucketQueue.h Algorithms/GraphPruner.cpp Algorithms/GraphPruner.h
        Algorithms/VoronoiDiagram.h Structures/Queue.h Structures/LinkedStack.h
        dynamic/SteinerHeuristic.h dynamic/DualAscentHeuristic.h dynamic/DualAscentHeuristic.cpp dynamic/LabelStore.h
        dynamic/LabelIterator.h dynamic/HashSetLabelStore.h dynamic/HashSetLabelStore.cpp dynamic/HsvSolver.h dynamic/HsvSolver.cpp
        dynamic/MstHeuristic.h dynamic/MstHeuristic.cpp
        HybridSolver.h)
SET_TARGET_PROPERTIES(steiner PROPERTIES LINKER_LANGUAGE CXX)
