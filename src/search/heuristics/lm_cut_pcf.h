#ifndef HEURISTICS_LM_CUT_PCF_H
#define HEURISTICS_LM_CUT_PCF_H

#include <iostream>
#include <memory>
#include <random>

namespace lm_cut_heuristic {
class LandmarkCutCore;
class LandmarkCutHeuristicExploration;

enum class PCFStrategy {
    HMAX,
    HADD,
    RANDOM,
    ALMOST_RANDOM
};

class PreconditionChoiceFunction {
protected:
    PCFStrategy pcf_strategy;
    unsigned int seed;
public:
    std::unique_ptr<LandmarkCutHeuristicExploration> get_heuristic_exploration(LandmarkCutCore &core);
    PreconditionChoiceFunction(const PCFStrategy &pcf_strategy, unsigned int seed);
};

std::ostream &operator<<(std::ostream &os, const PCFStrategy &pcf_strategy);
}

#endif
