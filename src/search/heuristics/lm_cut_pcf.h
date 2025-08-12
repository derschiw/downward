#ifndef HEURISTICS_LM_CUT_PCF_H
#define HEURISTICS_LM_CUT_PCF_H

#include <iostream>

namespace lm_cut_heuristic {
class LandmarkCutCore;
class LandmarkCutHeuristicExploration;

enum class PCFStrategy {
    HMAX,
    HADD,
    RANDOM,
    EGREEDY,
};

class PreconditionChoiceFunction {
protected:
    PCFStrategy pcf_strategy;
public:
    std::unique_ptr<LandmarkCutHeuristicExploration> get_heuristic_exploration(LandmarkCutCore &core);
    PreconditionChoiceFunction(const PCFStrategy &pcf_strategy);
};

std::ostream &operator<<(std::ostream &os, const PCFStrategy &pcf_strategy);
}

#endif
