#ifndef HEURISTICS_LM_CUT_PCF_H
#define HEURISTICS_LM_CUT_PCF_H

#include <iostream>

namespace lm_cut_heuristic {
enum class PCFStrategy {
    HMAX,
    HADD,
    RANDOM
};

class PreconditionChoiceFunction {
protected:
    PCFStrategy pcf_strategy;
public:
    PreconditionChoiceFunction(const PCFStrategy &pcf_strategy);
};

std::ostream &operator<<(std::ostream &os, const PCFStrategy &pcf_strategy);
}

#endif
