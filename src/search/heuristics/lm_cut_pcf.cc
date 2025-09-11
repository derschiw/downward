#include "lm_cut_pcf.h"
#include "lm_cut_landmarks.h"

#include "../utils/logging.h"
#include "../utils/system.h"
#include "../plugins/plugin.h"

using namespace std;
namespace lm_cut_heuristic {
PreconditionChoiceFunction::PreconditionChoiceFunction(
    const PCFStrategy &pcf_strategy,
    unsigned int seed
    ) : pcf_strategy(pcf_strategy), seed(seed) {
    utils::g_log << "PCF Strategy: " << pcf_strategy << endl;
    utils::g_log << "PCF Seed: " << seed << endl;
}

std::unique_ptr<LandmarkCutHeuristicExploration> PreconditionChoiceFunction::get_heuristic_exploration(LandmarkCutCore &core) {
    if (pcf_strategy == PCFStrategy::HMAX) {
        return std::make_unique<LandmarkCutHMaxExploration>(core);
    } else if (pcf_strategy == PCFStrategy::HADD) {
        return std::make_unique<LandmarkCutHAddExploration>(core);
    } else if (pcf_strategy == PCFStrategy::RANDOM) {
        return std::make_unique<LandmarkCutTotallyRandomExploration>(core, seed);
    } else if (pcf_strategy == PCFStrategy::ALMOST_RANDOM) {
        return std::make_unique<LandmarkCutAlmostRandomExploration>(core, seed);
    } else {
        throw std::runtime_error("Unsupported PCF strategy");
    }
}

ostream &operator<<(ostream &os, const PCFStrategy &pcf) {
    switch (pcf) {
    case PCFStrategy::HMAX:
        return os << "hmax";
    case PCFStrategy::HADD:
        return os << "hadd";
    case PCFStrategy::RANDOM:
        return os << "random";
    case PCFStrategy::ALMOST_RANDOM:
        return os << "almost_random";
    default:
        cerr << "Name of PCFStrategy not known";
        utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
    }
}

static plugins::TypedEnumPlugin<PCFStrategy> _mutex_type_enum_plugin({
        {"hmax", "pcf using hmax"},
        {"hadd", "pcf using hadd"},
        {"random", "pcf using random selection"},
        {"almost_random", "pcf using random selection avoiding preconditions already in the GOAL_ZONE"}
    });
}
