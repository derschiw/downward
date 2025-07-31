#include "lm_cut_pcf.h"

#include "../utils/logging.h"
#include "../utils/system.h"
#include "../plugins/plugin.h"

using namespace std;
namespace lm_cut_heuristic {
PreconditionChoiceFunction::PreconditionChoiceFunction(
    const PCFStrategy &pcf_strategy) :
    pcf_strategy(pcf_strategy) {
    utils::g_log << "PCF Strategy: " << pcf_strategy << endl;
}

ostream &operator<<(ostream &os, const PCFStrategy &pcf) {
    switch (pcf) {
    case PCFStrategy::HMAX:
        return os << "hmax";
    case PCFStrategy::HADD:
        return os << "hadd";
    case PCFStrategy::RANDOM:
        return os << "random";
    default:
        cerr << "Name of PCFStrategy not known";
        utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
    }
}

static plugins::TypedEnumPlugin<PCFStrategy> _mutex_type_enum_plugin({
        {"hmax", "pcf using hmax"},
        {"hadd", "pcf using hadd"},
        {"random", "pcf using random selection"}
    });
}
