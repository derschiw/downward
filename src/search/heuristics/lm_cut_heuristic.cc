#include "lm_cut_heuristic.h"

#include "lm_cut_landmarks.h"

#include "../task_proxy.h"

#include "../plugins/plugin.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"

#include <iostream>

using namespace std;

namespace lm_cut_heuristic {
LandmarkCutHeuristic::LandmarkCutHeuristic(
    const shared_ptr<AbstractTask> &transform, bool cache_estimates,
    const string &description, utils::Verbosity verbosity,
    const PCFStrategy &pcf_strategy)
    : Heuristic(transform, cache_estimates, description, verbosity),
      landmark_generator(make_unique<LandmarkCutLandmarks>(task_proxy, pcf_strategy)) {
    if (log.is_at_least_normal()) {
        log << "Initializing landmark cut heuristic..." << endl;
    }

    // task_properties::dump_task(task_proxy);
}

int LandmarkCutHeuristic::compute_heuristic(const State &ancestor_state) {
    // log << "" << endl;
    // task_properties::dump_fdr(ancestor_state);
    // task_properties::dump_pddl(ancestor_state);

    State state = convert_ancestor_state(ancestor_state);
    int total_cost = 0;
    bool dead_end = landmark_generator->compute_landmarks(
        state,
        [&total_cost](int cut_cost) {total_cost += cut_cost;},
        nullptr);

    if (dead_end)
        return DEAD_END;
    // log << total_cost << endl << endl;
    return total_cost;
}

class LandmarkCutHeuristicFeature
    : public plugins::TypedFeature<Evaluator, LandmarkCutHeuristic> {
public:
    LandmarkCutHeuristicFeature() : TypedFeature("lmcut") {
        document_title("Landmark-cut heuristic");

        add_heuristic_options_to_feature(*this, "lmcut");

        add_option<PCFStrategy>("pcfstrategy", "Precondition choice function seleciton", "hmax");

        document_language_support("action costs", "supported");
        document_language_support("conditional effects", "not supported");
        document_language_support("axioms", "not supported");

        document_property("admissible", "yes");
        document_property("consistent", "no");
        document_property("safe", "yes");
        document_property("preferred operators", "no");
    }

    virtual shared_ptr<LandmarkCutHeuristic>
    create_component(const plugins::Options &opts) const override {
        return plugins::make_shared_from_arg_tuples<LandmarkCutHeuristic>(
            get_heuristic_arguments_from_options(opts),
            opts.get<PCFStrategy>("pcfstrategy")
            );
    }
};

static plugins::FeaturePlugin<LandmarkCutHeuristicFeature> _plugin;
}
