#include "lm_cut_landmarks.h"

#include "../task_utils/task_properties.h"

#include <algorithm>
#include <limits>
#include <utility>

using namespace std;

namespace lm_cut_heuristic {
/*******************************************************
 * LANDMARK CUT CORE
 *******************************************************/

/**
 * @brief Constructor for LandmarkCutCore.
 *
 * Initializes the core by building propositions and relaxed operators.
 * From here the
 */
LandmarkCutCore::LandmarkCutCore(const TaskProxy &task_proxy) {
    task_properties::verify_no_axioms(task_proxy);
    task_properties::verify_no_conditional_effects(task_proxy);

    // Build propositions.
    num_propositions = 2; // artificial goal and artificial precondition
    VariablesProxy variables = task_proxy.get_variables();
    propositions.resize(variables.size());
    for (FactProxy fact : variables.get_facts()) {
        int var_id = fact.get_variable().get_id();
        propositions[var_id].push_back(RelaxedProposition());
        ++num_propositions;
    }

    // Build relaxed operators for operators and axioms.
    for (OperatorProxy op : task_proxy.get_operators())
        build_relaxed_operator(op);

    // Simplify relaxed operators.
    // simplify();
    /* TODO: Put this back in and test if it makes sense,
       but only after trying out whether and how much the change to
       unary operators hurts. */

    // Build artificial goal proposition and operator.
    vector<RelaxedProposition *> goal_op_pre, goal_op_eff;
    for (FactProxy goal : task_proxy.get_goals()) {
        goal_op_pre.push_back(get_proposition(goal));
    }
    goal_op_eff.push_back(&artificial_goal);
    /* Use the invalid operator ID -1 so accessing
       the artificial operator will generate an error. */
    add_relaxed_operator(move(goal_op_pre), move(goal_op_eff), -1, 0);

    // Cross-reference relaxed operators.
    for (RelaxedOperator &op : relaxed_operators) {
        for (RelaxedProposition *pre : op.preconditions)
            pre->precondition_of.push_back(&op);
        for (RelaxedProposition *eff : op.effects)
            eff->effect_of.push_back(&op);
    }
}

/**
 * @brief Build a relaxed operator from an OperatorProxy.
 */
void LandmarkCutCore::build_relaxed_operator(const OperatorProxy &op) {
    vector<RelaxedProposition *> precondition;
    vector<RelaxedProposition *> effects;
    for (FactProxy pre : op.get_preconditions()) {
        precondition.push_back(get_proposition(pre));
    }
    for (EffectProxy eff : op.get_effects()) {
        effects.push_back(get_proposition(eff.get_fact()));
    }
    add_relaxed_operator(
        move(precondition), move(effects), op.get_id(), op.get_cost());
}

/**
 * @brief Add a relaxed operator to the core.
 */
void LandmarkCutCore::add_relaxed_operator(
    vector<RelaxedProposition *> &&precondition,
    vector<RelaxedProposition *> &&effects,
    int op_id, int base_cost) {
    RelaxedOperator relaxed_op(move(precondition), move(effects), op_id, base_cost);
    if (relaxed_op.preconditions.empty())
        relaxed_op.preconditions.push_back(&artificial_precondition);
    relaxed_operators.push_back(relaxed_op);
}

/**
 * @brief Get the proposition for a given fact.
 */
RelaxedProposition *LandmarkCutCore::get_proposition(
    const FactProxy &fact) {
    int var_id = fact.get_variable().get_id();
    int val = fact.get_value();
    return &propositions[var_id][val];
}


/*******************************************************
 * HEURISTIC EXPLORATION
 *******************************************************/

/**
 * @brief Setup the exploration queue for the first phase.
 */
void LandmarkCutHeuristicExploration::setup_exploration_queue() {
    priority_queue.clear();

    for (auto &var_props : core.propositions) {
        for (RelaxedProposition &prop : var_props) {
            prop.status = UNREACHED;
            prop.h_add_cost = numeric_limits<int>::max();
        }
    }

    core.artificial_goal.status = UNREACHED;
    core.artificial_goal.h_add_cost = numeric_limits<int>::max();
    core.artificial_precondition.status = UNREACHED;
    core.artificial_precondition.h_add_cost = numeric_limits<int>::max();

    for (RelaxedOperator &op : core.relaxed_operators) {
        op.unsatisfied_preconditions = op.preconditions.size();
        op.h_max_supporter = nullptr;
        op.h_max_cost = numeric_limits<int>::max();
        op.h_add_cost = numeric_limits<int>::max();
        op.selected_precondition = nullptr;
    }
}

/**
 * @brief Setup the exploration queue with the initial state.
 */
void LandmarkCutHeuristicExploration::setup_exploration_queue_state(const State &state) {
    for (FactProxy init_fact : state) {
        RelaxedProposition *prop = core.get_proposition(init_fact);
        prop->h_add_cost = 0;  // Initialize h_add_cost for initial facts
        enqueue_if_necessary(prop, 0);
    }
    core.artificial_precondition.h_add_cost = 0;  // Initialize h_add_cost for artificial precondition
    enqueue_if_necessary(&core.artificial_precondition, 0);
}

/**
 * @brief Enqueue a relaxed proposition if necessary.
 */
void LandmarkCutHeuristicExploration::enqueue_if_necessary(RelaxedProposition *prop, int cost) {
    assert(cost >= 0);
    if (prop->status == UNREACHED || prop->h_max_cost > cost) {
        prop->status = REACHED;
        prop->h_max_cost = cost;
        priority_queue.push(cost, prop);
    }
}

/**
 * @brief Enqueue a relaxed proposition if necessary.
 */
void LandmarkCutHeuristicExploration::h_max_exploration(const State &state) {
    // Initialize (setup)
    assert(priority_queue.empty());
    setup_exploration_queue();
    setup_exploration_queue_state(state);

    // Use Dijkstra-like exploration to compute h_max values.
    // These are all propositions reachable from the initial state,
    // ordered by their minimal cost.
    while (!priority_queue.empty()) {
        pair<int, RelaxedProposition *> top_pair = priority_queue.pop();
        int popped_cost = top_pair.first;
        RelaxedProposition *prop = top_pair.second;
        int prop_cost = prop->h_max_cost;
        assert(prop_cost <= popped_cost);
        if (prop_cost < popped_cost)
            continue;
        const vector<RelaxedOperator *> &triggered_operators =
            prop->precondition_of;
        // Examine all operators having this proposition as a
        // precondition.
        for (RelaxedOperator *relaxed_op : triggered_operators) {
            // we now have one unsatisfied precondition less
            --relaxed_op->unsatisfied_preconditions;
            // we cannot have less than 0 unsatisfied preconditions
            assert(relaxed_op->unsatisfied_preconditions >= 0);

            // If all preconditions are satisfied, we can use this proposition
            // as the heuristic supporter.
            if (relaxed_op->unsatisfied_preconditions == 0) {
                // As the priority queue is ordered by cost, we can safely
                // assign the h_max supporter and its cost.
                relaxed_op->h_max_supporter = prop;
                relaxed_op->h_max_cost = prop->h_max_cost;

                // Effect can be achieved for prop_cost + relaxed_op->cost.
                int target_cost = prop->h_max_cost + relaxed_op->cost;
                for (RelaxedProposition *effect : relaxed_op->effects) {
                    enqueue_if_necessary(effect, target_cost);
                }

                // Select a precondition according to the strategy.
                select_precondition(*relaxed_op);
            }
        }
    }
}

/**
 * @brief Enqueue a relaxed proposition if necessary.
 */
void LandmarkCutHeuristicExploration::h_max_exploration_incremental(vector<RelaxedOperator *> &cut) {
    assert(priority_queue.empty());
    /* We pretend that this queue has had as many pushes already as we
       have propositions to avoid switching from bucket-based to
       heap-based too aggressively. This should prevent ever switching
       to heap-based in problems where action costs are at most 1.
    */
    priority_queue.add_virtual_pushes(core.num_propositions);

    // Enqueue the effects of the cut operators.
    for (RelaxedOperator *relaxed_op : cut) {
        int cost = relaxed_op->h_max_cost + relaxed_op->cost;
        for (RelaxedProposition *effect : relaxed_op->effects)
            enqueue_if_necessary(effect, cost);
    }
    while (!priority_queue.empty()) {
        pair<int, RelaxedProposition *> top_pair = priority_queue.pop();
        int popped_cost = top_pair.first;
        RelaxedProposition *prop = top_pair.second;
        int prop_cost = prop->h_max_cost;
        assert(prop_cost <= popped_cost);
        if (prop_cost < popped_cost)
            continue;
        const vector<RelaxedOperator *> &triggered_operators =
            prop->precondition_of;

        // Examine all operators having this proposition as a precondition.
        for (RelaxedOperator *relaxed_op : triggered_operators) {
            if (relaxed_op->h_max_supporter == prop) {
                int old_supp_cost = relaxed_op->h_max_cost;
                if (old_supp_cost > prop->h_max_cost) {
                    // Now we update the supporter to the best precondition.
                    assert(!relaxed_op->unsatisfied_preconditions);
                    for (size_t i = 0; i < relaxed_op->preconditions.size(); ++i)
                        if (relaxed_op->preconditions[i]->h_max_cost > relaxed_op->h_max_supporter->h_max_cost)
                            relaxed_op->h_max_supporter = relaxed_op->preconditions[i];
                    relaxed_op->h_max_cost = relaxed_op->h_max_supporter->h_max_cost;

                    // Then we select a precondition according to the strategy.
                    select_precondition(*relaxed_op);

                    int new_supp_cost = relaxed_op->h_max_cost;
                    if (new_supp_cost != old_supp_cost) {
                        // This operator has become cheaper.
                        assert(new_supp_cost < old_supp_cost);
                        int target_cost = new_supp_cost + relaxed_op->cost;
                        for (RelaxedProposition *effect : relaxed_op->effects)
                            enqueue_if_necessary(effect, target_cost);
                    }
                }
            }
        }
    }
}

/*******************************************************
 * H_MAX EXPLORATION
 *******************************************************/

/**
 * @brief Select the precondition for h_max exploration.
 *
 * Here the selected precondition is simply the h_max supporter.
 */
void LandmarkCutHMaxExploration::select_precondition(RelaxedOperator &op) const {
    op.selected_precondition = op.h_max_supporter;
}

/**
 * @brief Validate the h_max values.
 */
void LandmarkCutHMaxExploration::validate() const {
#ifndef NDEBUG
    // Using conditional compilation to avoid complaints about unused
    // variables when using NDEBUG. This whole code does nothing useful
    // when assertions are switched off anyway.
    for (const RelaxedOperator &op : core.relaxed_operators) {
        if (op.unsatisfied_preconditions) {
            bool reachable = true;
            for (RelaxedProposition *pre : op.preconditions) {
                if (pre->status == UNREACHED) {
                    reachable = false;
                    break;
                }
            }
            assert(!reachable);
            assert(!op.h_max_supporter);
        } else {
            assert(op.h_max_supporter);
            int h_max_cost = op.h_max_cost;
            assert(h_max_cost == op.h_max_supporter->h_max_cost);
            for (RelaxedProposition *pre : op.preconditions) {
                assert(pre->status != UNREACHED);
                assert(pre->h_max_cost <= h_max_cost);
            }
        }
    }
#endif
}

/*******************************************************
 * H_ADD EXPLORATION
 *******************************************************/

/**
 * @brief Update the h_add supporters for all operators.
 *
 * This function updates the heuristic supporters for all operators
 * based on the current state of the propositions.
 */
void LandmarkCutHAddExploration::select_precondition(RelaxedOperator &op) const {
    // We select the supporter with the highest h_add cost.
    RelaxedProposition *supporter = nullptr;
    int supporter_cost = 0;
    int max_cost = -1;
    for (size_t i = 0; i < op.preconditions.size(); ++i) {
        if (op.preconditions[i]->h_add_cost > max_cost) {
            max_cost = op.preconditions[i]->h_add_cost;
            supporter = op.preconditions[i];
        }
        supporter_cost += op.preconditions[i]->h_add_cost;
    }
    op.h_add_cost = supporter_cost;
    op.selected_precondition = supporter;

    // Now we have to update the effects' h_add costs.
    for (RelaxedProposition *effect : op.effects) {
        effect->h_add_cost = min(op.h_add_cost + op.cost, effect->h_add_cost);
    }
}

/**
 * @brief Validate the h_add values.
 */
void LandmarkCutHAddExploration::validate() const {
#ifndef NDEBUG
    assert(1);
#endif
}

/*******************************************************
 * RANDOM EXPLORATION
 *******************************************************/

/**
 * @brief Update the random supporters for all operators.
 *
 * This function updates the heuristic supporters for all operators
 * based a random choice of the preconditions avoiding preconditions
 * that are already in the GOAL_ZONE.
 */
void LandmarkCutAlmostRandomExploration::select_precondition(RelaxedOperator &op) const {
    // TODO: dont use preconditions that are allready in the GOAL_ZONE
    assert(!op.unsatisfied_preconditions);

    // Collect candidate preconditions
    std::vector<RelaxedProposition *> candidates;
    candidates.reserve(op.preconditions.size());
    for (RelaxedProposition *pre : op.preconditions) {
        if (pre->status != GOAL_ZONE && pre->status != UNREACHED) {
            candidates.push_back(pre);
        }
    }

    // Fallback if all in GOAL_ZONE / UNREACHED
    const std::vector<RelaxedProposition *> &pool =
        candidates.empty() ? op.preconditions : candidates;

    op.selected_precondition = pool[rng.random(pool.size())];
}

/**
 * @brief Update the random supporters for all operators.
 *
 * This function updates the heuristic supporters for all operators
 * based a random choice of the preconditions excluding preconditions
 * that have h_max cost zero if possible.
 */
void LandmarkCutTotallyRandomExploration::select_precondition(RelaxedOperator &op) const {
    assert(!op.unsatisfied_preconditions);

    // Collect candidate preconditions
    std::vector<RelaxedProposition *> candidates;
    candidates.reserve(op.preconditions.size());
    for (RelaxedProposition *pre : op.preconditions) {
        if (pre->h_max_cost != 0) {
            candidates.push_back(pre);
        }
    }

    // Fallback if all in GOAL_ZONE / UNREACHED
    const std::vector<RelaxedProposition *> &pool =
        candidates.empty() ? op.preconditions : candidates;

    op.selected_precondition = pool[rng.random(pool.size())];
}

/**
 * @brief Pseudo-validate the random exploration (prevent compiler errors).
 */
void LandmarkCutRandomExploration::validate() const {
#ifndef NDEBUG
    assert(1);
#endif
}

/*******************************************************
 * BACKWARD EXPLORATION
 *******************************************************/

/**
 * @brief Perform the forward exploration to identify the cut after marking the goal zone.
 */
void LandmarkCutBackwardExploration::cut_computation(
    const State &state, vector<RelaxedProposition *> &cut_computation_queue,
    vector<RelaxedOperator *> &cut) {
    assert(cut_computation_queue.empty());
    assert(cut.empty());

    // Connect all initial propositions to the artificial precondition.
    core.artificial_precondition.status = BEFORE_GOAL_ZONE;
    cut_computation_queue.push_back(&core.artificial_precondition);

    for (FactProxy init_fact : state) {
        RelaxedProposition *init_prop = core.get_proposition(init_fact);
        init_prop->status = BEFORE_GOAL_ZONE;
        cut_computation_queue.push_back(init_prop);
    }

    while (!cut_computation_queue.empty()) {
        RelaxedProposition *prop = cut_computation_queue.back();
        cut_computation_queue.pop_back();
        const vector<RelaxedOperator *> &triggered_operators =
            prop->precondition_of;
        for (RelaxedOperator *relaxed_op : triggered_operators) {
            if (relaxed_op->selected_precondition == prop) {
                bool reached_goal_zone = false;

                // Here we check if the operator can reach the goal zone.
                // If it can, we add it to the cut, as it bridges the gap
                // between the initial state region and the goal zone.
                for (RelaxedProposition *effect : relaxed_op->effects) {
                    if (effect->status == GOAL_ZONE) {
                        // With h_random we can potentially end up
                        // having the artificial goal operator in the cut
                        // assert(relaxed_op->cost > 0);
                        reached_goal_zone = true;
                        cut.push_back(relaxed_op);
                        break;
                    }
                }
                // If the operator does not reach the goal zone, the effects
                // are added to BEFORE_GOAL_ZONE.
                if (!reached_goal_zone) {
                    for (RelaxedProposition *effect : relaxed_op->effects) {
                        if (effect->status != BEFORE_GOAL_ZONE) {
                            assert(effect->status == REACHED);
                            effect->status = BEFORE_GOAL_ZONE;
                            cut_computation_queue.push_back(effect);
                        }
                    }
                }
            }
        }
    }
}

/**
 * @brief Mark the goal plateau.
 *
 * The goal plateau is the set of propositions that can be reached
 * from the goal using zero-cost actions.
 */
void LandmarkCutBackwardExploration::mark_goal_plateau(RelaxedProposition *subgoal) {
    // NOTE: subgoal can be null if we got here via recursion through
    // a zero-cost action that is relaxed unreachable. (This can only
    // happen in domains which have zero-cost actions to start with.)
    // For example, this happens in pegsol-strips #01.
    if (subgoal && subgoal->status != GOAL_ZONE) {
        subgoal->status = GOAL_ZONE;
        for (RelaxedOperator *achiever : subgoal->effect_of)
            if (achiever->cost == 0)
                mark_goal_plateau(achiever->selected_precondition);
    }
}

/*******************************************************
 * LANDMARK CUT LANDMARKS
 *******************************************************/
/**
 * @brief Initialize the landmark cut landmarks.
 */

LandmarkCutLandmarks::LandmarkCutLandmarks(const TaskProxy &task_proxy, const PCFStrategy &pcf_strategy, unsigned int seed)
    : core(task_proxy),
      backward(core),
      precondition_choice_function(pcf_strategy, seed) {
    heuristic = precondition_choice_function.get_heuristic_exploration(core);
}

/**
 * @brief Constructor for LandmarkCutLandmarks.
 *
 * Initializes the core, backward exploration, and heuristic exploration.
 * The precondition choice function is set based on the provided strategy.
 */
bool LandmarkCutLandmarks::compute_landmarks(
    const State &state,
    const CostCallback &cost_callback,
    const LandmarkCallback &landmark_callback
    ) {
    for (RelaxedOperator &op : core.relaxed_operators) {
        op.cost = op.base_cost;
    }
    // The following three variables could be declared inside the loop
    // ("cut_computation_queue" even inside cut_computation),
    // but having them here saves reallocations and hence provides a
    // measurable speed boost.
    vector<RelaxedOperator *> cut;
    Landmark landmark;
    vector<RelaxedProposition *> cut_computation_queue;

    // First forward exploration to compute the h_max values.
    heuristic->h_max_exploration(state);
    // validate_h_max();  // too expensive to use even in regular debug mode

    // If there are no reachable propositions, we have a dead end.
    if (core.artificial_goal.status == UNREACHED)
        return true;

    // we continue as long as the artificial goal is not reached
    while (core.artificial_goal.h_max_cost != 0) {
        // Here we would first calculate the justification graph,
        // for the current problem. We do not need this here, as we
        // store the h_max_supporter in the relaxed operators.
        // This essentially is the justification graph, but
        // without the need to compute it explicitly.

        // First we mark the goal zone
        backward.mark_goal_plateau(&core.artificial_goal);
        assert(cut.empty());

        // Backward exploration to find the cut (operators that
        // can reach the goal zone).
        cut_computation_queue.clear();
        backward.cut_computation(state, cut_computation_queue, cut);
        assert(!cut.empty());

        // here we find the minimum cost of the cut (starting
        // with a cut_cost of 'infinity')
        int cut_cost = numeric_limits<int>::max();
        for (RelaxedOperator *op : cut)
            cut_cost = min(cut_cost, op->cost);
        // then we substract that minimum cost from the cut
        // operators' costs
        for (RelaxedOperator *op : cut)
            op->cost -= cut_cost;

        // This callback returns the cost of the cut to
        // the caller (if it is not null).
        if (cost_callback) {
            cost_callback(cut_cost);
        }
        // This callback returns the cut as a vector of operator indices
        // to the caller (if it is not null).
        if (landmark_callback) {
            landmark.clear();
            for (RelaxedOperator *op : cut) {
                landmark.push_back(op->original_op_id);
            }
            landmark_callback(landmark, cut_cost);
        }

        // Compute the new heuristic values for the next round efficiently.
        // heuristic->h_max_exploration_incremental(cut);
        heuristic->h_max_exploration(state);
        // heuristic->validate();  // too expensive to use even in regular debug mode
        cut.clear();

        /*
          Note: This could perhaps be made more efficient, for example by
          using a round-dependent counter for GOAL_ZONE and BEFORE_GOAL_ZONE,
          or something based on total_cost, so that we don't need a per-round
          reinitialization.
        */
        for (auto &var_props : core.propositions) {
            for (RelaxedProposition &prop : var_props) {
                if (prop.status == GOAL_ZONE || prop.status == BEFORE_GOAL_ZONE)
                    prop.status = REACHED;
            }
        }
        core.artificial_goal.status = REACHED;
        core.artificial_precondition.status = REACHED;
    }
    return false;
}
}
