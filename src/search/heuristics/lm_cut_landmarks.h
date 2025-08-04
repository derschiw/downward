#ifndef HEURISTICS_LM_CUT_LANDMARKS_H
#define HEURISTICS_LM_CUT_LANDMARKS_H

#include "../task_proxy.h"
#include "lm_cut_pcf.h"

#include "../algorithms/priority_queues.h"

#include <cassert>
#include <functional>
#include <memory>
#include <vector>


namespace lm_cut_heuristic {
// TODO: Fix duplication with the other relaxation heuristics.
struct RelaxedProposition;

enum PropositionStatus {
    UNREACHED = 0,
    REACHED = 1,
    GOAL_ZONE = 2,
    BEFORE_GOAL_ZONE = 3
};

struct RelaxedOperator {
    int original_op_id;
    std::vector<RelaxedProposition *> preconditions;
    std::vector<RelaxedProposition *> effects;
    int base_cost; // 0 for axioms, 1 for regular operators

    int cost;
    int unsatisfied_preconditions;
    int heuristic_supporter_cost; // heuristic_cost of heuristic_supporter
    RelaxedProposition *heuristic_supporter;
    RelaxedOperator(std::vector<RelaxedProposition *> &&pre,
                    std::vector<RelaxedProposition *> &&eff,
                    int op_id, int base)
        : original_op_id(op_id), preconditions(pre), effects(eff), base_cost(base),
          cost(-1), unsatisfied_preconditions(-1), heuristic_supporter_cost(-1),
          heuristic_supporter(nullptr) {
    }

    inline void update_heuristic_supporter();
};

struct RelaxedProposition {
    std::vector<RelaxedOperator *> precondition_of;
    std::vector<RelaxedOperator *> effect_of;

    PropositionStatus status;
    int heuristic_cost;
};

class LandmarkCutLandmarks {
    std::vector<RelaxedOperator> relaxed_operators;
    std::vector<std::vector<RelaxedProposition>> propositions;
    RelaxedProposition artificial_precondition;
    RelaxedProposition artificial_goal;
    int num_propositions;
    priority_queues::AdaptiveQueue<RelaxedProposition *> priority_queue;
    PreconditionChoiceFunction precondition_choice_function;

    void build_relaxed_operator(const OperatorProxy &op);
    void add_relaxed_operator(std::vector<RelaxedProposition *> &&precondition,
                              std::vector<RelaxedProposition *> &&effects,
                              int op_id, int base_cost);
    RelaxedProposition *get_proposition(const FactProxy &fact);
    void setup_exploration_queue();
    void setup_exploration_queue_state(const State &state);
    void heuristic_exploration(const State &state);
    void heuristic_exploration_incremental(std::vector<RelaxedOperator *> &cut);
    void backward_exploration(const State &state,
                              std::vector<RelaxedProposition *> &backward_exploration_queue,
                              std::vector<RelaxedOperator *> &cut);

    void enqueue_if_necessary(RelaxedProposition *prop, int cost) {
        assert(cost >= 0);
        if (prop->status == UNREACHED || prop->heuristic_cost > cost) {
            prop->status = REACHED;
            prop->heuristic_cost = cost;
            priority_queue.push(cost, prop);
        }
    }

    void mark_goal_plateau(RelaxedProposition *subgoal);
    void validate_heuristic() const;
public:
    using Landmark = std::vector<int>;
    using CostCallback = std::function<void (int)>;
    using LandmarkCallback = std::function<void (const Landmark &, int)>;

    LandmarkCutLandmarks(const TaskProxy &task_proxy,
                         const PCFStrategy &pcf_strategy = PCFStrategy::HMAX);

    /*
      Compute LM-cut landmarks for the given state.

      If cost_callback is not nullptr, it is called once with the cost of each
      discovered landmark.

      If landmark_callback is not nullptr, it is called with each discovered
      landmark (as a vector of operator indices) and its cost. This requires
      making a copy of the landmark, so cost_callback should be used if only the
      cost of the landmark is needed.

      Returns true iff state is detected as a dead end.
    */
    bool compute_landmarks(const State &state, const CostCallback &cost_callback,
                           const LandmarkCallback &landmark_callback);
};

inline void RelaxedOperator::update_heuristic_supporter() {
    assert(!unsatisfied_preconditions);
    for (size_t i = 0; i < preconditions.size(); ++i)
        if (preconditions[i]->heuristic_cost > heuristic_supporter->heuristic_cost)
            heuristic_supporter = preconditions[i];
    heuristic_supporter_cost = heuristic_supporter->heuristic_cost;
}
}

#endif
