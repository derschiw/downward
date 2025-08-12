#ifndef HEURISTICS_LM_CUT_LANDMARKS_H
#define HEURISTICS_LM_CUT_LANDMARKS_H

#include "../task_proxy.h"
#include "lm_cut_pcf.h"

#include "../algorithms/priority_queues.h"

#include <cassert>
#include <functional>
#include <memory>
#include <vector>
#include <random>


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

    int count = 0;
    int cost;
    int unsatisfied_preconditions;
    int heuristic_supporter_cost; // e.g. h_max_cost of heuristic_supporter
    RelaxedProposition *heuristic_supporter;
    RelaxedOperator(std::vector<RelaxedProposition *> &&pre,
                    std::vector<RelaxedProposition *> &&eff,
                    int op_id, int base)
        : original_op_id(op_id), preconditions(pre), effects(eff), base_cost(base),
          cost(-1), unsatisfied_preconditions(-1), heuristic_supporter_cost(-1),
          heuristic_supporter(nullptr) {
    }
};

struct RelaxedProposition {
    std::vector<RelaxedOperator *> precondition_of;
    std::vector<RelaxedOperator *> effect_of;

    PropositionStatus status;
    int heuristic_cost;
    int count = 0;
};


class LandmarkCutCore {
    void build_relaxed_operator(const OperatorProxy &op);
    void add_relaxed_operator(std::vector<RelaxedProposition *> &&precondition,
                              std::vector<RelaxedProposition *> &&effects,
                              int op_id, int base_cost);
public:
    std::vector<RelaxedOperator> relaxed_operators;
    std::vector<std::vector<RelaxedProposition>> propositions;
    RelaxedProposition artificial_precondition;
    RelaxedProposition artificial_goal;
    int num_propositions;
    RelaxedProposition *get_proposition(const FactProxy &fact);

    void print() const {
        std::cout << "LandmarkCutCore with " << relaxed_operators.size()
                  << " operators and " << num_propositions << " propositions." << std::endl;
        for (const auto &var_props : propositions) {
            for (const RelaxedProposition &prop : var_props) {
                std::cout << "Proposition: " << prop.heuristic_cost
                          << ", status: " << prop.status
                          << ", count: " << prop.count << std::endl;
            }
        }

        for (const RelaxedOperator &op : relaxed_operators) {
            std::cout << "Operator ID: " << op.original_op_id
                      << ", base cost: " << op.base_cost
                      << ", cost: " << op.cost
                      << ", unsatisfied preconditions: " << op.unsatisfied_preconditions
                      << ", heuristic supporter cost: " << op.heuristic_supporter_cost
                      << ", count: " << op.count << std::endl;
            for (RelaxedProposition *pre : op.preconditions) {
                std::cout << "  Precondition: " << pre
                          << ", status: " << pre->status
                          << ", count: " << pre->count
                          << ", h_cost: " << pre->heuristic_cost << std::endl;
            }
        }
    }

    LandmarkCutCore(const TaskProxy &task_proxy);
    LandmarkCutCore() = default;
};

class LandmarkCutHeuristicExploration {
protected:
    priority_queues::AdaptiveQueue<RelaxedProposition *> priority_queue;
    LandmarkCutCore &core;
    virtual void update_supporters(RelaxedOperator &op) const = 0;

public:
    void setup_exploration_queue();
    void setup_exploration_queue_state(const State &state);
    void heuristic_exploration(const State &state);
    void heuristic_exploration_incremental(std::vector<RelaxedOperator *> &cut);
    void enqueue_if_necessary(RelaxedProposition *prop, int cost);
    virtual void trigger_operators(RelaxedOperator *relaxed_op, RelaxedProposition *prop) = 0;
    virtual void trigger_operators_incremental(RelaxedOperator *relaxed_op, RelaxedProposition *prop) = 0;
    virtual void validate() const = 0;

    virtual ~LandmarkCutHeuristicExploration() = default;
    LandmarkCutHeuristicExploration(LandmarkCutCore &core_ref)
        : core(core_ref) {}
};

class LandmarkCutHMaxExploration : public LandmarkCutHeuristicExploration {
protected:
    void update_supporters(RelaxedOperator &op) const override;
public:
    void trigger_operators(RelaxedOperator *relaxed_op, RelaxedProposition *prop) override;
    void trigger_operators_incremental(RelaxedOperator *relaxed_op, RelaxedProposition *prop) override;

    void validate() const override;
    LandmarkCutHMaxExploration(LandmarkCutCore &core_ref)
        : LandmarkCutHeuristicExploration(core_ref) {}
};

class LandmarkCutHAddExploration : public LandmarkCutHeuristicExploration {
protected:
    void update_supporters(RelaxedOperator &op) const override;
public:
    void trigger_operators(RelaxedOperator *relaxed_op, RelaxedProposition *prop) override;
    void trigger_operators_incremental(RelaxedOperator *relaxed_op, RelaxedProposition *prop) override;
    void validate() const override;

    LandmarkCutHAddExploration(LandmarkCutCore &core_ref)
        : LandmarkCutHeuristicExploration(core_ref) {}
};

class LandmarkCutRandomExploration : public LandmarkCutHeuristicExploration {
    mutable std::mt19937 rd_generator; // TODO: use random generator from lab

protected:
    void update_supporters(RelaxedOperator &op) const override;
public:
    void trigger_operators(RelaxedOperator *relaxed_op, RelaxedProposition *prop) override;
    void trigger_operators_incremental(RelaxedOperator *relaxed_op, RelaxedProposition *prop) override;
    void validate() const override;

    LandmarkCutRandomExploration(LandmarkCutCore &core_ref)
        : LandmarkCutHeuristicExploration(core_ref), rd_generator(std::random_device{}()) {}
};

class LandmarkCutEGreedyExploration : public LandmarkCutHeuristicExploration {
    float epsilon = 0.8f;
    mutable std::mt19937 rd_generator;

protected:
    void update_supporters(RelaxedOperator &op) const override;
public:
    void trigger_operators(RelaxedOperator *relaxed_op, RelaxedProposition *prop) override;
    void trigger_operators_incremental(RelaxedOperator *relaxed_op, RelaxedProposition *prop) override;
    void validate() const override;

    LandmarkCutEGreedyExploration(LandmarkCutCore &core_ref)
        : LandmarkCutHeuristicExploration(core_ref), rd_generator(std::random_device{}()) {
    }
};

class LandmarkCutBackwardExploration {
public:
    LandmarkCutCore &core;
    void cut_computation(const State &state,
                         std::vector<RelaxedProposition *> &cut_computation_queue,
                         std::vector<RelaxedOperator *> &cut);
    void mark_goal_plateau(RelaxedProposition *subgoal);

    LandmarkCutBackwardExploration(LandmarkCutCore &core_ref)
        : core(core_ref) {}
};


class LandmarkCutLandmarks {
    LandmarkCutCore core;
    std::unique_ptr<LandmarkCutHeuristicExploration> heuristic;
    LandmarkCutBackwardExploration backward;
    PreconditionChoiceFunction precondition_choice_function;

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
}

#endif
