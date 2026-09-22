"""
Systematic weight search for StudentMotionPlanner.heuristic_function, using
Optuna (Bayesian / TPE-based black-box optimization over the 5 free weights -
weights[0] is fixed to 1.0, see the comment in student.py for why).

Requirements: `pip install optuna` inside the same environment you normally
run the batch-processing scripts in (commonroad-py311 / the docker container).

Usage (from the repository root):
    python -m SMP.batch_processing.tune_weights

Tune the constants in the "Tuning configuration" section below before a real
run - the defaults are intentionally small (few scenarios, short timeout, few
trials) so a first run finishes in a couple of minutes and you can sanity
check the setup before scaling up.
"""

import copy
import random
import time
import warnings

import numpy as np
import optuna

import SMP.batch_processing.helper_functions as hf
import SMP.batch_processing.timeout_config
from SMP.batch_processing.process_scenario import get_planning_problem_and_id
from SMP.batch_processing.scenario_loader import ScenarioConfig
from SMP.motion_planner.plot_config import StudentScriptPlotConfig
from SMP.motion_planner.search_algorithms.student import StudentMotionPlanner

# ---------------------------------------------------------------------------
# Tuning configuration - adjust before a real run
# ---------------------------------------------------------------------------

# size of the representative scenario subset evaluated per trial (there are
# ~500 scenarios in scenarios/exercise/ - way too many to run per trial)
NUM_TUNING_SCENARIOS = 15

# per-scenario timeout used *during tuning* - shorter than the real 30s
# competition timeout so trials finish fast; validate the final weights
# against the real timeout afterwards (see main() below)
TUNING_TIMEOUT_SEC = 8.0

# number of weight vectors Optuna tries
N_TRIALS = 40

# fixed seed so the tuning subset is reproducible across runs
RANDOM_SEED = 42

# search range applied to each of the 5 free weights (weights[0] is fixed)
WEIGHT_SEARCH_RANGE = (0.0, 5.0)

# cost assigned to a scenario that is not solved within TUNING_TIMEOUT_SEC.
# Must be > 1.0 (the worst-case cost of a *solved* scenario, elapsed/timeout
# capped at 1.0) so Optuna always prefers "solved but slow" over "not
# solved" - i.e. solve-rate is optimized first, speed only as a tiebreaker.
FAILURE_PENALTY = 2.0


def pick_tuning_subset(scenario_loader, configuration, def_automaton, num_scenarios, seed):
    """
    Picks a reproducible random subset of scenarios that use the default
    vehicle type, so this script doesn't need to replicate the per-scenario
    automaton-override logic from process_scenario.py.
    """
    candidates = []
    for scenario_id in scenario_loader.scenario_ids:
        scenario_config = ScenarioConfig(scenario_id, configuration)
        if scenario_config.vehicle_type == def_automaton.type_vehicle:
            candidates.append(scenario_id)

    rng = random.Random(seed)
    rng.shuffle(candidates)
    return candidates[:num_scenarios]


def run_single_scenario(
    scenario_id, scenario_loader, configuration, def_automaton, weights, timeout_sec
):
    """
    Runs StudentMotionPlanner with the given weight vector on one scenario.
    Mirrors the scenario/automaton loading in process_scenario.py, but calls
    the planner directly in-process (no subprocess per scenario) so many
    trials can run back-to-back quickly.

    Returns (solved: bool, elapsed_sec: float).
    """
    scenario_config = ScenarioConfig(scenario_id, configuration)

    automaton = copy.deepcopy(def_automaton)
    automaton.deserialize()

    scenario, planning_problem_set = scenario_loader.load_scenario(scenario_id)
    planning_problem, _ = get_planning_problem_and_id(
        planning_problem_set, scenario_config.planning_problem_idx
    )

    SMP.batch_processing.timeout_config.use_sequential_processing = True
    SMP.batch_processing.timeout_config.timeout = timeout_sec

    planner = StudentMotionPlanner(
        scenario,
        planning_problem,
        automata=automaton,
        plot_config=StudentScriptPlotConfig(DO_PLOT=False),
    )
    planner.weights = weights

    start = time.perf_counter()
    try:
        path_solution, _, _ = planner.execute_search()
        elapsed = time.perf_counter() - start
        return path_solution is not None, elapsed
    except Exception:
        # covers both the "Time Out" exception raised inside execute_search
        # and any unexpected error - both count as "not solved" for tuning
        elapsed = time.perf_counter() - start
        return False, elapsed


def make_objective(scenario_ids, scenario_loader, configuration, def_automaton, timeout_sec):
    def objective(trial: optuna.Trial) -> float:
        weights = np.array(
            [
                1.0,  # fixed - only ratios between weights matter, see student.py
                trial.suggest_float("w_velocity", *WEIGHT_SEARCH_RANGE),
                trial.suggest_float("w_orientation", *WEIGHT_SEARCH_RANGE),
                trial.suggest_float("w_obstacle", *WEIGHT_SEARCH_RANGE),
                trial.suggest_float("w_time", *WEIGHT_SEARCH_RANGE),
                trial.suggest_float("w_lanelet_graph", *WEIGHT_SEARCH_RANGE),
            ]
        )

        costs = []
        for scenario_id in scenario_ids:
            solved, elapsed = run_single_scenario(
                scenario_id, scenario_loader, configuration, def_automaton, weights, timeout_sec
            )
            costs.append(elapsed / timeout_sec if solved else FAILURE_PENALTY)

        return float(np.mean(costs))

    return objective


def main():
    warnings.filterwarnings("ignore")

    configuration, logger, scenario_loader, def_automaton, _ = hf.init_processing(
        "Weight Tuning", for_multi_processing=False
    )

    scenario_ids = pick_tuning_subset(
        scenario_loader, configuration, def_automaton, NUM_TUNING_SCENARIOS, RANDOM_SEED
    )
    print(f"Tuning on {len(scenario_ids)} scenarios (timeout={TUNING_TIMEOUT_SEC}s/scenario):")
    for scenario_id in scenario_ids:
        print(f"  - {scenario_id}")

    study = optuna.create_study(direction="minimize")
    study.optimize(
        make_objective(
            scenario_ids, scenario_loader, configuration, def_automaton, TUNING_TIMEOUT_SEC
        ),
        n_trials=N_TRIALS,
    )

    print("\nBest weights found:")
    print("  w_manhattan     = 1.000  (fixed)")
    for name, value in study.best_params.items():
        print(f"  {name:<15} = {value:.3f}")
    print(f"Best objective (mean cost over tuning subset): {study.best_value:.3f}")
    print(
        "\nNext: plug these weights into StudentMotionPlanner.DEFAULT_WEIGHTS in "
        "student.py and validate on the full scenario set with the real 30s "
        "timeout (e.g. via batch_processing_sequential.py) before trusting them."
    )

    # returned (instead of only printed) so a Jupyter notebook can capture it,
    # e.g. `study = main()`, and inspect/plot it further with
    # optuna.visualization.plot_optimization_history(study) etc.
    return study


if __name__ == "__main__":
    main()
