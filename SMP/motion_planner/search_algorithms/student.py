import numpy as np
from SMP.motion_planner.node import PriorityNode

from SMP.motion_planner.plot_config import DefaultPlotConfig
from SMP.motion_planner.search_algorithms.best_first_search import GreedyBestFirstSearch


class StudentMotionPlanner(GreedyBestFirstSearch):
    """
    Motion planner implementation by students.
    Note that you may inherit from any given motion planner as you wish, or come up with your own planner.
    Here as an example, the planner is inherited from the GreedyBestFirstSearch planner.
    """


    DEFAULT_WEIGHTS = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

    # radius (in meters) beyond which a nearby obstacle no longer affects the
    # obstacle term - keeps it bounded to [0, 1] instead of the old 1/x, which
    # could blow up to arbitrarily large values right next to an obstacle
    OBSTACLE_DANGER_RADIUS = 5.0

    # fallback normalizer for the velocity term when the goal's own max
    # velocity is 0 (e.g. "come to a stop") - see heuristic_function() below
    NOMINAL_VELOCITY_SCALE = 10.0

    # caps for terms that aren't naturally bounded to ~[0, 1] (e.g. a node
    # moving away from the goal, or the search running past the goal's time
    # window) - without this a single outlier node could dominate the sum
    # regardless of its weight
    DIST_MANHATTAN_NORM_CAP = 2.0
    DIST_VEL_NORM_CAP = 2.0
    COST_TIME_NORM_CAP = 2.0

    def __init__(
        self, scenario, planningProblem, automata, plot_config=DefaultPlotConfig
    ):
        super().__init__(
            scenario=scenario,
            planningProblem=planningProblem,
            automaton=automata,
            plot_config=plot_config,
        )
        # instance copy so a tuning script can do `planner.weights = ...`
        # without mutating the class-level default for other instances
        self.weights = np.array(self.DEFAULT_WEIGHTS, dtype=float)

    def evaluation_function(self, node_current: PriorityNode) -> float:
        node_current.priority = self.heuristic_function(node_current=node_current)
        return node_current.priority

    def heuristic_function(self, node_current: PriorityNode) -> float:
        path_last = node_current.list_paths[-1]
        initial_state = node_current.list_paths[0][0]

        dist_manhattan = self.calc_heuristic_distance(path_last[-1], distance_type=1)
        if dist_manhattan is None:
            return np.inf

        # normalize by the distance at the very start of the search, so this
        # term is roughly a "fraction of the trip left" instead of raw meters
        dist_manhattan_start = self.calc_heuristic_distance(
            initial_state, distance_type=1
        )
        if dist_manhattan_start is not None and dist_manhattan_start > 0:
            dist_manhattan_norm = dist_manhattan / dist_manhattan_start
        else:
            dist_manhattan_norm = dist_manhattan
        # moving away from the goal can push this above 1 - cap it so one
        # regressing node can't dominate the weighted sum
        dist_manhattan_norm = min(dist_manhattan_norm, self.DIST_MANHATTAN_NORM_CAP)

        final_lanelet_id = self.scenario.lanelet_network.find_lanelet_by_position(
            [path_last[-1].position]
        )

        if final_lanelet_id[0] is None:
            return np.inf

        # hard discard if the goal lanelet cannot be reached from here
        cost_lanelet_graph = self.dict_lanelets_costs[final_lanelet_id[0]]
        if cost_lanelet_graph == -1:
            return np.inf

        # normalize by the number of lanelets in the map as a rough upper bound
        num_lanelets = len(self.scenario.lanelet_network.lanelets)
        cost_lanelet_graph_norm = (
            cost_lanelet_graph / num_lanelets if num_lanelets else cost_lanelet_graph
        )

        angleToGoal = self.calc_angle_to_goal(path_last[-1])
        orientationToGoalDiff = self.calc_orientation_diff(
            angleToGoal, path_last[-1].orientation
        )
        # abs(orientationToGoalDiff) is always within [0, pi], so this is
        # already bounded to [0, 1]
        orientation_diff_norm = abs(orientationToGoalDiff) / np.pi

        # penalize proximity to the closest obstacle in the current lanelet
        # instead of just counting obstacles, so a close obstacle costs more
        # than a far one, and no obstacle at all costs nothing. Saturating
        # (instead of 1/x) keeps this bounded to [0, 1] like the other terms -
        # 1/x could otherwise blow up to arbitrary size right next to an
        # obstacle and dominate the sum regardless of its weight.
        dist_closest_obstacle = self.calc_dist_to_closest_obstacle(
            final_lanelet_id[0], path_last[-1].position, path_last[-1].time_step
        )
        if np.isinf(dist_closest_obstacle):
            cost_obstacle = 0.0
        else:
            cost_obstacle = max(
                0.0, 1.0 - dist_closest_obstacle / self.OBSTACLE_DANGER_RADIUS
            )

        # time cost of the whole trajectory so far (from the initial state),
        # not just of the last primitive
        cost_time = self.calc_time_cost([initial_state, path_last[-1]])
        if hasattr(self.planningProblem.goal.state_list[0], "time_step"):
            time_goal_max = self.planningProblem.goal.state_list[0].time_step.end
        else:
            time_goal_max = None

        if time_goal_max is not None and time_goal_max > 0:
            cost_time_norm = cost_time / time_goal_max
        else:
            cost_time_norm = cost_time
        cost_time_norm = min(cost_time_norm, self.COST_TIME_NORM_CAP)

        if hasattr(self.planningProblem.goal.state_list[0], "velocity"):
            # target the upper bound of the goal's velocity interval: going
            # faster (up to the allowed max) is cheaper than going slower
            v_max_goal = self.planningProblem.goal.state_list[0].velocity.end
            dist_vel = abs(path_last[-1].velocity - v_max_goal)

            if v_max_goal is not None and v_max_goal > 0:
                v_scale = v_max_goal
            else:
                v_scale = self.NOMINAL_VELOCITY_SCALE
            dist_vel_norm = min(dist_vel / v_scale, self.DIST_VEL_NORM_CAP)
        else:
            dist_vel_norm = 0

        cost = (
            self.weights[0] * dist_manhattan_norm
            + self.weights[1] * dist_vel_norm
            + self.weights[2] * orientation_diff_norm
            + self.weights[3] * cost_obstacle
            + self.weights[4] * cost_time_norm
            + self.weights[5] * cost_lanelet_graph_norm
        )

        if cost < 0:
            cost = 0
        return cost

