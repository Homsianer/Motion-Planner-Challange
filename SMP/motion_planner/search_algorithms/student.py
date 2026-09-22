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

    def __init__(
        self, scenario, planningProblem, automata, plot_config=DefaultPlotConfig
    ):
        super().__init__(
            scenario=scenario,
            planningProblem=planningProblem,
            automaton=automata,
            plot_config=plot_config,
        )



    def evaluation_function(self, node_current: PriorityNode) -> float:
        node_current.priority = self.heuristic_function(node_current=node_current)
        return node_current.priority

    def heuristic_function(self, node_current: PriorityNode) -> float:
        path_last = node_current.list_paths[-1]
        initial_state = node_current.list_paths[0][0]

        dist_manhattan = self.calc_heuristic_distance(path_last[-1], distance_type=1)
        if dist_manhattan is None:
            return np.inf

        # NOTE: GreedyBestFirstSearch's evaluation_function is f(n) = h(n) only,
        # there is no g(n) accumulated over the path (see best_first_search.py).
        # So this heuristic is the only thing guiding the search, and nothing
        # here currently stops it from regressing/oscillating - it would make
        # sense to also penalize (not necessarily hard-discard) nodes whose
        # dist_manhattan increased compared to the state before this primitive.

        # normalize by the distance at the very start of the search, so this
        # term is roughly a "fraction of the trip left" instead of raw meters
        dist_manhattan_start = self.calc_heuristic_distance(
            initial_state, distance_type=1
        )
        dist_manhattan_norm = dist_manhattan / dist_manhattan_start if dist_manhattan_start else dist_manhattan

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
        # than a far one, and no obstacle at all costs nothing
        dist_closest_obstacle = self.calc_dist_to_closest_obstacle(
            final_lanelet_id[0], path_last[-1].position, path_last[-1].time_step
        )
        if np.isinf(dist_closest_obstacle):
            cost_obstacle = 0.0
        else:
            cost_obstacle = 1.0 / (dist_closest_obstacle + 1e-6)

        # time cost of the whole trajectory so far (from the initial state),
        # not just of the last primitive
        cost_time = self.calc_time_cost([initial_state, path_last[-1]])
        if hasattr(self.planningProblem.goal.state_list[0], "time_step"):
            time_goal_max = self.planningProblem.goal.state_list[0].time_step.end
        else:
            time_goal_max = None
        cost_time_norm = cost_time / time_goal_max if time_goal_max else cost_time #im else falls ggf nicht gut genormt

        if hasattr(self.planningProblem.goal.state_list[0], "velocity"):
            # target the upper bound of the goal's velocity interval: going
            # faster (up to the allowed max) is cheaper than going slower
            v_max_goal = self.planningProblem.goal.state_list[0].velocity.end
            dist_vel = abs(path_last[-1].velocity - v_max_goal)
            dist_vel_norm = dist_vel / v_max_goal if v_max_goal else dist_vel ##im else falls ggf nicht gut genormt
        else:
            dist_vel_norm = 0

        weights = np.zeros(6)
        weights[0] = 1.0  # manhattan distance to goal (normalized)
        weights[1] = 1.0  # velocity difference to goal's max velocity (normalized)
        weights[2] = 1.0  # orientation difference to goal (normalized)
        weights[3] = 1.0  # proximity to closest obstacle in current lanelet
        weights[4] = 1.0  # time cost of trajectory so far (normalized)
        weights[5] = 1.0  # lanelet-graph distance to goal lanelet (normalized)

        cost = (
            weights[0] * dist_manhattan_norm
            + weights[1] * dist_vel_norm
            + weights[2] * orientation_diff_norm
            + weights[3] * cost_obstacle
            + weights[4] * cost_time_norm
            + weights[5] * cost_lanelet_graph_norm
        )

        if cost < 0:
            cost = 0
        return cost

