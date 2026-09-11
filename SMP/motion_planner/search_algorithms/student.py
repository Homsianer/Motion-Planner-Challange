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

        dist_manhattan = self.calc_heuristic_distance(path_last[-1], distance_type=1)
        if dist_manhattan is None:
            return np.inf

        cost_lanelet, final_lanelet_id, start_lanelet_id = self.calc_heuristic_lanelet(
            path_last
        )
        if cost_lanelet is None or final_lanelet_id[0] is None:
            return np.inf

        # hard discard if the goal lanelet cannot be reached from here
        cost_lanelet_graph = self.dict_lanelets_costs[final_lanelet_id[0]]
        if cost_lanelet_graph == -1:
            return np.inf

        angleToGoal = self.calc_angle_to_goal(path_last[-1])
        orientationToGoalDiff = self.calc_orientation_diff(
            angleToGoal, path_last[-1].orientation
        )

        num_obstacles = self.num_obstacles_in_lanelet_at_time_step(
            path_last[-1].time_step, final_lanelet_id[0]
        )

        cost_time = self.calc_time_cost(path_last)

        if hasattr(self.planningProblem.goal.state_list[0], "velocity"):
            # target the upper bound of the goal's velocity interval: going
            # faster (up to the allowed max) is cheaper than going slower
            v_max_goal = self.planningProblem.goal.state_list[0].velocity.end
            dist_vel = abs(path_last[-1].velocity - v_max_goal)
        else:
            dist_vel = 0

        weights = np.zeros(6)
        weights[0] = 1.0  # manhattan distance to goal
        weights[1] = 1.0  # velocity difference to goal's max velocity
        weights[2] = 1.0  # orientation difference to goal
        weights[3] = 1.0  # obstacles in current lanelet
        weights[4] = 1.0  # time cost so far
        weights[5] = 1.0  # lanelet-graph distance to goal lanelet

        cost = (
            weights[0] * dist_manhattan
            + weights[1] * dist_vel
            + weights[2] * abs(orientationToGoalDiff)
            + weights[3] * num_obstacles
            + weights[4] * cost_time
            + weights[5] * cost_lanelet_graph
        )

        if cost < 0:
            cost = 0
        return cost
