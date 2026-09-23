import copy
import time
from typing import Any, Dict, List, Tuple, Union

import numpy as np
from commonroad.geometry.shape import Circle
from commonroad.scenario.state import KSState

import SMP.batch_processing.timeout_config
from SMP.maneuver_automaton.motion_primitive import MotionPrimitive
from SMP.motion_planner.node import PriorityNode
from SMP.motion_planner.plot_config import DefaultPlotConfig
from SMP.motion_planner.search_algorithms.best_first_search import GreedyBestFirstSearch
from SMP.motion_planner.utility import (
    MotionPrimitiveStatus,
    initial_visualization,
    update_visualization,
)


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

    # TEMPORARY diagnostic switch: set True to print "nodes evaluated / sec"
    # every DEBUG_LOG_EVERY_N heuristic calls, to tell apart "too slow per
    # node" from "search never converges / explodes in branching". Flip back
    # to False before a real/graded batch run - it's not needed otherwise.
    DEBUG_LOG_PROGRESS = True
    DEBUG_LOG_EVERY_N = 2000

    # GreedyBestFirstSearch (best_first_search.py) does a pure TREE search -
    # no duplicate-state pruning at all - so it can re-explore practically
    # identical states (reached via different primitive sequences) over and
    # over, burning the whole search on wide, shallow branching instead of
    # making depth progress (confirmed via DEBUG_LOG_PROGRESS: thousands of
    # nodes evaluated while depth barely moves). execute_search() below adds
    # a closed list keyed on a DISCRETIZED state to fix that. Grid fine
    # enough to not merge genuinely different states, coarse enough to
    # actually catch near-duplicates - these are the tunable knobs.
    DEDUP_POSITION_RESOLUTION = 1.0  # meters
    DEDUP_ORIENTATION_RESOLUTION = 0.15  # radians (~8.6 degrees)
    DEDUP_VELOCITY_RESOLUTION = 1.0  # m/s

    def _state_key(self, state):
        """
        Discretized key for a state, used to decide whether two states are
        "practically the same" for search-pruning purposes. time_step is
        NOT rasterized - it's already a discrete integer, and merging states
        from different time steps would be wrong since obstacles move.
        """
        grid_pos = tuple(
            np.round(state.position / self.DEDUP_POSITION_RESOLUTION).astype(int)
        )
        grid_orientation = int(
            np.round(state.orientation / self.DEDUP_ORIENTATION_RESOLUTION)
        )
        grid_velocity = int(np.round(state.velocity / self.DEDUP_VELOCITY_RESOLUTION))
        return (state.time_step, grid_pos, grid_orientation, grid_velocity)

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
        self._num_heuristic_calls = 0
        self._search_start_time = None
        # caches the (expensive) obstacle lookup keyed on (lanelet_id,
        # time_step) - see _get_circle_obstacle_centers_in_lanelet() below
        self._obstacle_centers_cache = {}

    def evaluation_function(self, node_current: PriorityNode) -> float:
        node_current.priority = self.heuristic_function(node_current=node_current)
        return node_current.priority

    def _get_circle_obstacle_centers_in_lanelet(self, lanelet_id, time_step):
        """
        Cached replacement for the obstacle lookup inside
        SearchBaseClass.calc_dist_to_closest_obstacle(): scanning every
        obstacle in the whole scenario and checking lanelet containment
        (get_obstacles() in base_class.py) is the expensive part, and it
        only depends on (lanelet_id, time_step) - NOT on the querying
        position - so many nodes (e.g. siblings from the same parent, which
        share the same time_step, or nodes the closed list didn't merge but
        that still end up in the same lanelet) can reuse this instead of
        re-scanning from scratch on every single heuristic call.
        """
        cache_key = (lanelet_id, time_step)
        cached = self._obstacle_centers_cache.get(cache_key)
        if cached is not None:
            return cached

        lanelet_obj = self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
        obstacles_in_lanelet = self.get_obstacles(lanelet_obj, time_step)
        # same restriction as the original calc_dist_to_closest_obstacle:
        # only Circle-shaped obstacles are considered
        centers = [
            obstacle.occupancy_at_time(time_step).shape.center
            for obstacle in obstacles_in_lanelet
            if isinstance(obstacle.occupancy_at_time(time_step).shape, Circle)
        ]
        self._obstacle_centers_cache[cache_key] = centers
        return centers

    def calc_dist_to_closest_obstacle(self, lanelet_id, pos, time_step):
        """
        Same result as SearchBaseClass.calc_dist_to_closest_obstacle(), but
        via the cached lookup above - only the (cheap) distance-to-pos loop
        over the cached centers is redone per node, not the expensive
        whole-scenario obstacle scan.
        """
        centers = self._get_circle_obstacle_centers_in_lanelet(lanelet_id, time_step)
        shortest_dist = np.inf
        for center in centers:
            dist = self.distance(pos, center)
            if dist < shortest_dist:
                shortest_dist = dist
        return shortest_dist

    def heuristic_function(self, node_current: PriorityNode) -> float:
        if self.DEBUG_LOG_PROGRESS:
            if self._search_start_time is None:
                self._search_start_time = time.perf_counter()
            self._num_heuristic_calls += 1
            if self._num_heuristic_calls % self.DEBUG_LOG_EVERY_N == 0:
                elapsed = time.perf_counter() - self._search_start_time
                print(
                    f"[{self.scenario.scenario_id}] {self._num_heuristic_calls} nodes "
                    f"evaluated in {elapsed:.1f}s ({self._num_heuristic_calls / elapsed:.0f} nodes/s), "
                    f"depth of current node: {node_current.depth_tree}"
                )

        path_last = node_current.list_paths[-1]
        initial_state = node_current.list_paths[0][0]

        dist_manhattan = self.calc_heuristic_distance(path_last[-1], distance_type=1)
        if dist_manhattan is None:
            return np.inf


        dist_manhattan_before_primitive = self.calc_heuristic_distance(
            path_last[0], distance_type=1
        )
        if (
            dist_manhattan_before_primitive is not None
            and dist_manhattan > dist_manhattan_before_primitive
        ):
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


        final_lanelet_ids = self.lanelets_of_position(
            self.scenario.lanelet_network.find_lanelet_by_position(
                [path_last[-1].position]
            )[0],
            path_last[-1],
        )

        if not final_lanelet_ids:
            return np.inf

        final_lanelet_id = final_lanelet_ids[0]

        # hard discard if the goal lanelet cannot be reached from here
        cost_lanelet_graph = self.dict_lanelets_costs[final_lanelet_id]
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


        dist_closest_obstacle = self.calc_dist_to_closest_obstacle(
            final_lanelet_id, path_last[-1].position, path_last[-1].time_step
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

    def execute_search(
        self,
    ) -> Tuple[
        Union[None, List[List[KSState]]], Union[None, List[MotionPrimitive]], Any
    ]:
        """
        Same as BestFirstSearch.execute_search() (best_first_search.py), but
        with a closed list added: a node whose (discretized) state was
        already expanded before is skipped instead of being re-expanded, and
        a child leading to an already-expanded state is never even inserted
        into the frontier. See the comment above DEDUP_POSITION_RESOLUTION
        for why this is needed.
        """
        # fresh per search run
        self._closed_states = set()

        # for visualization in jupyter notebook
        list_status_nodes = []
        dict_status_nodes: Dict[int, Tuple] = {}

        # shift initial state of planning problem from vehicle center to rear axle position
        # (reference point of motion primitives)
        new_state_initial = self.state_initial.translate_rotate(
            -np.array(
                [
                    self.rear_ax_dist * np.cos(self.state_initial.orientation),
                    self.rear_ax_dist * np.sin(self.state_initial.orientation),
                ]
            ),
            0,
        )

        # first node
        node_initial = PriorityNode(
            list_paths=[[new_state_initial]],
            list_primitives=[self.motion_primitive_initial],
            depth_tree=0,
            priority=0,
        )
        initial_visualization(
            self.scenario,
            self.state_initial,
            self.shape_ego,
            self.planningProblem,
            self.config_plot,
            self.path_fig,
        )

        # add current node (i.e., current path and primitives) to the frontier
        f_initial = self.evaluation_function(node_initial)
        self.frontier.insert(item=node_initial, priority=f_initial)

        if self.config_plot.DO_PLOT:
            dict_status_nodes = update_visualization(
                primitive=node_initial.list_paths[-1],
                status=MotionPrimitiveStatus.IN_FRONTIER,
                dict_node_status=dict_status_nodes,
                path_fig=self.path_fig,
                config=self.config_plot,
                count=len(list_status_nodes),
            )
            list_status_nodes.append(copy.copy(dict_status_nodes))

        start_time = time.perf_counter()
        while not self.frontier.empty():
            if SMP.batch_processing.timeout_config.use_sequential_processing:
                if (
                    time.perf_counter() - start_time
                ) >= SMP.batch_processing.timeout_config.timeout:
                    raise Exception("Time Out")
            # pop the last node
            node_current = self.frontier.pop()

            # skip re-expanding a state we've (approximately) already
            # expanded via a different primitive sequence
            key_current = self._state_key(node_current.list_paths[-1][-1])
            if key_current in self._closed_states:
                continue
            self._closed_states.add(key_current)

            if self.config_plot.DO_PLOT:
                dict_status_nodes = update_visualization(
                    primitive=node_current.list_paths[-1],
                    status=MotionPrimitiveStatus.CURRENTLY_EXPLORED,
                    dict_node_status=dict_status_nodes,
                    path_fig=self.path_fig,
                    config=self.config_plot,
                    count=len(list_status_nodes),
                )
                list_status_nodes.append(copy.copy(dict_status_nodes))

            # goal test
            if self.reached_goal(node_current.list_paths[-1]):
                path_solution = self.remove_states_behind_goal(node_current.list_paths)
                list_status_nodes = self.plot_solution(
                    path_solution=path_solution,
                    node_status=dict_status_nodes,
                    list_states_nodes=list_status_nodes,
                )
                # return solution
                return path_solution, node_current.list_primitives, list_status_nodes

            # check all possible successor primitives(i.e., actions) for current node
            for primitive_successor in node_current.get_successors():

                # translate/rotate motion primitive to current position
                list_primitives_current = copy.copy(node_current.list_primitives)
                path_translated = self.translate_primitive_to_current_state(
                    primitive_successor, node_current.list_paths[-1]
                )
                # check for collision, if is not collision free it is skipped
                if not self.is_collision_free(path_translated):
                    if self.config_plot.DO_PLOT:
                        list_status_nodes, dict_status_nodes = (
                            self.plot_colliding_primitives(
                                current_node=node_current,
                                path_translated=path_translated,
                                node_status=dict_status_nodes,
                                list_states_nodes=list_status_nodes,
                            )
                        )
                    continue

                # skip children leading to an already-expanded state too -
                # saves both frontier bloat and a wasted heuristic evaluation
                key_child = self._state_key(path_translated[-1])
                if key_child in self._closed_states:
                    continue

                list_primitives_current.append(primitive_successor)

                path_new = node_current.list_paths + [
                    [node_current.list_paths[-1][-1]] + path_translated
                ]
                node_child = PriorityNode(
                    list_paths=path_new,
                    list_primitives=list_primitives_current,
                    depth_tree=node_current.depth_tree + 1,
                    priority=node_current.priority,
                )
                f_child = self.evaluation_function(node_current=node_child)

                # insert the child to the frontier:
                if self.config_plot.DO_PLOT:
                    dict_status_nodes = update_visualization(
                        primitive=node_child.list_paths[-1],
                        status=MotionPrimitiveStatus.IN_FRONTIER,
                        dict_node_status=dict_status_nodes,
                        path_fig=self.path_fig,
                        config=self.config_plot,
                        count=len(list_status_nodes),
                    )
                    list_status_nodes.append(copy.copy(dict_status_nodes))
                self.frontier.insert(item=node_child, priority=f_child)

            if self.config_plot.DO_PLOT:
                dict_status_nodes = update_visualization(
                    primitive=node_current.list_paths[-1],
                    status=MotionPrimitiveStatus.EXPLORED,
                    dict_node_status=dict_status_nodes,
                    path_fig=self.path_fig,
                    config=self.config_plot,
                    count=len(list_status_nodes),
                )
                list_status_nodes.append(copy.copy(dict_status_nodes))

        return None, None, list_status_nodes

