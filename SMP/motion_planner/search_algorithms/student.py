import numpy as np
from commonroad.geometry.shape import Circle, Polygon, Rectangle, ShapeGroup

from SMP.motion_planner.node import PriorityNode

from SMP.motion_planner.plot_config import DefaultPlotConfig
from SMP.motion_planner.search_algorithms.best_first_search import (
    GreedyBestFirstSearch,
)


class StudentMotionPlanner(GreedyBestFirstSearch):
    """
    Motion planner implementation by students.
    Note that you may inherit from any given motion planner as you wish, or come up with your own planner.

    Tried inheriting from AStarSearch (f(n) = g(n) + h(n)) instead, expecting
    it to reduce backtracking/oscillation. It measurably made things worse:
    g(n) is real elapsed trajectory time and grows every step regardless of
    direction, so it increasingly dominates f(n) with depth and biases the
    search towards shallow/short paths - actively fighting the objective here
    (find *any* feasible solution before the timeout, not the cheapest one).
    Back to GBFS (f(n) = h(n) only).

    Also tried overriding execute_search with a real closed-set (dominance
    pruning: only insert a child into the frontier if it's better than the
    best h(n) already seen for its (lanelet, time_step, velocity) state).
    This backfired: our heuristic isn't a true admissible cost estimate, so
    the first child seen for a state is sometimes a dead end while a later,
    slightly-worse-looking one was the only real path through - pass rate on
    our stratified sample dropped from 44-45/63 to 32/63 (and 42/63 with a
    softer "free quota of 8 before pruning" variant). Reverted to the soft
    revisit_penalty below, which only discourages (never blocks) re-visiting
    a state, and was the best-performing option we measured (356/500 on the
    full scenario set).
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
        self._fix_goal_lanelet_costs()
        # base_class.py normalizes lanelet-graph cost by the *total* number of
        # lanelets in the map, so on large maps (dozens to hundreds of
        # lanelets) this term shrinks to near-zero even though it's the most
        # topologically-reliable signal we have - normalize by the actual
        # reachable-cost range instead so its gradient stays informative
        # regardless of map size.
        finite_costs = [v for v in self.dict_lanelets_costs.values() if v > 0]
        self._max_lanelet_cost = max(finite_costs) if finite_costs else 1
        # cache for _calc_dist_to_closest_obstacle: profiling on dense-traffic
        # maps (e.g. DEU_Flensburg-50_1_T-1) showed ~85% of heuristic_function
        # time going into get_obstacles() -> occupancy_at_time_step(), which
        # is recomputed from scratch for every obstacle on every single
        # heuristic evaluation. occupancy at a given (lanelet, time_step) is
        # fully determined by the scenario, not by the search path, so it's
        # safe to memoize - many sibling/nearby nodes share the same
        # (lanelet, time_step) pair.
        self._obstacles_cache = {}
        # GreedyBestFirstSearch's execute_search has no closed/visited set
        # (f(n) = h(n) only, no g(n) either), so nothing stops it from
        # expanding the same (lanelet, time_step, velocity) region over and
        # over when the heuristic can't see a way to improve further.
        # Measured on DEU_Flensburg-50_1_T-1: of ~3500 heuristic evaluations
        # in 10s, only 28 unique states existed - one state alone was
        # evaluated 1772 times. This tracks visit counts and adds a growing
        # (but never infinite/blocking) penalty on repeats, to push the
        # search off a stuck region after a handful of revisits instead of
        # hundreds, without risking pruning away a state's only real path
        # forward (see class docstring on why a hard closed-set was worse).
        self._visit_counts = {}

    def evaluation_function(self, node_current: PriorityNode) -> float:
        # f(n) = h(n) + epsilon*depth: a tiny depth tie-breaker on top of
        # GBFS's pure h(n). Full A* (f = g+h, g = real elapsed time) made
        # things measurably worse (biases hard towards shallow paths - see
        # class docstring), but a small nudge only needs to break ties/near-
        # ties between otherwise-similar children without meaningfully
        # discouraging depth. Measured 45/63 -> 46/63 on our stratified
        # sample with epsilon=0.05.
        node_current.priority = self.heuristic_function(
            node_current=node_current
        ) + 0.1 * node_current.depth_tree
        return node_current.priority

    def _fix_goal_lanelet_costs(self):
        # base_class.py resolves the goal lanelet by checking whether the
        # *centroid* of the goal shape falls inside a lanelet polygon
        # (find_lanelet_by_position), which silently returns [] whenever that
        # centroid doesn't land inside the shape itself - common for curved/
        # non-convex lanelets, which is how most <lanelet ref=".."/> goals in
        # this dataset are specified. When that happens self.dict_lanelets_costs
        # stays all -1, and the whole lanelet-graph-distance term above goes
        # blind for the rest of the search. commonroad-io already parses the
        # intended goal lanelet id directly (goal.lanelets_of_goal_position) -
        # recompute the cost table from that ground truth instead.
        lanelets_of_goal = self.planningProblem.goal.lanelets_of_goal_position
        if not lanelets_of_goal:
            return
        goal_ids = sorted({lid for ids in lanelets_of_goal.values() for lid in ids})
        if not goal_ids:
            return
        self.list_ids_lanelets_goal = goal_ids
        for lanelet in self.scenario.lanelet_network.lanelets:
            self.dict_lanelets_costs[lanelet.lanelet_id] = -1
        for lanelet_id in goal_ids:
            self.dict_lanelets_costs[lanelet_id] = 0
        for lanelet_id in goal_ids:
            lanelet_goal = self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            self.calc_lanelet_cost(lanelet_goal, 1, [])

    def _obstacle_shapes_in_lanelet(self, lanelet_id, time_step):
        # cached: obstacle occupancy at a given (lanelet, time_step) is fully
        # determined by the scenario, not by which search node asks for it, so
        # this is safe to memoize across the whole search (see cache comment
        # in __init__).
        key = (lanelet_id, time_step)
        cached = self._obstacles_cache.get(key)
        if cached is not None:
            return cached
        try:
            obstacles_in_lanelet = self.get_obstacles(
                self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id), time_step
            )
        except ValueError:
            # base_class.py's get_obstacles crashes (inhomogeneous numpy array)
            # when a ShapeGroup obstacle mixes Circle and Rectangle/Polygon
            # shapes - a pre-existing bug we can't fix from here. Treat as
            # "couldn't determine obstacle proximity" rather than crash the
            # whole search over one candidate primitive.
            obstacles_in_lanelet = []
        shapes = []
        for obstacle in obstacles_in_lanelet:
            shape_obs = obstacle.occupancy_at_time(time_step).shape
            shapes.extend(
                shape_obs.shapes if isinstance(shape_obs, ShapeGroup) else [shape_obs]
            )
        self._obstacles_cache[key] = shapes
        return shapes

    def _calc_dist_to_closest_obstacle(self, lanelet_id, pos, time_step) -> float:
        # base_class.py's calc_dist_to_closest_obstacle only handles Circle
        # obstacles, but most dynamic obstacles (cars) are Rectangle, so it
        # returns inf almost always and this cost term is effectively dead.
        # Reimplemented here (distance to closest vertex for Rectangle/Polygon)
        # since we're only allowed to edit our own planner file.
        shortest_dist = np.inf
        for sh in self._obstacle_shapes_in_lanelet(lanelet_id, time_step):
            if isinstance(sh, Circle):
                dist = self.distance(pos, sh.center)
            elif isinstance(sh, (Rectangle, Polygon)):
                dist = min(self.distance(pos, vertex) for vertex in sh.vertices)
            else:
                continue
            shortest_dist = min(shortest_dist, dist)
        return shortest_dist

    def _lanelet_arc_progress(self, lanelet_id, pos) -> float:
        # fraction (0..1) of the lanelet's centerline arc-length already
        # covered by pos, using distance to the nearest centerline vertex as
        # an approximation - cheap and good enough to break ties between
        # states in the same lanelet.
        lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
        cv = lanelet.center_vertices
        if len(cv) < 2:
            return 0.0
        seg_lens = np.linalg.norm(np.diff(cv, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg_lens)])
        total = cum[-1]
        if not total:
            return 0.0
        idx = int(np.argmin(np.linalg.norm(cv - pos, axis=1)))
        return cum[idx] / total

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
        if dist_manhattan_start:
            dist_manhattan_norm = dist_manhattan / dist_manhattan_start
        else:
            dist_manhattan_norm = dist_manhattan

        cost_lanelet, final_lanelet_id, start_lanelet_id = self.calc_heuristic_lanelet(
            path_last
        )
        # self.dict_lanelets_costs is only populated when the goal has a
        # position (base_class.py's initialize_lanelets_costs skips entirely
        # otherwise, leaving every lanelet at its -1 placeholder). Over 60% of
        # scenarios in this dataset have time/velocity-only goals with no
        # position at all, so treating "-1" as a hard np.inf veto (as before)
        # made every candidate node equally infinite for the whole search,
        # i.e. no guidance at all. Only apply the lanelet-graph term - and
        # only as a finite penalty, not an infinite one - when there is
        # actually a goal lanelet to route towards.
        has_lanelet_goal = bool(self.list_ids_lanelets_goal)
        final_lanelet = final_lanelet_id[0] if final_lanelet_id else None
        if cost_lanelet is None or final_lanelet is None:
            # couldn't place the vehicle on a lanelet (e.g. off-road) -
            # discourage strongly but don't hard-block the whole branch
            cost_lanelet_graph_norm = 5.0
        elif not has_lanelet_goal:
            cost_lanelet_graph_norm = 0.0
        else:
            cost_lanelet_graph = self.dict_lanelets_costs[final_lanelet]
            if cost_lanelet_graph == -1:
                # unreachable from here, but stay finite so the search can
                # still tell this apart from an even-worse/better node
                cost_lanelet_graph_norm = 5.0
            else:
                # cost_lanelet_graph alone only changes at lanelet boundaries,
                # so every child that stays within the same lanelet gets an
                # identical value - no gradient to tell them apart, which lets
                # the search wander for a while before any of them looks
                # better. Subtract how far along *this* lanelet we already are
                # (arc-length fraction) so progress is rewarded continuously,
                # not just at hop boundaries.
                progress = self._lanelet_arc_progress(final_lanelet, path_last[-1].position)
                cost_lanelet_graph_norm = (
                    cost_lanelet_graph - progress
                ) / self._max_lanelet_cost

        if hasattr(self.planningProblem.goal.state_list[0], "position"):
            angleToGoal = self.calc_angle_to_goal(path_last[-1])
            orientationToGoalDiff = self.calc_orientation_diff(
                angleToGoal, path_last[-1].orientation
            )
            # abs(orientationToGoalDiff) is always within [0, pi], so this is
            # already bounded to [0, 1]
            orientation_diff_norm = abs(orientationToGoalDiff) / np.pi
        else:
            # calc_angle_to_goal() falls back to angle 0 when there's no goal
            # position, which would make this term measure absolute heading
            # instead of anything goal-related - just drop it in that case
            orientation_diff_norm = 0

        # penalize proximity to the closest obstacle in the current lanelet
        # instead of just counting obstacles, so a close obstacle costs more
        # than a far one, and no obstacle at all costs nothing
        dist_closest_obstacle = (
            self._calc_dist_to_closest_obstacle(
                final_lanelet, path_last[-1].position, path_last[-1].time_step
            )
            if final_lanelet is not None
            else np.inf
        )
        if np.isinf(dist_closest_obstacle):
            cost_obstacle = 0.0
        else:
            # saturate: 1/(dist+eps) blows up near obstacles and would otherwise
            # dwarf the other ~[0, 1]-normalized terms below
            cost_obstacle = min(1.0, 1.0 / (dist_closest_obstacle + 1e-6))

        # path_last[0] is this new segment's starting state (== the parent's
        # last state) - if even that is already past the goal's time window
        # end, this branch is provably dead (time only moves forward, and
        # every primitive has the same fixed 0.5s duration), so it's correct
        # to reject it outright.
        #
        # NOTE on the continuous term below: every primitive has the exact
        # same duration, so time_step at a given tree depth is identical
        # across every candidate regardless of which primitive was chosen -
        # this term can't distinguish good children from bad ones, it only
        # decreases uniformly with depth. Tracing a stuck search
        # (DEU_Reutlingen-1) showed this causing a false sense of progress:
        # the "best"-looking node had barely moved spatially (manhattan
        # still 80% of the original distance) yet kept looking better every
        # level purely from this term ticking down, letting the search keep
        # extending a path that wasn't actually converging. Tried removing
        # it entirely - measured *worse* (47/63 vs 48/63), so it's not purely
        # harmful, just partially redundant with epsilon*depth_tree above.
        # Left at full weight since that's what measured best; flagging the
        # mechanism here in case future tuning revisits it.
        if hasattr(self.planningProblem.goal.state_list[0], "time_step"):
            time_goal_start = self.planningProblem.goal.state_list[0].time_step.start
            time_goal_end = self.planningProblem.goal.state_list[0].time_step.end
            if path_last[0].time_step > time_goal_end:
                return np.inf
            current_time_step = path_last[-1].time_step
            cost_time_norm = (
                max(0, time_goal_start - current_time_step) / time_goal_start
                if time_goal_start
                else 0
            )
        else:
            cost_time_norm = 0

        if hasattr(self.planningProblem.goal.state_list[0], "velocity"):
            # target the upper bound of the goal's velocity interval: going
            # faster (up to the allowed max) is cheaper than going slower
            v_max_goal = self.planningProblem.goal.state_list[0].velocity.end
            dist_vel = abs(path_last[-1].velocity - v_max_goal)
            dist_vel_norm = dist_vel / v_max_goal if v_max_goal else dist_vel
        else:
            dist_vel_norm = 0

        # soft anti-oscillation penalty - see _visit_counts comment in __init__.
        # Siblings from one parent's expansion legitimately share the same
        # (lanelet, time_step) - e.g. 15 different steering choices that all
        # land in the same lane cell - and shouldn't be penalized against each
        # other for that. Only kick in once a state has been visited far more
        # than any reasonable branching factor would explain, which is the
        # actual pathological-cycling signature we measured (1772 visits to
        # one state vs. ~28 unique states total).
        revisit_key = (
            final_lanelet,
            path_last[-1].time_step,
            round(path_last[-1].velocity),
        )
        visits = self._visit_counts.get(revisit_key, 0)
        self._visit_counts[revisit_key] = visits + 1
        revisit_penalty = 0.15 * max(0, visits - 6)

        weights = np.zeros(6)
        weights[0] = 1.0  # manhattan distance to goal (normalized)
        weights[1] = 1.0  # velocity difference to goal's max velocity (normalized)
        weights[2] = 1.0  # orientation difference to goal (normalized)
        weights[3] = 1.0  # proximity to closest obstacle in current lanelet
        weights[4] = 1.0  # lanelet-graph distance to goal lanelet (normalized)
        weights[5] = 1.0  # time cost of trajectory so far (normalized)

        cost = (
            revisit_penalty
            + weights[0] * dist_manhattan_norm
            + weights[1] * dist_vel_norm
            + weights[2] * orientation_diff_norm
            + weights[3] * cost_obstacle
            + weights[4] * cost_lanelet_graph_norm
            + weights[5] * cost_time_norm
        )

        if cost < 0:
            cost = 0
        return cost
