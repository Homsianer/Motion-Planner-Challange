import heapq
import itertools
import math
import os
import time

import numpy as np
from commonroad.geometry.shape import Circle, Polygon, Rectangle, ShapeGroup

import SMP.batch_processing.timeout_config
from SMP.maneuver_automaton.maneuver_automaton import ManeuverAutomaton
from SMP.motion_planner.node import PriorityNode
from SMP.motion_planner.plot_config import DefaultPlotConfig
from SMP.motion_planner.queue import PriorityQueue
from SMP.motion_planner.search_algorithms.best_first_search import (
    GreedyBestFirstSearch,
)

# --- motion primitive sets & time budget -------------------------------------
# Same vehicle (BMW 320i), different velocity resolution. The default set of the
# batch config (4 m/s steps) can only hold 0/4/8/12/16/20 m/s, which is too
# coarse to arrive inside a goal time window of 1-2 time steps; the 2 m/s and
# 2.22 m/s sets solve partly different scenarios, so both are tried in turn.
AUTOMATON_V2_0 = (
    "V_0.0_20.0_Vstep_2.0_SA_-1.066_1.066_SAstep_0.18_T_0.5_Model_BMW_320i.automaton"
)
AUTOMATON_V2_22 = (
    "V_0.0_20.0_Vstep_2.22_SA_-1.066_1.066_SAstep_0.18_T_0.5_Model_BMW_320i.automaton"
)
# Route search phases for goals with a position: (primitive set, deadline in
# seconds since the planner started). The batch processor kills the process
# after 30 s; the remaining time goes to the team's GBFS as a fallback.
ROUTE_PHASES = ((AUTOMATON_V2_0, 9.5), (AUTOMATON_V2_22, 19.0))
# primitive set for a second GBFS attempt when the first one ran out of nodes
GBFS_RETRY_AUTOMATON = AUTOMATON_V2_0

# --- route search: all costs are in "time steps" (1 primitive = 5 steps) -----
W_TIMING = 2.0  # per m/s the car is too slow to reach the goal zone in time
W_LANE_CHANGE = 4.0  # per lane change still needed to reach the goal lanelet
W_LATERAL = 1.0  # per m of lateral offset from the lane center
W_HEADING = 6.0  # per rad of heading error w.r.t. the lane direction
W_WRONG_WAY = 8.0  # driving in the oncoming lane (allowed, but only if needed)
W_GOAL_VELOCITY = 2.0  # per m/s outside the goal's velocity interval (near goal)
W_GOAL_ORIENTATION = 15.0  # per rad outside the goal's orientation interval
W_GOAL_LATERAL = 3.0  # per m lateral offset from a rectangle goal's center
OFF_ROUTE_PENALTY = 40.0  # position not on any lanelet at all
LANE_CHANGE_COST_M = 20.0  # route choice: one lane change "costs" 20 m
NEAR_GOAL_M = 25.0  # goal velocity/orientation terms apply below this distance
PRUNE_MARGIN_M = 10.0  # slack for the reachability pruning (geometry is approx.)
DEDUP_CELL_M = 1.0  # duplicate detection grid for positions
DEDUP_CELL_RAD = 0.1  # duplicate detection grid for orientations
# Kinematic bounds of the primitive sets above: velocity changes by at most
# 4.44 m/s per 0.5 s primitive (2.22 set) and never exceeds 20 m/s.
A_MAX = 9.0
V_MAX = 20.0

REPO_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)
DIAG_DIR = os.path.join(REPO_ROOT, "outputs", "logs", "student_diag")

DEAD_END = "dead_end"


def _angle_diff(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class _LaneletGeometry:
    """Centerline of one lanelet, prepared for fast point projection."""

    def __init__(self, center_vertices):
        c = np.asarray(center_vertices, dtype=float)
        if len(c) < 2:
            c = np.vstack([c, c + 1e-3])
        self.start = c[:-1]
        self.seg = c[1:] - c[:-1]
        self.seglen = np.hypot(self.seg[:, 0], self.seg[:, 1])
        self.seglen2 = np.maximum(self.seglen**2, 1e-12)
        self.cum = np.concatenate([[0.0], np.cumsum(self.seglen)])
        self.length = float(self.cum[-1])
        self.heading = np.arctan2(self.seg[:, 1], self.seg[:, 0])

    def project(self, p):
        """Returns (arc length s, signed lateral offset, lane heading) of p."""
        d = p - self.start
        t = np.clip((d * self.seg).sum(axis=1) / self.seglen2, 0.0, 1.0)
        diff = d - self.seg * t[:, None]
        dist2 = (diff**2).sum(axis=1)
        i = int(np.argmin(dist2))
        s = self.cum[i] + t[i] * self.seglen[i]
        cross = self.seg[i, 0] * diff[i, 1] - self.seg[i, 1] * diff[i, 0]
        lateral = math.copysign(math.sqrt(dist2[i]), cross)
        return float(s), lateral, float(self.heading[i])


class StudentMotionPlanner(GreedyBestFirstSearch):
    """
    Two search strategies, chosen by the type of the goal region:

    1. Goals with a position (lanelet or rectangle, 196 of 500 scenarios): a
       route-based greedy best-first search (_route_search). Nearly all of
       these goals only count inside a narrow time window (161 lanelet goals
       at time step 99-100), so the search tracks the driving distance along
       the lanelet route to the goal zone, keeps the car fast enough to get
       there in time, prunes states that provably cannot hit the window any
       more, and drops near-duplicate states. It runs with two finer motion
       primitive sets in turn (ROUTE_PHASES), then falls back to 2.

    2. Goals with only a time step (304 scenarios), and fallback for 1.: the
       team's GBFS heuristic (heuristic_function below) on the inherited
       search loop, which solved 289/304 time-only goals. If it runs out of
       nodes, it is retried with a finer primitive set.

    Notes from earlier experiments with the GBFS heuristic:
    - Full A* (f = g + h) was measurably worse: g grows with depth and biases
      the search towards short paths, while any feasible path is enough here.
    - A hard closed set keyed on (lanelet, time_step, velocity) dropped the
      pass rate on a 63-scenario sample from 44-45 to 32: that key merges
      states that are far apart inside one lanelet. The route search dedups on
      a 1 m / 0.1 rad grid instead, which only merges near-identical states.
    """

    def __init__(
        self, scenario, planningProblem, automata, plot_config=DefaultPlotConfig
    ):
        self._t_start = time.perf_counter()
        initial_velocity_raw = planningProblem.initial_state.velocity
        super().__init__(
            scenario=scenario,
            planningProblem=planningProblem,
            automaton=automata,
            plot_config=plot_config,
        )
        self._fix_goal_lanelet_costs()
        # base_class.py normalizes lanelet-graph cost by the *total* number of
        # lanelets in the map, so on large maps this term shrinks to near-zero
        # - normalize by the actual reachable-cost range instead.
        finite_costs = [v for v in self.dict_lanelets_costs.values() if v > 0]
        self._max_lanelet_cost = max(finite_costs) if finite_costs else 1
        # obstacle occupancy per (lanelet, time_step) is fixed by the scenario,
        # so it is memoized across the whole search (was ~85% of heuristic
        # time on dense maps like DEU_Flensburg-50_1_T-1).
        self._obstacles_cache = {}
        # soft anti-oscillation penalty for the GBFS heuristic: on
        # DEU_Flensburg-50_1_T-1 one (lanelet, time_step, velocity) state was
        # evaluated 1772 times out of ~3500 evaluations.
        self._visit_counts = {}

        # (automaton, initial primitive, snapped initial velocity) per primitive
        # set, loaded on first use; None is the batch config's default set.
        # Every set snaps the initial velocity onto its own grid (and writes it
        # into the shared initial state), so the true value is kept to re-snap
        # from and the matching snapped value is restored on every switch.
        self._v_initial_raw = initial_velocity_raw
        self._automata = {
            None: (self.automaton, self.motion_primitive_initial, self.state_initial.velocity)
        }
        self._diag = {"scenario": str(self.scenario.scenario_id), "phases": []}
        self._diag_last_write = 0.0

        goal_state = self.planningProblem.goal.state_list[0]
        self._t0 = int(self.time_desired.start)
        self._t1 = int(min(self.time_desired.end, 10**6))
        self._goal_velocity = getattr(goal_state, "velocity", None)
        self._goal_orientation = getattr(goal_state, "orientation", None)
        self._goal_lateral = {}
        self._route = None
        if hasattr(goal_state, "position"):
            try:
                self._route = self._build_route(goal_state)
            except Exception as err:  # route is an optimization, never fatal
                self._diag["route_error"] = repr(err)
                self._route = None
            if self._route is not None:
                # without a route from the start there is nothing to follow
                root_info = self._route_info(self._root_state())
                self._diag["root_route_info"] = root_info
                if root_info is None or root_info is DEAD_END:
                    self._route = None

    # ------------------------------------------------------------------ search

    def execute_search(self):
        # goals with a position: route search with each primitive set in turn;
        # a phase that runs out of nodes early hands its time to the next one
        if self._route is not None:
            for automaton_file, deadline in ROUTE_PHASES:
                if not self._use_automaton(automaton_file):
                    continue
                solution = self._route_search(self._t_start + deadline)
                if solution is not None:
                    return solution[0], solution[1], []

        # time-only goals, or fallback for position goals: the team's GBFS
        # with the default primitive set, retried with a finer set if it ran
        # out of nodes (the inherited loop only stops on exhaustion/timeout)
        self._use_automaton(None)
        self._diag_phase("gbfs")
        path, primitives, status = super().execute_search()
        if path is None and self._use_automaton(GBFS_RETRY_AUTOMATON):
            self._diag_phase("gbfs_retry")
            self.frontier = PriorityQueue()
            self._visit_counts = {}
            path, primitives, status = super().execute_search()
        self._diag_phase("gbfs_failed" if path is None else "gbfs_solved")
        return path, primitives, status

    def _use_automaton(self, automaton_file) -> bool:
        """Switches the primitive set (None: default); False if it can't be loaded."""
        if automaton_file not in self._automata:
            try:
                automaton = ManeuverAutomaton.load_automaton(automaton_file)
                self.state_initial.velocity = self._v_initial_raw
                initial = automaton.create_initial_motion_primitive(self.planningProblem)
            except Exception as err:
                self._diag["automaton_error"] = repr(err)
                return False
            self._automata[automaton_file] = (
                automaton,
                initial,
                self.state_initial.velocity,
            )
        automaton, initial, velocity = self._automata[automaton_file]
        self.automaton = automaton
        self.motion_primitive_initial = initial
        self.state_initial.velocity = velocity
        return True

    def _root_state(self):
        # the planning problem's initial state refers to the vehicle center,
        # the primitives to the rear axle (same shift as in best_first_search)
        o = self.state_initial.orientation
        return self.state_initial.translate_rotate(
            -np.array([self.rear_ax_dist * np.cos(o), self.rear_ax_dist * np.sin(o)]),
            0,
        )

    def _route_search(self, deadline):
        """
        Greedy best-first search on the route heuristic (_route_heuristic).
        Differences to the inherited loop: children are pruned *before* the
        collision check, pruned children are never queued, near-duplicate
        states are dropped, and the goal test happens when a child is created.
        Returns (paths, primitives) or None.
        """
        stats = self._diag_phase("route")
        stats["automaton"] = next(
            (name for name, entry in self._automata.items() if entry[0] is self.automaton),
            None,
        )
        root = PriorityNode(
            list_paths=[[self._root_state()]],
            list_primitives=[self.motion_primitive_initial],
            depth_tree=0,
            priority=0.0,
        )
        if self.reached_goal(root.list_paths[-1]):
            return self.remove_states_behind_goal(root.list_paths), root.list_primitives

        counter = itertools.count()
        frontier = [(0.0, next(counter), root)]
        seen = set()
        while frontier:
            if time.perf_counter() > deadline:
                stats["end"] = "budget"
                self._diag_write(force=True)
                return None
            self._check_sequential_timeout()
            _, _, node = heapq.heappop(frontier)
            path_current = node.list_paths[-1]
            stats["expanded"] += 1
            stats["max_depth"] = max(stats["max_depth"], node.depth_tree)
            # nothing after the goal window can reach the goal any more
            if path_current[-1].time_step >= self._t1:
                continue

            for primitive in node.get_successors():
                path_translated = self.translate_primitive_to_current_state(
                    primitive, path_current
                )
                state_last = path_translated[-1]
                key = (
                    state_last.time_step,
                    int(round(state_last.position[0] / DEDUP_CELL_M)),
                    int(round(state_last.position[1] / DEDUP_CELL_M)),
                    int(round(state_last.orientation / DEDUP_CELL_RAD)),
                    round(primitive.state_final.velocity, 1),
                    round(primitive.state_final.steering_angle, 2),
                )
                if key in seen:
                    stats["dup"] += 1
                    continue
                h = self._route_heuristic(state_last, stats)
                if h is None:
                    continue
                if not self.is_collision_free(path_translated):
                    stats["collision"] += 1
                    continue
                seen.add(key)
                child = PriorityNode(
                    list_paths=node.list_paths + [[path_current[-1]] + path_translated],
                    list_primitives=node.list_primitives + [primitive],
                    depth_tree=node.depth_tree + 1,
                    priority=h,
                )
                if self.reached_goal(child.list_paths[-1]):
                    stats["end"] = "solved"
                    stats["solution_depth"] = child.depth_tree
                    self._diag_write(force=True)
                    return (
                        self.remove_states_behind_goal(child.list_paths),
                        child.list_primitives,
                    )
                heapq.heappush(frontier, (h, next(counter), child))
            self._diag_write()

        stats["end"] = "exhausted"
        self._diag_write(force=True)
        return None

    def _check_sequential_timeout(self):
        # mirrors the inherited loop: in sequential mode the planner has to
        # stop itself, in parallel mode the batch processor kills it
        if SMP.batch_processing.timeout_config.use_sequential_processing:
            if (
                time.perf_counter() - self._t_start
                >= SMP.batch_processing.timeout_config.timeout
            ):
                raise Exception("Time Out")

    # ------------------------------------------------------------------- route

    def _build_route(self, goal_state):
        """
        Longitudinal driving distance from every lanelet to the goal zone.

        The goal zone is an arc-length interval [s_in, s_out] on each goal
        lanelet: the whole lanelet for lanelet goals, the rectangle's extent
        around its center for rectangle goals. A backwards Dijkstra over the
        lanelet graph then gives, per lanelet L, the distance D(L) from the
        start of L to the zone entry. Successor edges cost the lanelet length,
        lane changes into a same-direction neighbour cost no distance (they
        are counted separately), so a position at arc length s on L is
        d_in = D(L) - s before the zone entry and d_in + zone length before
        the zone exit.
        """
        network = self.scenario.lanelet_network
        geometry = {
            l.lanelet_id: _LaneletGeometry(l.center_vertices) for l in network.lanelets
        }

        zones = {}
        lanelets_of_goal = self.planningProblem.goal.lanelets_of_goal_position
        if lanelets_of_goal:
            for ids in lanelets_of_goal.values():
                for lanelet_id in ids:
                    if lanelet_id in geometry:
                        zones[lanelet_id] = (0.0, geometry[lanelet_id].length)
        else:
            shape = goal_state.position
            center = np.asarray(shape.center, dtype=float)
            half = shape.length / 2.0 if isinstance(shape, Rectangle) else 0.5
            for lanelet_id in network.find_lanelet_by_position([center])[0]:
                s, lateral, _ = geometry[lanelet_id].project(center)
                zones[lanelet_id] = (s - half, s + half)
                self._goal_lateral[lanelet_id] = lateral
        if not zones:
            return None

        predecessors = {lanelet_id: set() for lanelet_id in geometry}
        lane_change_from = {lanelet_id: set() for lanelet_id in geometry}
        opposite = {lanelet_id: set() for lanelet_id in geometry}
        for lanelet in network.lanelets:
            lid = lanelet.lanelet_id
            for succ in lanelet.successor or []:
                if succ in geometry:
                    predecessors[succ].add(lid)
            for pred in lanelet.predecessor or []:
                if pred in geometry:
                    predecessors[lid].add(pred)
            for adj, same_dir in (
                (lanelet.adj_left, lanelet.adj_left_same_direction),
                (lanelet.adj_right, lanelet.adj_right_same_direction),
            ):
                if adj is None or adj not in geometry:
                    continue
                if same_dir:
                    lane_change_from[adj].add(lid)
                else:
                    opposite[lid].add(adj)
                    opposite[adj].add(lid)

        # (priority, lanelet, D, lane changes, zone length)
        heap = [(s_in, lid, s_in, 0, s_out - s_in) for lid, (s_in, s_out) in zones.items()]
        heapq.heapify(heap)
        best = {}
        while heap:
            prio, lid, dist, lane_changes, zone_len = heapq.heappop(heap)
            if lid in best:
                continue
            best[lid] = (dist, lane_changes, zone_len)
            for pred in predecessors.get(lid, ()):
                if pred not in best:
                    length = geometry[pred].length
                    heapq.heappush(
                        heap, (prio + length, pred, dist + length, lane_changes, zone_len)
                    )
            for neighbour in lane_change_from.get(lid, ()):
                if neighbour not in best:
                    heapq.heappush(
                        heap,
                        (prio + LANE_CHANGE_COST_M, neighbour, dist, lane_changes + 1, zone_len),
                    )
        return {"geometry": geometry, "best": best, "zones": zones, "opposite": opposite}

    def _route_info(self, state):
        """
        (d_in, d_out, lane_changes, lateral_error, heading_error, wrong_way) of
        the vehicle center; DEAD_END if it is only on lanelets that cannot lead
        to the goal any more (wrong turn or behind the goal); None if it is not
        on any lanelet at all.
        """
        orientation = state.orientation
        center = np.asarray(state.position, dtype=float) + self.rear_ax_dist * np.array(
            [math.cos(orientation), math.sin(orientation)]
        )
        geometry = self._route["geometry"]
        candidates = self.scenario.lanelet_network.find_lanelet_by_position([center])[0]
        # A car driving against a lanelet is in the oncoming lane (e.g. while
        # overtaking) - the collision checker allows that as long as nothing is
        # hit, so it is evaluated on the neighbouring lanelet of its own
        # direction instead of being treated as lost.
        targets = []
        for lanelet_id in candidates:
            _, _, lane_heading = geometry[lanelet_id].project(center)
            if abs(_angle_diff(orientation, lane_heading)) <= math.pi / 2:
                targets.append((lanelet_id, False))
            else:
                targets.extend((lid, True) for lid in self._route["opposite"][lanelet_id])
        best = None
        for lanelet_id, wrong_way in targets:
            route_entry = self._route["best"].get(lanelet_id)
            if route_entry is None:
                continue
            s, lateral, lane_heading = geometry[lanelet_id].project(center)
            heading_error = abs(_angle_diff(orientation, lane_heading))
            if heading_error > math.pi / 2:
                continue
            dist, lane_changes, zone_len = route_entry
            d_in = dist - s
            score = d_in + LANE_CHANGE_COST_M * (lane_changes + wrong_way) + 10.0 * heading_error
            if best is None or score < best[0]:
                if lanelet_id in self._goal_lateral:
                    lateral -= self._goal_lateral[lanelet_id]
                best = (score, d_in, d_in + zone_len, lane_changes, lateral, heading_error, wrong_way)
        if best is None:
            return DEAD_END if candidates else None
        return best[1:]

    @staticmethod
    def _max_distance(v, tau):
        """Farthest distance within tau seconds (full throttle up to V_MAX)."""
        if tau <= 0:
            return 0.0
        t_acc = max(0.0, (V_MAX - v) / A_MAX)
        if tau <= t_acc:
            return v * tau + 0.5 * A_MAX * tau**2
        return v * t_acc + 0.5 * A_MAX * t_acc**2 + V_MAX * (tau - t_acc)

    @staticmethod
    def _min_distance(v, tau):
        """Shortest distance within tau seconds (full braking, then standing)."""
        if tau <= 0:
            return 0.0
        t_stop = v / A_MAX
        if tau >= t_stop:
            return v**2 / (2 * A_MAX)
        return v * tau - 0.5 * A_MAX * tau**2

    def _route_heuristic(self, state, stats):
        """
        Estimated cost in time steps, or None if the state is pruned.

        h = remaining steps until the window opens      (drives the search deep)
          + W_TIMING * how much slower the car is than the average speed
            needed to reach the goal zone before t1      (don't fall behind)
          + lane change / wrong-way / lane-center / heading terms (stay on route)
          + goal velocity/orientation terms near the goal (rectangle goals)
        """
        t = state.time_step
        v = state.velocity
        remaining_steps = max(0, self._t0 - t)
        info = self._route_info(state)
        if info is None:
            return remaining_steps + OFF_ROUTE_PENALTY
        if info is DEAD_END:
            stats["dead_end"] += 1
            return None
        d_in, d_out, lane_changes, lateral, heading_error, wrong_way = info

        dt = self.scenario.dt
        tau0 = (self._t0 - t) * dt
        tau1 = (self._t1 - t) * dt
        # Pruning: even at full throttle the zone entry is out of reach before
        # the window closes, or even under full braking the car passes the
        # zone exit before the window opens. Both are final - the goal cannot
        # be reached from this state any more.
        if d_in > self._max_distance(v, tau1) + PRUNE_MARGIN_M:
            stats["too_far"] += 1
            return None
        if d_out < self._min_distance(v, tau0) - PRUNE_MARGIN_M:
            stats["overshoot"] += 1
            return None

        # Only being too slow is penalized: arriving early and waiting inside
        # the goal zone is fine (also penalizing speeds that would pass the zone
        # exit before the window opens measured clearly worse, 138 vs 143 of
        # 196 in the offline simulation) - the overshoot pruning above already
        # removes states that cannot stop in time.
        v_low = max(0.0, d_in) / tau1 if tau1 > 0 else 0.0
        speed_error = max(0.0, v_low - v)

        h = (
            remaining_steps
            + W_TIMING * speed_error
            + W_LANE_CHANGE * lane_changes
            + W_WRONG_WAY * wrong_way
            + W_HEADING * heading_error
        )
        h += (W_GOAL_LATERAL if self._goal_lateral else W_LATERAL) * abs(lateral)
        if d_in < NEAR_GOAL_M:
            if self._goal_velocity is not None:
                h += W_GOAL_VELOCITY * max(
                    0.0, self._goal_velocity.start - v, v - self._goal_velocity.end
                )
            if self._goal_orientation is not None:
                mid = 0.5 * (self._goal_orientation.start + self._goal_orientation.end)
                half = 0.5 * (self._goal_orientation.end - self._goal_orientation.start)
                h += W_GOAL_ORIENTATION * max(
                    0.0, abs(_angle_diff(state.orientation, mid)) - half
                )
        return h

    # ------------------------------------------------------------- diagnostics
    # Temporary: one small text file per scenario under outputs/logs (git-
    # ignored) so failed/timed-out runs can be analysed afterwards. Never
    # allowed to break the planner.

    def _diag_phase(self, name, write=False):
        stats = {
            "phase": name,
            "t_start": round(time.perf_counter() - self._t_start, 2),
            "expanded": 0,
            "max_depth": 0,
            "dup": 0,
            "collision": 0,
            "too_far": 0,
            "overshoot": 0,
            "dead_end": 0,
            "end": None,
        }
        self._diag["phases"].append(stats)
        self._diag_write(force=True)
        return stats

    def _diag_write(self, force=False):
        now = time.perf_counter()
        if not force and now - self._diag_last_write < 2.0:
            return
        self._diag_last_write = now
        self._diag["elapsed"] = round(now - self._t_start, 2)
        try:
            os.makedirs(DIAG_DIR, exist_ok=True)
            path = os.path.join(DIAG_DIR, self._diag["scenario"] + ".txt")
            with open(path, "w") as f:
                f.write(repr(self._diag) + "\n")
        except Exception:
            pass

    # ------------------------------------------- GBFS heuristic (team version)

    def evaluation_function(self, node_current: PriorityNode) -> float:
        # f(n) = h(n) + epsilon*depth: a small depth tie-breaker on top of
        # GBFS's pure h(n); measured 45/63 -> 46/63 on the stratified sample.
        node_current.priority = self.heuristic_function(
            node_current=node_current
        ) + 0.1 * node_current.depth_tree
        return node_current.priority

    def _fix_goal_lanelet_costs(self):
        # base_class.py resolves the goal lanelet via the *centroid* of the
        # goal shape, which misses curved lanelets; commonroad-io already
        # parses the intended goal lanelet ids (lanelets_of_goal_position), so
        # the lanelet cost table is recomputed from those.
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
            # for ShapeGroups mixing circles and polygons
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
        # base_class.py's version only handles Circle obstacles, but most
        # obstacles are Rectangles - so it almost always returned inf.
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
        # fraction (0..1) of the lanelet's centerline already covered by pos
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

        # normalized by the distance at the start: "fraction of the trip left"
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
        # Without a goal position every lanelet cost stays -1; treating -1 as
        # an infinite veto made every node equally infinite (no guidance at
        # all for 61% of the scenarios). Only use the lanelet-graph term when
        # there is a goal lanelet, and only as a finite penalty.
        has_lanelet_goal = bool(self.list_ids_lanelets_goal)
        final_lanelet = final_lanelet_id[0] if final_lanelet_id else None
        if cost_lanelet is None or final_lanelet is None:
            cost_lanelet_graph_norm = 5.0  # not on a lanelet (e.g. off-road)
        elif not has_lanelet_goal:
            cost_lanelet_graph_norm = 0.0
        else:
            cost_lanelet_graph = self.dict_lanelets_costs[final_lanelet]
            if cost_lanelet_graph == -1:
                cost_lanelet_graph_norm = 5.0  # goal unreachable from here
            else:
                # subtract the progress along the current lanelet so that the
                # term also has a gradient inside a lanelet
                progress = self._lanelet_arc_progress(final_lanelet, path_last[-1].position)
                cost_lanelet_graph_norm = (
                    cost_lanelet_graph - progress
                ) / self._max_lanelet_cost

        if hasattr(self.planningProblem.goal.state_list[0], "position"):
            angleToGoal = self.calc_angle_to_goal(path_last[-1])
            orientationToGoalDiff = self.calc_orientation_diff(
                angleToGoal, path_last[-1].orientation
            )
            orientation_diff_norm = abs(orientationToGoalDiff) / np.pi
        else:
            # without a goal position calc_angle_to_goal() returns 0, which
            # would just reward an absolute heading
            orientation_diff_norm = 0

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
            # saturated so it cannot dwarf the other ~[0, 1] terms
            cost_obstacle = min(1.0, 1.0 / (dist_closest_obstacle + 1e-6))

        # A segment that already starts after the goal window can never reach
        # the goal (time only moves forward) - reject it outright.
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
            # target the upper bound of the goal's velocity interval
            v_max_goal = self.planningProblem.goal.state_list[0].velocity.end
            dist_vel = abs(path_last[-1].velocity - v_max_goal)
            dist_vel_norm = dist_vel / v_max_goal if v_max_goal else dist_vel
        else:
            dist_vel_norm = 0

        # soft anti-oscillation penalty (see _visit_counts in __init__); only
        # kicks in after more visits than the branching factor explains
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
        weights[5] = 1.0  # remaining time until the goal window (normalized)

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
