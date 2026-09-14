import numpy as np
from commonroad.scenario.state import KSState

from SMP.motion_planner.node import PriorityNode

from SMP.motion_planner.plot_config import DefaultPlotConfig
from SMP.motion_planner.search_algorithms.best_first_search import GreedyBestFirstSearch


class StudentMotionPlanner(GreedyBestFirstSearch):
    """
    Motion planner implementation by students.
    Note that you may inherit from any given motion planner as you wish, or come up with your own planner.
    Here as an example, the planner is inherited from the GreedyBestFirstSearch planner.
    """

    # unterhalb dieser (nicht normierten) Krümmungssumme aus calc_curvature_of_polyline
    # wird die Mittellinie des aktuellen Lanelets als "gerade genug" behandelt.
    # Empirisch zu justieren, da der Wert mit der Vertex-Dichte des Lanelets skaliert.
    CURVATURE_THRESHOLD = 0.05

    def __init__(
        self, scenario, planningProblem, automata, plot_config=DefaultPlotConfig
    ):
        super().__init__(
            scenario=scenario,
            planningProblem=planningProblem,
            automaton=automata,
            plot_config=plot_config,
        )
        # Krümmung ändert sich während der Suche nicht (statische Straßengeometrie),
        # daher wird sie pro Lanelet nur einmal berechnet und danach wiederverwendet.
        self._lanelet_curvature_cache = {}

    def _get_lanelet_curvature(self, lanelet_id: int) -> float:
        """
        Gibt die (gecachte) Krümmung der Mittellinie des gegebenen Lanelets zurück.
        """
        if lanelet_id not in self._lanelet_curvature_cache:
            lanelet_obj = self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            self._lanelet_curvature_cache[lanelet_id] = self.calc_curvature_of_polyline(
                lanelet_obj.center_vertices
            )
        return self._lanelet_curvature_cache[lanelet_id]

    def calc_distance_to_goal(self, state: KSState) -> float:
        """
        Berechnet eine realistische Distanz zwischen dem gegebenen Zustand und der Zielregion,
        auch wenn diese in einem anderen Lanelet liegt.

        Laufzeitdispatch: Das alleinige Kriterium ist die Krümmung der Mittellinie des
        Ziellanelets (siehe CURVATURE_THRESHOLD), unabhängig davon, ob sich der
        Zustand bereits im Ziel-Lanelet befindet oder nicht.

        - Ist die Straße dort annähernd gerade, wird die günstige euklidische Distanz
          zurückgegeben.
        - Andernfalls wird die reale Bogenlänge entlang der Lanelet-Mittellinie(n)
          berechnet: falls Start und Ziel im selben Lanelet liegen, als direkte
          Bogenlänge zwischen den beiden Projektionspunkten; ansosnten

        :param state: der Zustand, dessen Distanz zum Ziel berechnet werden soll
        """
        dist_euclidean = self.calc_heuristic_distance(state)
        if dist_euclidean is None:
            return np.inf

        list_lanelets_current = self.scenario.lanelet_network.find_lanelet_by_position(
            [state.position]
        )[0] #
        if not list_lanelets_current:
            return dist_euclidean
        lanelet_id_current = list_lanelets_current[0]

        # einziges Dispatch-Kriterium: Krümmung des Ziellanelets
        if self._get_lanelet_curvature(lanelet_id_current) < self.CURVATURE_THRESHOLD:
            return dist_euclidean

        cost_current = self.dict_lanelets_costs.get(lanelet_id_current, -1)
        if cost_current == -1:
            # Ziel ist von hier aus nicht erreichbar
            return np.inf

        lanelet_obj_current = self.scenario.lanelet_network.find_lanelet_by_id(
            lanelet_id_current
        )
        _, index_prev, _ = self.find_nearest_point_on_lanelet(
            lanelet_obj_current, state.position
        )
        if index_prev is None:
            index_prev = 0

        if lanelet_id_current in self.list_ids_lanelets_goal:
            # Ziel liegt im selben (gekrümmten) Lanelet -> direkte Bogenlänge zwischen
            # den beiden Projektionspunkten statt Graph-Traversierung
            goal_center = np.array(
                [
                    (self.position_desired[0].start + self.position_desired[0].end)
                    / 2.0,
                    (self.position_desired[1].start + self.position_desired[1].end)
                    / 2.0,
                ]
            )
            _, index_prev_goal, _ = self.find_nearest_point_on_lanelet(
                lanelet_obj_current, goal_center
            )
            if index_prev_goal is None:
                index_prev_goal = len(lanelet_obj_current.center_vertices) - 1
            index_start, index_end = sorted((index_prev, index_prev_goal))
            dist_route = self.calc_length_of_polyline(
                lanelet_obj_current.center_vertices[index_start : index_end + 1]
            )
            return max(dist_route, dist_euclidean)

        # Ziel liegt in einem anderen Lanelet -> reale Bogenlänge entlang der Route aufsummieren
        dist_route = self.calc_length_of_polyline(
            lanelet_obj_current.center_vertices[index_prev:]
        )

        lanelet_id_walk = lanelet_id_current
        while cost_current > 0:
            lanelet_obj_walk = self.scenario.lanelet_network.find_lanelet_by_id(
                lanelet_id_walk
            )
            candidates = list(lanelet_obj_walk.successor or [])
            if (
                lanelet_obj_walk.adj_left is not None
                and lanelet_obj_walk.adj_left_same_direction
            ):
                candidates.append(lanelet_obj_walk.adj_left)
            if (
                lanelet_obj_walk.adj_right is not None
                and lanelet_obj_walk.adj_right_same_direction
            ):
                candidates.append(lanelet_obj_walk.adj_right)

            lanelet_id_next = next(
                (
                    c
                    for c in candidates
                    if self.dict_lanelets_costs.get(c, -1) == cost_current - 1
                ),
                None,
            )
            if lanelet_id_next is None:
                # Route konnte nicht eindeutig rekonstruiert werden -> auf euklidisch zurückfallen
                return dist_euclidean

            lanelet_obj_next = self.scenario.lanelet_network.find_lanelet_by_id(
                lanelet_id_next
            )
            if lanelet_id_next in self.list_ids_lanelets_goal:
                # letztes Teilstück: nur bis zur Zielposition, nicht die volle Lanelet-Länge
                goal_center = np.array(
                    [
                        (self.position_desired[0].start + self.position_desired[0].end)
                        / 2.0,
                        (self.position_desired[1].start + self.position_desired[1].end)
                        / 2.0,
                    ]
                )
                _, index_prev_goal, _ = self.find_nearest_point_on_lanelet(
                    lanelet_obj_next, goal_center
                )
                if index_prev_goal is not None:
                    dist_route += self.calc_length_of_polyline(
                        lanelet_obj_next.center_vertices[: index_prev_goal + 1]
                    )
            else:
                dist_route += self.calc_length_of_polyline(
                    lanelet_obj_next.center_vertices
                )

            lanelet_id_walk = lanelet_id_next
            cost_current = self.dict_lanelets_costs[lanelet_id_walk]

        return max(dist_route, dist_euclidean)

    def evaluation_function(self, node_current: PriorityNode) -> float:
        ########################################################################
        # todo: Implement your own evaluation function here.                   #
        ########################################################################
        pass

    def heuristic_function(self, node_current: PriorityNode) -> float:
        ########################################################################
        # todo: Implement your own heuristic cost calculation here.            #
        # Hint:                                                                #
        #   Use the State of the current node and the information from the     #
        #   planning problem, as well as from the scenario.                    #
        #   Some helper functions for your convenience can be found in         #
        #   ./search_algorithms/base_class.py                             #
        ########################################################################
        pass
