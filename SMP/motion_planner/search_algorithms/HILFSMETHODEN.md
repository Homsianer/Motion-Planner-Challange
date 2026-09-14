# Hilfsmethoden der `SearchBaseClass`

Diese Übersicht listet alle Hilfsmethoden (helper functions) der Klasse
[`SearchBaseClass`](base_class.py), die als Basisklasse für alle Suchalgorithmen
(`BreadthFirstSearch`, `DepthFirstSearch`, `AStarSearch`, `GreedyBestFirstSearch`,
`UniformCostSearch`, `DepthLimitedSearch`, `StudentMotionPlanner`, ...) dient.

Nicht enthalten sind `__init__`, `parse_planning_problem`, `initialize_lanelets_costs`,
`execute_search` (abstrakt) sowie die reinen Visualisierungs-Methoden
(`plot_solution`, `plot_colliding_primitives`) — das sind Kern-/Infrastrukturmethoden,
keine frei nutzbaren Hilfsmethoden.

---

## 1. Distanzberechnungen (Punkt-zu-Punkt)

Grundlegende Metriken zwischen zwei 2D-Positionen. Werden von `distance()` als
zentraler Dispatcher gebündelt.

| Methode | Beschreibung |
|---|---|
| `euclidean_distance(pos1, pos2)` | Euklidischer Abstand (Luftlinie) zwischen zwei Punkten. Standardmetrik für die meisten Heuristiken. |
| `manhattan_distance(pos1, pos2)` | Manhattan-Distanz (Summe der absoluten Koordinatendifferenzen), entspricht Bewegung auf einem Gitter. |
| `chebyshev_distance(pos1, pos2)` | Chebyshev-Distanz (Maximum der absoluten Koordinatendifferenzen). |
| `canberra_distance(pos1, pos2)` | Canberra-Distanz, eine gewichtete Variante der Manhattan-Distanz, die empfindlicher auf Werte nahe Null reagiert. |
| `cosine_distance(pos1, pos2)` | Kosinus-Distanz (1 − Kosinus-Ähnlichkeit) zwischen zwei Vektoren; misst den Winkelunterschied statt der Länge. |
| `sum_of_squared_difference(pos1, pos2)` | Quadrierte euklidische Distanz (ohne Wurzel) zwischen zwei Punkten. |
| `mean_absolute_error(pos1, pos2)` | Hälfte der Manhattan-Distanz zwischen zwei Punkten. |
| `mean_squared_error(pos1, pos2)` | Hälfte der quadrierten euklidischen Distanz zwischen zwei Punkten. |
| `distance(pos1, pos2, distance_type=0)` | Zentraler Dispatcher: liefert je nach `distance_type` (0–7) eine der obigen Distanzmetriken. `distance_type=0` ist Standard (euklidisch). |

---

## 2. Geometrie auf Polylinien / Lanelet-Mittellinien

Methoden zur Projektion von Positionen auf Linienzüge (z. B. Lanelet-Mittellinien)
und zur Berechnung geometrischer Eigenschaften von Polylinien.

| Methode | Beschreibung |
|---|---|
| `find_closest_vertex(centerVertices, pos)` | Gibt den Index des zu `pos` nächstgelegenen Stützpunkts (Vertex) einer Polylinie zurück. |
| `find_closest_point_on_line(vertexA, vertexB, pos)` | Berechnet die orthogonale Projektion von `pos` auf das Liniensegment `vertexA–vertexB` (geklemmt auf die Segmentenden). |
| `calc_dist_to_closets_point_on_line(vertexA, vertexB, pos)` | Kürzeste Distanz von `pos` zum Liniensegment `vertexA–vertexB`. |
| `calc_distance_to_nearest_point(centerVertices, pos)` | Kürzeste euklidische Distanz von `pos` zu einer gesamten Polylinie (z. B. Lanelet-Mittellinie). |
| `find_nearest_point_on_lanelet(lanelet, pos)` | Findet die Projektion von `pos` auf ein Lanelet sowie die Indizes des vorherigen/nächsten Mittellinien-Vertex. |
| `calc_angle_of_position(centerVertices, pos)` | Berechnet den Winkel (Weltkoordinaten, Radiant) der Linie zwischen den zwei nächstgelegenen Mittellinien-Punkten zu `pos`. |
| `calc_curvature_of_polyline(polyline)` | Berechnet die (aufsummierte absolute) Krümmung einer Polylinie. |
| `calc_length_of_polyline(polyline)` | Berechnet die Gesamtlänge einer Polylinie (Summe der Segmentlängen). |
| `calc_normal_distance_from_line_with_direction(direction_vector, position)` | Berechnet die vorzeichenbehaftete Normalendistanz einer Position zu einer Richtung (nur gültig, wenn `position` senkrecht zu `direction_vector` steht). |

---

## 3. Lanelet-Beziehungen und -Eigenschaften

Methoden, die Nachbarschafts-, Vorgänger-/Nachfolger-Beziehungen sowie
Eigenschaften einzelner Lanelets abfragen.

| Methode | Beschreibung |
|---|---|
| `find_adjacent_lanelets_same_direction(curr_lanelet)` | Gibt die IDs der links/rechts angrenzenden Lanelets zurück, die dieselbe Fahrtrichtung haben. |
| `get_lanelet_width(lanelet, index)` | Berechnet die Breite eines Lanelets an einem gegebenen Vertex-Index (Abstand linker zu rechter Begrenzung). |
| `is_adjacent(start_lanelet_id, final_lanelet_id)` | Prüft, ob `final_lanelet_id` (in gleicher Richtung) links oder rechts neben `start_lanelet_id` liegt. |
| `is_successor(start_lanelet_id, final_lanelet_id)` | Prüft, ob `final_lanelet_id` ein direkter Nachfolger von `start_lanelet_id` ist. |
| `is_goal_in_lane(lanelet_id, traversed_lanelets=None)` | Prüft rekursiv, ob das Ziel-Lanelet über `lanelet_id` oder dessen Nachfolgerkette erreichbar ist. |
| `calc_lanelet_cost(lanelet_current, dist, list_lanelets_visited)` | Rekursive Berechnung der heuristischen Kosten (Anzahl zu durchquerender Lanelets bis zum Ziel) für alle über Vorgänger/Nachbarn erreichbaren Lanelets; befüllt `self.dict_lanelets_costs`. |
| `calc_lanelet_orientation(lanelet_id, pos)` | Gibt die Orientierung (Winkel) eines Lanelets an einer bestimmten Position zurück. |
| `lanelets_of_position(lanelets, state, diff=π/5)` | Filtert aus einer Lanelet-Liste diejenigen heraus, deren Orientierung an der Position von `state` hinreichend gut (Winkeldifferenz < `diff`) mit der Orientierung von `state` übereinstimmt, und sortiert nach Erreichbarkeit des Ziels aus. |

---

## 4. Hindernisse (Obstacles)

Methoden zur Abfrage und Zuordnung von Hindernissen im Szenario.

| Methode | Beschreibung |
|---|---|
| `get_obstacles(laneletObj, time_step)` | Gibt alle Hindernisse zurück, die sich zum Zeitpunkt `time_step` innerhalb des gegebenen Lanelets befinden. |
| `map_obstacles_to_lanelets(time_step)` | Erstellt ein Dictionary `{lanelet_id: [obstacles]}`, das jedem Lanelet die zu diesem Zeitschritt darin befindlichen Hindernisse zuordnet. |
| `num_obstacles_in_lanelet_at_time_step(time_step, lanelet_id)` | Zählt die Anzahl der Hindernisse in einem gegebenen Lanelet zu einem gegebenen Zeitschritt. |
| `calc_dist_to_closest_obstacle(lanelet_id, pos, time_step)` | Berechnet die Distanz von `pos` zum Mittelpunkt des nächstgelegenen Hindernisses (aktuell nur für kreisförmige Hindernisse implementiert) im gegebenen Lanelet. |

---

## 5. Heuristiken und Zielabstand

Methoden, die für die Heuristikfunktionen von informierten Suchalgorithmen
(z. B. A*, Greedy Best-First) relevant sind.

| Methode | Beschreibung |
|---|---|
| `calc_heuristic_distance(state, distance_type=0)` | Distanz (per `distance()`, Standard: euklidisch) zwischen der Position eines Zustands und dem Zentrum der Zielregion. |
| `calc_euclidean_distance(current_node)` | Euklidischer Abstand des Fahrzeugzentrums (nicht Hinterachse) des letzten Pfadzustands eines Knotens zur gewünschten Zielposition (`self.position_desired`), unter Berücksichtigung von Zielintervallen. |
| `calc_heuristic_lanelet(path)` | Summiert für jeden Zustand des Pfads die Distanz zur Mittellinie des zugehörigen Lanelets; liefert zusätzlich Start- und End-Lanelet-ID des Pfads zurück. |
| `calc_angle_to_goal(state)` | Berechnet den Winkel von der Position eines Zustands zur Zielposition. |
| `calc_orientation_diff(orientation_1, orientation_2)` | Berechnet die kleinste Winkeldifferenz zwischen zwei Orientierungen (in Radiant, korrekt über den ±π-Wrap-around hinweg). |
| `calc_goal_interval(vertices)` | Berechnet die achsenparallele Begrenzungsbox (min/max Intervalle in x und y) einer als Vertices gegebenen Zielregion. |

---

## 6. Pfad-/Trajektorienkosten

Methoden zur Bewertung eines fertigen (Teil-)Pfads, z. B. für Kostenfunktionen
in Uniform-Cost-Search oder A*.

| Methode | Beschreibung |
|---|---|
| `calc_travelled_distance(path)` | Summiert die euklidische Distanz zwischen aufeinanderfolgenden Zuständen des Pfads (zurückgelegte Strecke). |
| `calc_time_cost(path)` | Anzahl der Zeitschritte zwischen erstem und letztem Zustand des Pfads. |
| `calc_path_efficiency(path)` | Effizienz des Pfads = zurückgelegte Strecke / benötigte Zeit. |
| `calc_steering_angle_cost(path)` | Summe der absoluten Lenkwinkel (bzw. approximiert über Orientierungsänderung, falls kein `steering_angle` vorhanden) entlang des Pfads. |
| `calc_steering_velocity_cost(path)` | Summe der absoluten Lenkwinkelgeschwindigkeit (bzw. approximiert über Differenz der Lenkwinkel) entlang des Pfads. |
| `calc_acceleration_cost(path)` | Summe der absoluten Beschleunigung (bzw. approximiert über Geschwindigkeitsänderung) entlang des Pfads. |

---

## 7. Zielprüfung (Goal-Test)

Methoden, um zu prüfen, ob und wo ein Pfad die Zielbedingungen erfüllt.

| Methode | Beschreibung |
|---|---|
| `reached_goal(path)` | Prüft für jeden Zustand des Pfads, ob Position (Fahrzeugzentrum), Orientierung, Geschwindigkeit und Zeit gleichzeitig die Zielbedingungen erfüllen. Gibt `True` zurück, sobald ein Zustand das Ziel erfüllt. |
| `remove_states_behind_goal(path)` | Entfernt aus dem letzten Teilpfad alle Zustände, die nach dem ersten zielerfüllenden Zustand liegen (Pfad-Pruning nach Zielerreichung). |

---

## 8. Kollisionsprüfung

| Methode | Beschreibung |
|---|---|
| `is_collision_free(path)` | Prüft, ob der gegebene Pfad (nach Verschiebung der Zustände von der Hinterachse auf das Fahrzeugzentrum) mit Hindernissen oder dem Straßenrand kollidiert. Gibt `True` zurück, wenn keine Kollision vorliegt. |

---

## 9. Pfad-/Primitiven-Verwaltung

Hilfsfunktionen, um Motion Primitives an bestehende Pfade anzuhängen.

| Methode | Beschreibung |
|---|---|
| `translate_primitive_to_current_state(primitive, path_current)` | Transformiert (verschiebt/rotiert) die Trajektorie eines Motion Primitives so, dass sie nahtlos an den letzten Zustand von `path_current` anschließt. |
| `append_path(path_current, newPath)` | Hängt `newPath` an eine Kopie von `path_current` an und gibt den kombinierten Pfad zurück. |

---

### Hinweis
Alle mit `@staticmethod` markierten Methoden benötigen keine Instanz der Klasse
und können direkt über `SearchBaseClass.methodenname(...)` aufgerufen werden.
Die übrigen Methoden greifen auf Instanzattribute zu (z. B. `self.scenario`,
`self.planningProblem`, `self.dict_lanelets_costs`) und müssen auf einer
konkreten Suchalgorithmus-Instanz (`self.methodenname(...)`) aufgerufen werden.
