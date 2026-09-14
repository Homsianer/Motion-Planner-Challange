# Aufgabenstellung erklärt: CommonRoad Search – Motion Planner

Dieses Dokument fasst zusammen, wie das Repository aufgebaut ist, wie CommonRoad
grundsätzlich funktioniert und was du konkret tun musst, um die Übung
"Search-based Motion Planning with Motion Primitives" (IN2406 / IN2062) zu bestehen.

Quelle der offiziellen Aufgabenstellung: `exercise_guide.pdf` (im Repo-Root).
Abgabetermin laut Guide: **23.01.2026, 12:00 Uhr CET** (Moodle).
Ziel: **mindestens 350 von 500 Szenarien** in `scenarios/exercise/` lösen
(ca. 330 sind bereits mit den mitgelieferten Standardplanern lösbar).

---

## 1. Worum geht es überhaupt?

CommonRoad ist ein Benchmark-Framework für automatisiertes Fahren. Ein
**Szenario** besteht aus einer Straßenkarte (Lanelets) und Verkehrsteilnehmern
(Hindernissen). Ein **Planungsproblem** (Planning Problem) definiert für ein
Ego-Fahrzeug einen Startzustand und eine Zielregion. Deine Aufgabe:

> Implementiere eine **Heuristikfunktion** und/oder einen **Suchalgorithmus**
> für einen Motion Planner, der mit **Motion Primitives** kollisionsfreie,
> fahrbare Trajektorien vom Start- zum Zielzustand findet.

Du musst **keinen** Fahrzeugsimulator, keine Kollisionsprüfung und keine
Suchalgorithmen-Infrastruktur selbst schreiben – das ist alles schon da. Du
schreibst im Wesentlichen **eine Bewertungsfunktion**, die entscheidet, welcher
Zustand als nächstes im Suchbaum expandiert wird.

---

## 2. Repo-Struktur (die wichtigen Teile)

```
SMP/
  maneuver_automaton/     -> Motion Primitives & "Maneuver Automaton"
  motion_planner/
    node.py                -> Node / PriorityNode (ein Knoten im Suchbaum)
    queue.py                -> PriorityQueue für die Suche
    search_algorithms/
      base_class.py          -> SearchBaseClass: Grundgerüst + viele Hilfsfunktionen
      best_first_search.py   -> BFS-artiges Grundgerüst + UCS/GBFS/A* Implementierungen
      sequential_search.py   -> BFS, DFS
      depth_limited_search.py-> Depth-Limited Search
      student.py              <-- HIER implementierst du deine Lösung
      student_example.py      -> Beispiel-Implementierung eines Vorgängers
  batch_processing/         -> Löst automatisiert viele Szenarien parallel/seriell
    batch_processing_config.yaml  <-- Konfiguration (Planer auswählen, Pfade, ...)
scenarios/
  exercise/                -> die 500 Szenarien, die du lösen sollst (*.xml)
  tutorial/                -> kleine Szenarien für die Tutorials
tutorials/
  0_commonroad_input-output/  -> CommonRoad-Grundlagen: Szenarien lesen/schreiben/plotten
  1_search_algorithms/        -> Suchalgorithmen + Motion Primitives an einfachen Beispielen
  2_commonroad_search/        -> dasselbe, aber mit echten CommonRoad-Szenarien
  3_motion_primitive_generator/-> wie man eigene Motion Primitives erzeugt
  4_batch_processing/         -> Lösungen für alle Szenarien automatisiert generieren
  5_route_planner/            -> High-Level-Routenplanung als Zusatz-Heuristik
outputs/solutions/          -> hier landen die generierten Lösungs-XML-Dateien
exercise_guide.pdf          -> die offizielle, ausführliche Aufgabenstellung
```

**Empfohlene Lesereihenfolge laut Guide** (Abschnitt 4.6):
1. `SMP/maneuver_automaton/motion_primitive.py`
2. `SMP/maneuver_automaton/maneuver_automaton.py`
3. `SMP/motion_planner/motion_planner.py`
4. `SMP/motion_planner/search_algorithms/`

---

## 3. CommonRoad-Grundkonzepte, die du kennen musst

### 3.1 Szenario (`Scenario`)

- **Lanelet Network**: Das Straßennetz besteht aus **Lanelets** – einzelnen
  Fahrstreifen-Segmenten, definiert durch eine linke und rechte Begrenzung
  (jeweils eine Punktfolge / Polylinie). Lanelets kennen ihre Nachbarn:
  `predecessor`, `successor`, `adj_left`, `adj_right` (inkl. Info, ob die
  Nachbarspur gleiche Fahrtrichtung hat).
- **Obstacles**: statische (parkende Fahrzeuge) und dynamische Hindernisse
  (andere Autos, Fußgänger). Dynamische Hindernisse haben eine **vorgegebene**
  zukünftige Trajektorie (du musst also nicht selbst vorhersagen, wo sie
  hinfahren).

### 3.2 Planungsproblem (`PlanningProblem`)

- **Initial State**: Position `[x, y]`, Steering Angle `δ`, Geschwindigkeit
  `v`, Orientierung `θ ∈ [-π, π]`.
- **Goal Region**: Zielzustände, typischerweise mit
  - Positions-Polygon,
  - Orientierungs-Intervall,
  - Geschwindigkeits-Intervall,
  - Zeit-Intervall (in **Zeitschritten**, nicht Sekunden! `t = k·dt`, meist
    `dt = 0.1s`, d.h. Zeitschritt 61 = 6.1s).
- **Zwei Szenario-Typen**:
  - **Regular**: Ziel hat konkrete Anforderungen an Position/Geschwindigkeit/
    Orientierung/Zeit.
  - **Survival**: Ziel hat nur eine Zeitanforderung – du musst nur
    kollisionsfrei "überleben" bis zu diesem Zeitpunkt.

### 3.3 Was ist eine gültige Lösung?

Eine gültige Trajektorie:
1. beginnt im Initial State,
2. erreicht einen Zustand der Zielregion,
3. ist mit dem **Kinematic Single-Track Modell (KS)** und dem Fahrzeugtyp
   **BMW 320i** fahrbar (wird durch die Motion Primitives automatisch
   sichergestellt, wenn du nur vorgegebene Primitives aneinanderhängst),
4. kollidiert nicht mit Hindernissen und bleibt auf der Fahrbahn.

---

## 4. Motion Primitives & Maneuver Automaton

Ein **Motion Primitive** ist ein kurzes, vorberechnetes, garantiert fahrbares
Trajektoriensegment (in dieser Übung: **0.5 Sekunden** lang), das bei
Position `(0,0)` beginnt und sich in Geschwindigkeit, Lenkwinkel,
Beschleunigung usw. von anderen unterscheidet.

- Zwei Primitives sind **connectable**, wenn Geschwindigkeit und Lenkwinkel
  am Ende von Primitive A gleich Geschwindigkeit/Lenkwinkel am Anfang von
  Primitive B sind.
- Das **Maneuver Automaton** (`SMP/maneuver_automaton/maneuver_automaton.py`)
  ist der Graph aller Primitives inklusive ihrer erlaubten Nachfolger
  (`list_successors`). Fertige Primitive-Sets liegen als `.xml`/`.automaton`
  in `SMP/maneuver_automaton/primitives` bzw. `automata`.
- Durch Verketten von Primitives entsteht ein **Suchbaum**: jeder Knoten ist
  ein erreichter Zustand, jede Kante ist ein Motion Primitive. Die Suche nach
  einer Lösung ist also eine klassische Graphsuche über diesen Baum.
- Falls dir die vorhandenen Primitive-Sets nicht reichen, kannst du mit
  Tutorial 3 (`3_motion_primitive_generator`) eigene generieren.

---

## 5. Das Such-Framework (das steckt hinter `student.py`)

### 5.1 Node / PriorityNode (`SMP/motion_planner/node.py`)

- `Node`: hält `list_paths` (Liste von Teilpfaden, also Zustandslisten),
  `list_primitives` (welche Primitives benutzt wurden) und `depth_tree`.
  `get_successors()` liefert alle möglichen nächsten Primitives.
- `PriorityNode(Node)`: zusätzlich ein `priority`-Wert – das ist der Wert,
  mit dem der Knoten in die Prioritätswarteschlange (`frontier`) einsortiert
  wird.

### 5.2 `SearchBaseClass` (`base_class.py`)

Basisklasse für **alle** Planer. Wichtig für dich:

- Parst das Planning Problem (`parse_planning_problem`) und legt u.a.
  `self.position_desired`, `self.velocity_desired`,
  `self.orientation_desired`, `self.time_desired` an.
- Berechnet für jedes Lanelet die "Distanz" (Anzahl zu durchfahrender
  Lanelets) zum Ziel-Lanelet in `self.dict_lanelets_costs`
  (`-1` = Ziel nicht erreichbar von diesem Lanelet aus, `0` = Ziel-Lanelet).
- Baut einen **Kollisionschecker** (`self.collision_checker`) inkl.
  Fahrbahnrand.
- Stellt jede Menge **fertige Hilfsfunktionen** bereit (musst du nicht
  benutzen, aber sie sparen dir viel Arbeit), u. a.:
  - `calc_heuristic_distance(state)` – euklidische (oder andere) Distanz
    zwischen Zustand und Zielposition.
  - `calc_euclidean_distance(node)` – Distanz vom Fahrzeug-**Mittelpunkt**
    (nicht Hinterachse!) zur Zielregion.
  - `calc_heuristic_lanelet(path)` – Distanz zur Lanelet-Mittellinie entlang
    des Pfades + End-/Start-Lanelet-ID.
  - `calc_orientation_diff`, `calc_angle_to_goal`, `calc_travelled_distance`,
    `calc_time_cost`, `calc_path_efficiency`, `calc_steering_angle_cost`,
    `calc_acceleration_cost`, ...
  - `is_goal_in_lane(lanelet_id)` – prüft rekursiv, ob man vom aktuellen
    Lanelet aus überhaupt das Ziel erreichen kann.
  - `is_collision_free(path)`, `reached_goal(path)`,
    `remove_states_behind_goal(path)`.
  - Diverse Distanzmetriken: `euclidean_distance`, `manhattan_distance`,
    `chebyshev_distance`, `canberra_distance`, `cosine_distance`, ...

  **Wichtig:** Die Referenzposition eines Zustands ist die **Hinterachse**,
  nicht die Fahrzeugmitte. Bei Vergleichen mit der Zielregion (z. B. in
  `calc_euclidean_distance`) wird das intern mit `self.rear_ax_dist`
  korrigiert – falls du eigene Positionsvergleiche baust, beachte das.

### 5.3 `BestFirstSearch` (`best_first_search.py`)

Implementiert die eigentliche Suchschleife `execute_search()`:

1. Start-Zustand wird von Fahrzeugmitte auf Hinterachse verschoben und als
   `PriorityNode` in die `PriorityQueue` (`self.frontier`) eingefügt.
2. **Schleife**, solange die Frontier nicht leer ist:
   - Knoten mit **niedrigster Priorität** wird "gepoppt" (= expandiert).
   - **Goal-Test**: `reached_goal(...)` → wenn ja, Pfad zurückgeben (fertig!).
   - Sonst: für jeden möglichen Nachfolger-Primitive:
     - Primitive an aktuelle Position/Orientierung anhängen
       (`translate_primitive_to_current_state`).
     - **Kollisionscheck** (`is_collision_free`) – bei Kollision wird der
       Kandidat verworfen.
     - Neuen Kind-Knoten erzeugen, **`evaluation_function(node_child)`**
       aufrufen → Ergebnis ist die Priorität, mit der der Knoten in die
       Frontier eingefügt wird.
3. Timeout wird über `SMP.batch_processing.timeout_config` überwacht
   (relevant beim Batch-Processing, siehe unten).

Diese Klasse ist **abstrakt** bzgl. `evaluation_function` – jede konkrete
Suchstrategie definiert, wie `f(n)` berechnet wird:

| Klasse                | Formel                                   | Typ                    |
|------------------------|-------------------------------------------|-------------------------|
| `UniformCostSearch`     | `f(n) = g(n)`                             | uninformiert (Dijkstra) |
| `GreedyBestFirstSearch` | `f(n) = h(n)`                             | informiert, gierig      |
| `AStarSearch`           | `f(n) = g(n) + h(n)`                      | informiert, optimal*    |

`g(n)` ist die bisherige Pfadkosten (hier: vergangene Zeit,
`len(path)-1) * scenario.dt`), `h(n)` ist die **Heuristikfunktion**
(Schätzung der Restkosten zum Ziel). Die Default-`heuristic_function` in
`BestFirstSearch` schätzt grob die verbleibende Zeit
(Distanz-zu-Ziel / aktuelle Geschwindigkeit).

Daneben gibt es noch uninformierte, "reine" Suchverfahren ohne
Priority-Queue: `BreadthFirstSearch`, `DepthFirstSearch`
(`sequential_search.py`) und `DepthLimitedSearch`
(`depth_limited_search.py`) – die expandieren ohne Bewertungsfunktion, sind
aber entsprechend langsamer/ineffizienter auf großen Szenarien.

### 5.4 `motion_planner.py`

Registriert alle Planer-Klassen unter Kürzeln (`bfs`, `dfs`, `ucs`, `gbfs`,
`astar`, `dls`, `student`, `student_example`) und instanziiert sie. Genau
dieses Kürzel (`planner: student`) trägst du später in die
`batch_processing_config.yaml` ein.

---

## 6. Genau hier musst du ran: `student.py`

```python
class StudentMotionPlanner(GreedyBestFirstSearch):
    def evaluation_function(self, node_current: PriorityNode) -> float:
        # todo: deine eigene Bewertungsfunktion f(n)
        pass

    def heuristic_function(self, node_current: PriorityNode) -> float:
        # todo: deine eigene Heuristik h(n)
        pass
```

Du hast laut Aufgabenstellung **zwei Freiheitsgrade**, die du einzeln oder
kombiniert nutzen kannst:

1. **Nur die Heuristik verbessern**: Du lässt die Klasse von
   `GreedyBestFirstSearch` oder `AStarSearch` erben und überschreibst nur
   `heuristic_function` (bei A* automatisch `f = g + h`, bei GBFS `f = h`).
2. **Einen eigenen Suchalgorithmus schreiben**: Du erbst stattdessen direkt
   von `SearchBaseClass` (oder `BestFirstSearch`) und implementierst
   `execute_search` bzw. `evaluation_function` komplett neu – z. B. eine
   Kombination aus mehreren Heuristiken, Beam Search, IDA*, etc.

`SMP/motion_planner/search_algorithms/student_example.py` zeigt eine
konkrete (nicht perfekte, aber funktionierende) Beispielimplementierung
eines früheren Studenten: gewichtete Summe aus

- Distanz zur Lanelet-Mittellinie (`calc_heuristic_lanelet`),
- Orientierungsdifferenz zum Ziel (`calc_orientation_diff`),
- verstrichener Zeit (`calc_time_cost`),
- Distanz zum Ziel (`calc_heuristic_distance`),
- zurückgelegter Strecke,
- Geschwindigkeitsdifferenz zur Zielgeschwindigkeit,

mit manuell gewählten Gewichten (`weights = [8.7, 0.01, 0.5, 0.1, 0.05, 1]`)
und ein paar "harten" Ausschlusskriterien (`np.inf`, wenn z. B. das
Ziel-Lanelet von hier aus unerreichbar ist). Das ist ein guter Startpunkt,
aber die Gewichte sind offensichtlich nicht optimiert – genau das ist deine
Aufgabe.

---

## 7. Tipps zur Heuristikfunktion (aus Kapitel 6 des Guides)

Allgemeine Form:

```
h(state) = Σ_i  weight_i × metric_i(state)
```

Sinnvolle Bestandteile (alle mit passenden Hilfsfunktionen in
`base_class.py` vorhanden):

- **Positionsdistanz** zur Zielregion (euklidisch, Manhattan, ...).
- **Geschwindigkeitsdifferenz** zur Ziel-Geschwindigkeit (Intervallmitte).
- **Orientierungsdifferenz** zur Ziel-Orientierung.
- **Zeitdifferenz** zum Ziel-Zeitfenster.
- **Lanelet-ID / Lanelet-Distanz**: belohne Zustände, die auf dem
  (oder einem Vorgänger-)Lanelet des Ziels liegen; bestrafe Lanelets, von
  denen aus das Ziel gar nicht erreichbar ist (`dict_lanelets_costs == -1`).
- **Hindernisse auf dem Lanelet**: bestrafe Zustände in einem Lanelet mit
  vielen Hindernissen (→ Spurwechsel wird attraktiver).
- **Trajektorien-Effizienz**: zurückgelegte Strecke / benötigte Zeit
  (`calc_path_efficiency`) – belohnt zügiges statt trödelndes Fahren.

Die Gewichte lassen sich kaum analytisch herleiten – **iteratives
Ausprobieren und Feintuning** (z. B. über Batch Processing auf allen 500
Szenarien und Beobachten der Erfolgsquote) ist der übliche Weg.

Zusätzlich gibt es einen **Route Planner** (Tutorial 5), der High-Level-
Routen (Lanelet-Sequenzen) und Referenzpfade vom Start- zum Ziel-Lanelet
berechnet – diese Info kannst du in deine Heuristik einbauen, um die Suche
gezielter Richtung Ziel zu lenken, statt dich nur auf lokale Distanzmaße zu
verlassen.

---

## 8. Praktischer Ablauf – Schritt für Schritt

1. **Tutorials in Reihenfolge durcharbeiten** (Jupyter Notebooks unter
   `tutorials/`):
   - `0_commonroad_input-output`: CommonRoad-XML lesen/schreiben/plotten.
   - `1_search_algorithms`: Motion Primitives + Suchalgorithmen an
     einfachen, synthetischen Beispielen verstehen (BFS/DFS/UCS/GBFS/A*
     visuell nachvollziehen).
   - `2_commonroad_search`: dasselbe an echten CommonRoad-Szenarien.
   - `3_motion_primitive_generator`: nur nötig, falls du eigene Primitives
     bauen willst.
   - `4_batch_processing`: **zentral für die Abgabe** – automatisiertes
     Lösen aller Szenarien.
   - `5_route_planner`: optional, aber hilfreich für bessere Heuristiken.

2. **Code-Struktur verstehen** (siehe Abschnitt 4.6 des Guides / Abschnitt 2
   oben dieses Dokuments).

3. **`student.py` implementieren.** Starte pragmatisch:
   - Kopiere/orientiere dich an `student_example.py`.
   - Wähle als Basisklasse z. B. `GreedyBestFirstSearch` oder `AStarSearch`.
   - Baue deine `heuristic_function` aus den Bausteinen in Abschnitt 7.
   - Behandle Sonderfälle robust (z. B. `None`-Rückgaben von
     `calc_heuristic_lanelet`, Geschwindigkeit 0 bei Division, etc. →
     `np.inf` zurückgeben statt Exception).

4. **Einzelne Szenarien manuell testen** (in Tutorial 2 oder eigenem Skript):
   Szenario laden, `MotionPlanner.create(..., motion_planner_type=MotionPlannerType.STUDENT)`,
   `execute_search()` aufrufen, Ergebnis plotten/prüfen.

5. **`batch_processing_config.yaml` anpassen**:
   - `planner: student` setzen (statt `gbfs`).
   - `input_path` zeigt auf `../../scenarios/exercise/`.
   - `vehicle_model: KS`, `vehicle_type: BMW_320i`, `cost_function: SM1`
     (Vorgabe – nicht ändern, sonst zählt die Submission nicht).
   - `max_tree_depth` und `timeout` ggf. erhöhen, wenn deine Heuristik mehr
     Tiefe/Zeit braucht (Vorsicht: höherer Timeout = langsamerer
     Gesamtdurchlauf über 500 Szenarien).

6. **Batch Processing laufen lassen** (`tutorial_batch_processing.ipynb`,
   parallele oder sequenzielle Zelle) → erzeugt Lösungsdateien in
   `outputs/solutions/`.

7. **Erfolgsquote prüfen**: Wie viele der 500 Szenarien wurden gelöst
   (`validate_solution: True` prüft automatisch Machbarkeit/Kollisionen)?
   Ziel: **≥ 350**. Iterativ Heuristik/Gewichte anpassen und erneut laufen
   lassen.

8. **Lösungen auf die CommonRoad-Website hochladen** (Challenge-Webseite,
   Registrierung + Enrollment in "AI Programming Exercise 2025 - Bonus
   exercise"), Submissions **publishen** (Status muss `Success` **und**
   `Public` sein, sonst zählt es nicht).

9. **Abgabe auf Moodle** bis 23.01.2026, 12:00 Uhr CET, mit:
   - deiner `search_algorithms/student.py`,
   - optional `requirements.txt`, falls du zusätzliche Pakete nutzt,
   - einer `.zip` mit allen gültigen, hochgeladenen Lösungs-XML-Dateien,
   - deiner `batch_processing_config.yaml`,
   - eigenen Motion-Primitive-XML-Dateien, falls selbst generiert.
   - **Nur relative Pfade verwenden**, alles direkt im Abgabeordner (keine
     Unterordner, kein erneutes Zippen einzelner Dateien).

---

## 9. Worauf du besonders achten musst

- **`student.py` muss über `MotionPlannerType.STUDENT` /
  `MotionPlanner.StudentMotionPlanner(...)` aufrufbar bleiben** – Signatur
  von `__init__` nicht verändern (Scenario, PlanningProblem, Automaton,
  Plot-Config).
- **Vehicle Model/Type/Cost Function sind vorgegeben** (KS, BMW 320i, SM1) –
  nicht in der Config ändern, sonst wird die Submission nicht gewertet.
- **Referenzpunkt Hinterachse vs. Fahrzeugmitte**: Bei eigenen
  Positionsvergleichen mit der Zielregion musst du ggf. wie in
  `calc_euclidean_distance`/`reached_goal` um `self.rear_ax_dist` in
  Fahrtrichtung verschieben, sonst ist dein Heuristikwert leicht verzerrt.
- **Numerische Robustheit**: Division durch 0 (z. B. Geschwindigkeit 0),
  `None`-Werte bei nicht gefundenen Lanelets → sauber mit `np.inf` oder
  Sonderfällen abfangen statt Exceptions werfen zu lassen (sonst bricht die
  Suche/Batch-Verarbeitung für dieses Szenario ab).
- **Performance**: Bei 500 Szenarien mit Timeout pro Szenario (Default 30s)
  zählt auch Effizienz deiner Heuristik – zu teure Berechnungen pro Knoten
  (z. B. sehr viele Lanelet-Iterationen) verlangsamen die gesamte Suche.
- **`max_tree_depth` und `timeout`** in der Config: zu niedrig → Lösungen
  werden nicht gefunden, obwohl sie existieren; zu hoch → Batch-Durchlauf
  dauert sehr lange. Guter Kompromiss nötig.
- **Nicht mehr Szenarien lösen als nötig zählt doppelt** – nur die Anzahl
  unterschiedlicher gelöster Szenarien zählt, keine "Mehrfachlösungen".
- **Absolute Pfade vermeiden**, sowohl im Code als auch implizit über eigene
  Notebooks/Skripte, wegen Portabilität bei der Bewertung.
- **DrPlanner** (in Abschnitt 4.4.2 des Guides erwähnt) ist ein optionales
  LLM-gestütztes Tool zur automatischen Diagnose/Reparatur von
  Motion-Plannern – kein Pflichtbestandteil, aber als Hilfsmittel erlaubt.

---

## 10. Kurzreferenz: welche Datei tut was

| Datei | Zweck |
|---|---|
| `SMP/motion_planner/search_algorithms/student.py` | **Deine Implementierung** |
| `SMP/motion_planner/search_algorithms/student_example.py` | Beispiel eines Vorgängers |
| `SMP/motion_planner/search_algorithms/base_class.py` | Basisklasse + Hilfsfunktionen |
| `SMP/motion_planner/search_algorithms/best_first_search.py` | GBFS/A*/UCS-Referenzimplementierungen |
| `SMP/motion_planner/node.py` | `Node`/`PriorityNode`-Datenstrukturen |
| `SMP/motion_planner/motion_planner.py` | Registry aller Planer, Factory-Methode |
| `SMP/maneuver_automaton/` | Motion Primitives & Automaton |
| `SMP/batch_processing/batch_processing_config.yaml` | Konfiguration für Massentest aller Szenarien |
| `tutorials/4_batch_processing/tutorial_batch_processing.ipynb` | Notebook zum Lösungen-Generieren |
| `scenarios/exercise/*.xml` | Die 500 zu lösenden Szenarien |
| `exercise_guide.pdf` | Offizielle, vollständige Aufgabenstellung |
