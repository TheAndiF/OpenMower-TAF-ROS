POO execution.path refactor
===========================

Ziel
----
Der currentMowingPlan enthaelt pro Pfad weiterhin die unveraenderte Slicer-Quelle und zusaetzlich einen fertigen Ausfuehrungspfad:

paths[].slicer_source.path_id  = originale Slicer-Pfad-ID / urspruengliche Reihenfolge
paths[].slicer_source.path     = unveraenderte Slicer-Geometrie
paths[].execution.path         = real zu fahrender Pfad

Abarbeitungsregel
-----------------
Die Mower-Logic faehrt ausschliesslich paths[].execution.path. path_direction ist damit Metadatum und wird nicht mehr waehrend der Fahrt zur Umrechnung der Pose-Reihenfolge benutzt.

Standardfall
------------
Bei FORWARD ist execution.path eine Kopie von slicer_source.path.

POO Reverse
-----------
Wenn der POO einen Pfad reversed zurueckgibt, wird optimizeSrv.response.paths[i] als execution.path in den MowingPlan uebernommen. Der POO dreht also die Pose-Reihenfolge und berechnet die Orientierungen neu. slicer_source bleibt unveraendert.

Snapshot
--------
Der JSON-Snapshot schreibt jetzt slicer_source.path_id und execution.path. Alte Snapshots mit slicer_source.path_index werden weiterhin gelesen; fehlt execution.path, wird er aus slicer_source.path und path_direction rekonstruiert.

Geaenderte Dateien
------------------
- src/mower_logic/src/mower_logic/behaviors/MowingBehavior.h
- src/mower_logic/src/mower_logic/behaviors/MowingBehavior.cpp

Hinweis
-------
Ein Build konnte in dieser Sandbox nicht ausgefuehrt werden, weil keine ROS/catkin-Umgebung verfuegbar ist. git diff --check fuer die geaenderten Dateien war erfolgreich.
