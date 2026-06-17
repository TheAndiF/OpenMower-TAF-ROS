OpenMower mower_logic settings MQTT bridge
===========================================

Implemented resource topics
---------------------------

Retained status:
  settings/mower_logic/json

Commands:
  settings/mower_logic/set/session/json
  settings/mower_logic/set/persistent/json
  settings/mower_logic/set/renew/json

Validation feedback:
  settings/mower_logic/validation/json

Session changes
---------------

Payloads are JSON objects keyed by setting name. Each setting entry contains a value field. Example:

  {
    "motor_hot_temperature": {"value": 80.0},
    "rain_delay_minutes": {"value": 45}
  }

Session changes are applied through the existing dynamic_reconfigure service:

  /mower_logic/set_parameters

Persistent changes
------------------

Persistent payloads use the same object-with-value JSON format. They update the persistent value, apply it to the active session immediately and store it in:

  /data/ros/settings_persistent.json

The path follows the shared ROS parameter:

  /settings/persistent_file

The same default/persistent/active model is therefore used as for ll_board and mow_load_factor.

Status payload
--------------

settings/mower_logic/json publishes a schema marker plus a settings object. Each setting includes:

  active
  persistent
  different
  session_apply_supported
  restart_required
  group
  label
  unit
  description
  order
  type
  min / max for numeric settings

The registry includes the complete currently editable mower_logic dynamic_reconfigure setting set.
The frontend may hide or group settings independently without backend changes.

Validation
----------

The backend validates:

  * known registry key
  * JSON type
  * numeric ranges
  * selected cross-field plausibility checks
    - motor_hot_temperature >= motor_cold_temperature
    - mow_load_current_end >= mow_load_current_start
    - mow_load_motor_temp_end >= mow_load_motor_temp_start
    - mow_load_esc_temp_end >= mow_load_esc_temp_start
    - mow_load_factor_smoothing_down_alpha: 0.0 bis 1.0
    - mow_load_factor_smoothing_up_alpha: 0.0 bis 1.0

Batch writes are atomic at the validation level: when validation fails, the request is rejected and not applied.


Zusätzliche Einstellung
-----------------------

  mow_motor_direction_mode
    -1 = feste Richtung reverse/left
     0 = beim Start zwischen beiden Richtungen wechseln
     1 = feste Richtung forward/right

Die tatsächliche mechanische Zuordnung left/right hängt von der Motor-/Board-Verdrahtung ab.

Load-Factor-Glättung
---------------------

Die Lastfaktor-Glättung ist Teil von settings/mower_logic:

  mow_load_factor_smoothing_enabled
    true = berechneten Lastfaktor asymmetrisch tiefpassfiltern
    false = Rohfaktor direkt als computed_factor verwenden

  mow_load_factor_smoothing_down_alpha
    Alpha wenn der Faktor sinkt, z. B. 0.50 fuer schnelle Reaktion bei Last.

  mow_load_factor_smoothing_up_alpha
    Alpha wenn der Faktor steigt, z. B. 0.10 fuer langsame Erholung nach Entlastung.

Beispiel Session-Set:

  Topic: settings/mower_logic/set/session/json
  Payload: {"mow_load_factor_smoothing_enabled":{"value":true},"mow_load_factor_smoothing_down_alpha":{"value":0.50},"mow_load_factor_smoothing_up_alpha":{"value":0.10}}

Path Order Optimizer
--------------------

Der path-order optimizer wird unter settings/mower_logic geführt. Der Node läuft standardmäßig, aber die Optimierung wird über den Wert gesteuert:

  path_order_optimizer_enabled
    false = Optimizer-Service gibt die Slicer-Reihenfolge unverändert zurück
    true  = Optimizer-Service sortiert die Pfade

  path_order_optimizer_processing_mode
    1 = slicer_order
    2 = ordered_fills_plus_ordered_obstacles
    3 = ordered_obstacles_plus_ordered_fills
    4 = mixed_fills_and_obstacles

  path_order_optimizer_outline_entry_mode
    0 = slicer_entry, äußere Area-Outline startet am originalen Slicer-Punkt
    1 = approach_outer_outline_entry, Area-Outlines werden anhand des ersten Schnittpunkts zwischen geplanter Anfahrt und äußerster Outline synchron rotiert; die Anfahrt zielt weiterhin auf den ursprünglichen Startpunkt der innersten Outline

Beispiel:

  Topic: settings/mower_logic/set/session/json
  Payload: {"path_order_optimizer_enabled":{"value":true},"path_order_optimizer_processing_mode":{"value":2},"path_order_optimizer_outline_entry_mode":{"value":1}}

Obstacle-/innere Outlines
-------------------------

Diese Werte gehören zur Outline-/Slicer-Konfiguration und wirken unabhängig vom POO:

  obstacle_outline_count
    Anzahl innerer Hindernis-Outlines.
    -1 = verwende outline_count.

  obstacle_outline_overlap_count
    Innerer Hindernis-Overlap für die Fill-Begrenzung.
    -1 = verwende outline_overlap_count.

Beispiel:

  Topic: settings/mower_logic/set/session/json
  Payload: {"obstacle_outline_count":{"value":1},"obstacle_outline_overlap_count":{"value":0}}

Progressive Outline-Simplification
----------------------------------

Die Glättung gilt für äußere Area-Outlines. Die äußerste Outline bleibt unverändert. Ab der zweiten Outline wird Douglas-Peucker-Simplification verwendet. Bei aktiviertem outline_simplify_affects_next_offset wird die geglättete Ausgabelinie als Basis für die nächste Outline verwendet.

  outline_simplify_per_loop
    Glättung pro Outline-Tiefe in Metern. 0.0 deaktiviert die Funktion vollständig.

  outline_simplify_max_tolerance
    Maximale Douglas-Peucker-Toleranz in Metern.

  outline_simplify_safety_factor
    Sicherheitsfaktor für Überlappung: der Outline-Abstand wird um factor * tolerance reduziert.

  outline_simplify_min_distance_factor
    Untergrenze für den reduzierten Outline-Abstand als Faktor von tool_width.

  outline_simplify_affects_next_offset
    true = geglättete Ausgabe wird Basis für die nächste Outline.

Beispiel:

  Topic: settings/mower_logic/set/session/json
  Payload: {"outline_simplify_per_loop":{"value":0.02},"outline_simplify_max_tolerance":{"value":0.05},"outline_simplify_safety_factor":{"value":2.0},"outline_simplify_min_distance_factor":{"value":0.5},"outline_simplify_affects_next_offset":{"value":true}}
