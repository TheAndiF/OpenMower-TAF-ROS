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

Payloads are flat JSON objects. Example:

  {
    "motor_hot_temperature": 80.0,
    "rain_delay_minutes": 45
  }

Session changes are applied through the existing dynamic_reconfigure service:

  /mower_logic/set_parameters

Persistent changes
------------------

Persistent payloads use the same flat JSON format. They update the persistent value, apply it to the active session immediately and store it in:

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
  Payload: {"mow_load_factor_smoothing_enabled":true,"mow_load_factor_smoothing_down_alpha":0.50,"mow_load_factor_smoothing_up_alpha":0.10}
