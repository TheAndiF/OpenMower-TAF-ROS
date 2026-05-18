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

Persistent payloads use the same flat JSON format, but they only update YAML and do not alter the active session.
The YAML target defaults to:

  $PARAMS_PATH/mower_params.yaml

or, if PARAMS_PATH is not defined:

  /data/ros/mower_params.yaml

The path can be overridden with the private ROS parameter:

  ~mower_logic_settings_yaml_path

YAML writes use a temporary file, a .bak backup of the previous file, and an atomic rename.

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

Batch writes are atomic at the validation level: when validation fails, the request is rejected and not applied.
