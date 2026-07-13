MQTT GPS Logging
================

Canonical API only
------------------
GPS logging is exposed to the app exclusively below gps_state:

- gps_state/settings/json
- gps_state/settings/set/session/json
- gps_state/settings/set/persistent/json
- gps_state/settings/set/renew/json
- gps_state/logging/set/control/json
- gps_state/logging/set/renew/json
- gps_state/logging/status/json
- gps_state/logging/last/json
- gps_state/logging/validation/json

No MQTT alias below settings/mower_logic/satellite_logging is subscribed or
published. App implementations must use the gps_state topics directly.

Start, stop and cancel are commands. Do not implement them as a persistent
boolean switch. The retained runtime status is the source of truth for the app.

Control examples
----------------

  {"command":"start","trigger":"next_cycle","mode":"from_start_to_docking"}
  {"command":"start","trigger":"ad_hoc","mode":"until_docking"}
  {"command":"start","trigger":"area_id","mode":"until_docking","area_id":"3"}
  {"command":"stop"}
  {"command":"cancel"}

Configuration
-------------
The logging group in gps_state/settings/json contains defaults and expert paths:

- logging_default_trigger
- logging_default_mode
- logging_default_area_id
- logging_script_path
- logging_ram_path
- logging_output_path
- logging_container_name

Runtime schema
--------------
gps_state/logging/status/json uses openmower.gps_state.logging.v2. Runtime data
is available only in the structured runtime, request, storage and implementation
objects. Removed compatibility fields are not duplicated at the top level.

See:

  src/lib/xbot_monitoring/GPS_LOGGING_API.md
  SATELLITE_LOGGING_MOWER_LOGIC_README.txt
