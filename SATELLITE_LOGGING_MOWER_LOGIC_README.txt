OpenMower MQTT Satellite Logging - mower_logic integration

Canonical placement
-------------------
Satellite logging is handled under mower_logic only. There are no active compatibility topics for satellite_logger/... and no separate settings/satellite_logging namespace.

Settings/defaults
-----------------
The persistent/default options are regular settings/mower_logic entries. They are published with all other mower_logic settings:

  settings/mower_logic/json

They are changed through the normal mower_logic settings endpoints:

  settings/mower_logic/set/session/json
  settings/mower_logic/set/persistent/json
  settings/mower_logic/set/renew/json

Relevant keys in the settings object:

  satellite_logging_enabled
  satellite_logging_default_trigger
  satellite_logging_default_mode
  satellite_logging_script_path
  satellite_logging_ram_path
  satellite_logging_output_path
  satellite_logging_container_name

Runtime control
---------------
The runtime start/stop control is below mower_logic as well:

  mower_logic/satellite_logging/set/control/json
  mower_logic/satellite_logging/set/renew/json

Examples:

  {"command":"start","trigger":"next_cycle","mode":"from_start_to_docking"}
  {"command":"start","trigger":"ad_hoc","mode":"until_docking"}
  {"command":"stop"}
  {"command":"cancel"}

Runtime status
--------------
The current status is published as a mower_logic current value:

  mower_logic/satellite_logging/json

The status contains state, trigger, mode, armed/running, session_id, timestamps, generated filenames, RAM path, output path and error.

Responsibility split
--------------------
settings/mower_logic: persistent defaults and software switch.
mower_logic runtime control/status: arm/start/stop/renew.
mower_logic node: decides start and end from mower state.
record_satellites.sh: records into RAM and flushes to persistent storage when terminated.
