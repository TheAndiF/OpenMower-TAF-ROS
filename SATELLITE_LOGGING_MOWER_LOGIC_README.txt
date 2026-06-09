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
  satellite_logging_default_area_id
  satellite_logging_script_path
  satellite_logging_ram_path
  satellite_logging_output_path
  satellite_logging_container_name

Enable-switch behavior
----------------------
satellite_logging_enabled is the normal user-facing start/stop switch.

When satellite_logging_enabled becomes true, mower_logic checks the configured script path, makes the script executable if needed, and then starts or arms logging according to the currently configured MQTT settings:

  satellite_logging_default_trigger
  satellite_logging_default_mode
  satellite_logging_default_area_id

When satellite_logging_enabled becomes false, mower_logic stops a running log process and cancels any armed logging request.

When the configured end condition is reached, mower_logic stops the log process and automatically sets satellite_logging_enabled back to false so the app/MQTT setting visibly returns to Off.

Start variants
--------------
  ad_hoc
    Start immediately when satellite_logging_enabled is switched on.

  next_cycle
    Arm logging and start automatically on the next matching mowing cycle.

  area_id
    Arm logging and start when the current mowing area matches satellite_logging_default_area_id.

End/mode variants
-----------------
  until_docking
    Keep logging until the mower docks.

  from_start_to_docking
    Start with active mowing work and stop when the mower docks.

  from_docking_to_docking
    Start when the mower leaves docking and stop when the mower docks again.

Expert/runtime paths
--------------------
  satellite_logging_script_path
    Script started by mower_logic. Default:
    /home/openmower/scripts/record_satellites.sh

  satellite_logging_ram_path
    RAM directory for live logs. Default:
    /dev/shm/openmower_satellite_logs

  satellite_logging_output_path
    Persistent output directory. Default:
    /home/openmower/recordings/logs

  satellite_logging_container_name
    Optional ROS container override. Leave empty for auto-detection.
    If empty, record_satellites.sh runs directly when already inside a container. On a host with Docker, it tries to find the ROS container automatically, preferring names such as openmower-open_mower_ros-1 or OpenMowerROS.

Runtime control
---------------
The runtime control endpoint remains available for advanced/manual control:

  mower_logic/satellite_logging/set/control/json
  mower_logic/satellite_logging/set/renew/json

Examples:

  {"command":"start","trigger":"next_cycle","mode":"from_start_to_docking"}
  {"command":"start","trigger":"ad_hoc","mode":"until_docking"}
  {"command":"start","trigger":"area_id","mode":"until_docking","area_id":"3"}
  {"command":"stop"}
  {"command":"cancel"}

Normal app usage should use satellite_logging_enabled and the default settings; the control endpoint is not required for ordinary start/stop operation.

Runtime status
--------------
The current status is published as a mower_logic current value:

  mower_logic/satellite_logging/json

The status contains state, trigger, mode, target_area_id, armed/running, session_id, timestamps, generated filenames, RAM path, output path, script path, container override and error.

Responsibility split
--------------------
settings/mower_logic: user-facing switch, defaults and expert paths.
mower_logic node: validates script path, starts/arms/stops logging, decides start and end from mower state.
record_satellites.sh: auto-detects the ROS container when needed, records into RAM and flushes to persistent storage when terminated.
